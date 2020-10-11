// ===================== Mode S detection and decoding  ===================

use std::convert::TryFrom;
use std::ffi::CStr;
use std::io::Write;
use std::os::raw::{c_char, c_double, c_int, c_uchar, c_uint, c_ulong};
use std::{io, ptr, str};

use crate::interactive::interactive_receive_data;
use crate::mode_ac::{decode_mode_a_message, detect_mode_a, mode_a_to_mode_c, MODEAC_MSG_SAMPLES};
use crate::{
    Aircraft, Altitude, ModeS, ModesMessage, MODES_ACFLAGS_ALTITUDE_VALID,
    MODES_ACFLAGS_EWSPEED_VALID, MODES_ACFLAGS_HEADING_VALID, MODES_ACFLAGS_LATLON_REL_OK,
    MODES_ACFLAGS_LATLON_VALID, MODES_ACFLAGS_NSSPEED_VALID, MODES_ACFLAGS_SPEED_VALID,
    MODES_ACFLAGS_VERTRATE_VALID, MODES_ASYNC_BUF_SAMPLES, MODES_DEBUG_BADCRC, MODES_DEBUG_DEMOD,
    MODES_DEBUG_DEMODERR, MODES_DEBUG_GOODCRC, MODES_DEBUG_NOPREAMBLE,
    MODES_DEBUG_NOPREAMBLE_LEVEL, MODES_LONG_MSG_BITS, MODES_LONG_MSG_BYTES,
    MODES_LONG_MSG_SAMPLES, MODES_LONG_MSG_SIZE, MODES_MAX_BITERRORS, MODES_MSG_ENCODER_ERRS,
    MODES_MSG_SQUELCH_LEVEL, MODES_PREAMBLE_SAMPLES, MODES_PREAMBLE_SIZE, MODES_PREAMBLE_US,
    MODES_SHORT_MSG_BITS, MODES_USER_LATITUDE_DFLT, MODES_USER_LATLON_VALID,
    MODES_USER_LONGITUDE_DFLT,
};

// Parity table for MODE S Messages.
// The table contains 112 elements, every element corresponds to a bit set
// in the message, starting from the first bit of actual data after the
// preamble.
//
// For messages of 112 bit, the whole table is used.
// For messages of 56 bits only the last 56 elements are used.
//
// The algorithm is as simple as xoring all the elements in this table
// for which the corresponding bit on the message is set to 1.
//
// The latest 24 elements in this table are set to 0 as the checksum at the
// end of the message should not affect the computation.
//
// Note: this function can be used with DF11 and DF17, other modes have
// the CRC xored with the sender address as they are reply to interrogations,
// but a casual listener can't split the address from the checksum.
const MODES_CHECKSUM_TABLE: [u32; 112] = [
    0x3935ea, 0x1c9af5, 0xf1b77e, 0x78dbbf, 0xc397db, 0x9e31e9, 0xb0e2f0, 0x587178, 0x2c38bc,
    0x161c5e, 0x0b0e2f, 0xfa7d13, 0x82c48d, 0xbe9842, 0x5f4c21, 0xd05c14, 0x682e0a, 0x341705,
    0xe5f186, 0x72f8c3, 0xc68665, 0x9cb936, 0x4e5c9b, 0xd8d449, 0x939020, 0x49c810, 0x24e408,
    0x127204, 0x093902, 0x049c81, 0xfdb444, 0x7eda22, 0x3f6d11, 0xe04c8c, 0x702646, 0x381323,
    0xe3f395, 0x8e03ce, 0x4701e7, 0xdc7af7, 0x91c77f, 0xb719bb, 0xa476d9, 0xadc168, 0x56e0b4,
    0x2b705a, 0x15b82d, 0xf52612, 0x7a9309, 0xc2b380, 0x6159c0, 0x30ace0, 0x185670, 0x0c2b38,
    0x06159c, 0x030ace, 0x018567, 0xff38b7, 0x80665f, 0xbfc92b, 0xa01e91, 0xaff54c, 0x57faa6,
    0x2bfd53, 0xea04ad, 0x8af852, 0x457c29, 0xdd4410, 0x6ea208, 0x375104, 0x1ba882, 0x0dd441,
    0xf91024, 0x7c8812, 0x3e4409, 0xe0d800, 0x706c00, 0x383600, 0x1c1b00, 0x0e0d80, 0x0706c0,
    0x038360, 0x01c1b0, 0x00e0d8, 0x00706c, 0x003836, 0x001c1b, 0xfff409, 0x000000, 0x000000,
    0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
    0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
    0x000000, 0x000000, 0x000000, 0x000000,
];

//
//=========================================================================
//
// Code for introducing a less CPU-intensive method of correcting
// single bit errors.
//
// Makes use of the fact that the crc checksum is linear with respect to
// the bitwise xor operation, i.e.
//      crc(m^e) = (crc(m)^crc(e)
// where m and e are the message resp. error bit vectors.
//
// Call crc(e) the syndrome.
//
// The code below works by precomputing a table of (crc(e), e) for all
// possible error vectors e (here only single bit and double bit errors),
// search for the syndrome in the table, and correct the then known error.
// The error vector e is represented by one or two bit positions that are
// changed. If a second bit position is not used, it is -1.
//
// Run-time is binary search in a sorted table, plus some constant overhead,
// instead of running through all possible bit positions (resp. pairs of
// bit positions).
//
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
#[repr(C)]
pub struct ErrorInfo {
    pub syndrome: u32,
    pub bits: u8,
    pub pos: [i8; 2],
}

// TODO: capture the size of the message in a type. E.g. enum Message { Short(7 bytes), Long(14 bytes) }
pub(crate) fn mode_s_checksum(msg: [u8; MODES_LONG_MSG_BYTES], mut bits: c_int) -> u32 {
    // extract checksum supplied with message, 24-bits, last 3 bytes
    let rem = if bits == 112 {
        (msg[11] as u32) << 16 | (msg[12] as u32) << 8 | msg[13] as u32
    } else {
        (msg[4] as u32) << 16 | (msg[5] as u32) << 8 | msg[6] as u32
    };
    let mut crc: u32 = 0;
    let mut offset = if bits == 112 { 0 } else { 112 - 56 };
    let mut byte_offset = 0;
    let mut the_byte: u8 = msg[0];
    let mut bit: c_int = 0;

    // We don't include the checksum itself
    bits -= 24;
    while bit < bits {
        if bit & 7 == 0 {
            the_byte = msg[byte_offset];
            byte_offset += 1;
        }

        // If bit is set, xor with corresponding table entry.
        if the_byte & 0x80 != 0 {
            crc ^= MODES_CHECKSUM_TABLE[offset] // FIXME: bounds?
        }

        offset += 1;
        the_byte = the_byte << 1;
        bit += 1
    }

    (crc ^ rem) & 0xffffff // 24 bit checksum syndrome.
}

// Given the Downlink Format (DF) of the message, return the message length in bits.
//
// All known DF's 16 or greater are long. All known DF's 15 or less are short.
// There are lots of unused codes in both category, so we can assume ICAO will stick to
// these rules, meaning that the most significant bit of the DF indicates the length.
fn mode_s_message_len_by_type(type_: c_int) -> c_int {
    if type_ & 0x10 == 0x10 {
        MODES_LONG_MSG_BITS
    } else {
        MODES_SHORT_MSG_BITS
    }
}

// Search for syndrome in table and if an entry is found, flip the necessary
// bits. Make sure the indices fit into the array
// Additional parameter: fix only less than maxcorrected bits, and record
// fixed bit positions in corrected[]. This array can be NULL, otherwise
// must be of length at least maxcorrected.
// Return number of fixed bits.
//
fn fix_bit_errors(mm: &mut ModesMessage, maxfix: c_int, bit_error_table: &[ErrorInfo]) {
    let bits = i8::try_from(mm.msgbits).unwrap();
    let maxfix = u8::try_from(maxfix).unwrap();
    let syndrome = mode_s_checksum(mm.msg, mm.msgbits);
    let ei = match bit_error_table.binary_search_by(|e| e.syndrome.cmp(&syndrome)) {
        Ok(index) => &bit_error_table[index],
        Err(_) => {
            // No syndrome found
            mm.correctedbits = 0;
            return;
        }
    };

    // TODO: Clean up this closure workaround
    mm.correctedbits = (|| {
        let mut bitpos;

        // Check if the syndrome fixes more bits than we allow
        if maxfix < ei.bits {
            return 0;
        }

        // Check that all bit positions lie inside the message length
        let offset = i8::try_from(MODES_LONG_MSG_BITS).unwrap() - bits;
        let mut i = 0;
        while i < ei.bits {
            bitpos = ei.pos[i as usize] - offset;
            if bitpos < 0 || bitpos >= bits {
                return 0 as c_int;
            }
            i += 1
        }

        // Fix the bits
        let mut res = 0;
        i = res;
        while i < ei.bits {
            bitpos = ei.pos[i as usize] - offset;
            let ref mut fresh1 = mm.msg[(bitpos >> 3) as usize];
            *fresh1 ^= 1 << 7 - (bitpos & 7);
            let fresh2 = res;
            res = res + 1;
            mm.corrected[fresh2 as usize] = bitpos as c_char;
            i += 1
        }
        return c_int::from(res);
    })();
}

// In the squawk (identity) field bits are interleaved as follows in
// (message bit 20 to bit 32):
//
// C1-A1-C2-A2-C4-A4-ZERO-B1-D1-B2-D2-B4-D4
//
// So every group of three bits A, B, C, D represent an integer from 0 to 7.
//
// The actual meaning is just 4 octal numbers, but we convert it into a hex
// number tha happens to represent the four octal numbers.
//
// For more info: http://en.wikipedia.org/wiki/Gillham_code
//
#[rustfmt::skip]
fn decode_id13_field(id13_field: c_int) -> c_int {
    let mut hex_gillham = 0;
    if id13_field & 0x1000 != 0 { hex_gillham |= 0x0010 } // Bit 12 = C1
    if id13_field & 0x0800 != 0 { hex_gillham |= 0x1000 } // Bit 11 = A1
    if id13_field & 0x0400 != 0 { hex_gillham |= 0x0020 } // Bit 10 = C2
    if id13_field & 0x0200 != 0 { hex_gillham |= 0x2000 } // Bit  9 = A2
    if id13_field & 0x0100 != 0 { hex_gillham |= 0x0040 } // Bit  8 = C4
    if id13_field & 0x0080 != 0 { hex_gillham |= 0x4000 } // Bit  7 = A4
    // TODO: Find out why bit 6 was commented out in the C code
    // if (id13_field & 0x0040) {hex_gillham |= 0x0800;}  // Bit  6 = X  or M
    if id13_field & 0x0020 != 0 { hex_gillham |= 0x0100 } // Bit  5 = B1
    if id13_field & 0x0010 != 0 { hex_gillham |= 0x0001 } // Bit  4 = D1 or Q
    if id13_field & 0x0008 != 0 { hex_gillham |= 0x0200 } // Bit  3 = B2
    if id13_field & 0x0004 != 0 { hex_gillham |= 0x0002 } // Bit  2 = D2
    if id13_field & 0x0002 != 0 { hex_gillham |= 0x0400 } // Bit  1 = B4
    if id13_field & 0x0001 != 0 { hex_gillham |= 0x0004 } // Bit  0 = D4
    hex_gillham
}

// Decode the 13 bit AC altitude field (in DF 20 and others).
// Returns the altitude, and set 'unit' to either MODES_UNIT_METERS or MDOES_UNIT_FEETS.
//
fn decode_ac13_field(ac13_field: c_int, unit: &mut Altitude) -> c_int {
    let m_bit = (ac13_field & 0x40) != 0; // set = meters, clear = feet
    let q_bit = (ac13_field & 0x10) != 0; // set = 25 ft encoding, clear = Gillham Mode C encoding
    if !m_bit {
        *unit = Altitude::Feet;
        if q_bit {
            // N is the 11 bit integer resulting from the removal of bit Q and M
            let n: c_int = (ac13_field & 0x1f80) >> 2 | (ac13_field & 0x20) >> 1 | ac13_field & 0xf;
            // The final altitude is resulting number multiplied by 25, minus 1000.
            n * 25 - 1000
        } else {
            // N is an 11 bit Gillham coded altitude
            let mut n_0: c_int = mode_a_to_mode_c(decode_id13_field(ac13_field) as c_uint);
            if n_0 < -12 {
                n_0 = 0
            }
            100 * n_0
        }
    } else {
        // TODO(inherited): Implement altitude when meter unit is selected
        *unit = Altitude::Metres;
        0
    }
}

// Decode the 12 bit AC altitude field (in DF 17 and others).
//
fn decode_ac12_field(ac12_field: c_int, unit: &mut Altitude) -> c_int {
    let q_bit = (ac12_field & 0x10) != 0; // Bit 48 = Q
    *unit = Altitude::Feet;
    if q_bit {
        // / N is the 11 bit integer resulting from the removal of bit Q at bit 4
        let n: c_int = (ac12_field & 0xfe0) >> 1 | ac12_field & 0xf;
        // The final altitude is the resulting number multiplied by 25, minus 1000.
        n * 25 - 1000
    } else {
        // Make N a 13 bit Gillham coded altitude by inserting M=0 at bit 6
        let mut n_0: c_int = (ac12_field & 0xfc0) << 1 | ac12_field & 0x3f;
        n_0 = mode_a_to_mode_c(decode_id13_field(n_0) as c_uint);
        if n_0 < -12 {
            n_0 = 0
        }
        100 * n_0
    }
}

// FIXME: this function has no test coverage
// Decode the 7 bit ground movement field PWL exponential style scale
//
fn decode_movement_field(movement: c_int) -> c_int {
    // Note: movement codes 0,125,126,127 are all invalid, but they are
    //       trapped before this function is called.
    // FIXME: Capture above in types
    match movement {
        _ if movement > 123 => 199, // > 175kt
        _ if movement > 108 => (movement - 108) * 5 + 100,
        _ if movement > 93 => (movement - 93) * 2 + 70,
        _ if movement > 38 => movement - 38 + 15,
        _ if movement > 12 => (movement - 11 >> 1) + 2,
        _ if movement > 8 => (movement - 6 >> 2) + 1,
        _ => 0,
    }
}

// In AIS characters are encoded using 6-bits giving 64 possible values
const AIS_CHARSET: &[u8; 64] = b"?ABCDEFGHIJKLMNOPQRSTUVWXYZ????? ???????????????0123456789??????";

// Decode a raw Mode S message demodulated as a stream of bytes by detectModeS(),
// and split it into fields populating a ModesMessage structure.
//
fn decode_mode_s_message(
    mm: &mut ModesMessage,
    msg: [u8; MODES_LONG_MSG_BYTES],
    mode_s: &mut ModeS, // FIXME: This is mutable only for icao_cache
    bit_errors: &[ErrorInfo],
) {
    // Copy the raw data onto the message, it may be modified by fix_bit_errors
    mm.msg = msg;

    // Get the message type ASAP as other operations depend on this
    mm.msgtype = c_int::from(mm.msg[0]) >> 3; // Downlink Format
    mm.msgbits = mode_s_message_len_by_type(mm.msgtype);
    mm.crc = mode_s_checksum(mm.msg, mm.msgbits);

    if mm.crc != 0 && mode_s.nfix_crc != 0 && (mm.msgtype == 17 || mm.msgtype == 18) {
        //  if ((mm->crc) && (Modes.nfix_crc) && ((mm->msgtype == 11) || (mm->msgtype == 17))) {
        //
        // Fixing single bit errors in DF-11 is a bit dodgy because we have no way to
        // know for sure if the crc is supposed to be 0 or not - it could be any value
        // less than 80. Therefore, attempting to fix DF-11 errors can result in a
        // multitude of possible crc solutions, only one of which is correct.
        //
        // We should probably perform some sanity checks on corrected DF-11's before
        // using the results. Perhaps check the ICAO against known Aircraft, and check
        // IID against known good IID's. That's a TODO.
        //
        fix_bit_errors(mm, mode_s.nfix_crc, bit_errors);

        // If we correct, validate ICAO addr to help filter birthday paradox solutions.
        if mm.correctedbits != 0 {
            let ul_addr: u32 = ((mm.msg[1] as c_int) << 16 as c_int
                | (mm.msg[2] as c_int) << 8 as c_int
                | mm.msg[3] as c_int) as u32;
            if mode_s.icao_cache.address_was_recently_seen(ul_addr) == 0 {
                mm.correctedbits = 0;
            }
        }
    }

    // Note that most of the other computation happens *after* we fix the
    // single/two bit errors, otherwise we would need to recompute the fields again.
    match mm.msgtype {
        11 => {
            // DF 11
            mm.iid = mm.crc as c_int; // Responder capabilities, zero indicates no communications capability (surveillance only)
            mm.addr = ((mm.msg[1] as c_int) << 16 as c_int
                | (mm.msg[2] as c_int) << 8 as c_int
                | mm.msg[3] as c_int) as u32;
            mm.ca = mm.msg[0] as c_int & 0x7 as c_int;
            mm.crcok = (0 as c_int as c_uint == mm.crc) as c_int;
            if mm.crcok != 0 {
                // DF 11 : if crc == 0 try to populate our ICAO addresses whitelist.
                mode_s.icao_cache.add_recently_seen_addr(mm.addr);
            } else if mm.crc < 80 as c_int as c_uint {
                mm.crcok = mode_s.icao_cache.address_was_recently_seen(mm.addr);
                if mm.crcok != 0 {
                    mode_s.icao_cache.add_recently_seen_addr(mm.addr);
                }
            }
        }
        17 => {
            // DF 17
            mm.addr = ((mm.msg[1] as c_int) << 16 as c_int
                | (mm.msg[2] as c_int) << 8 as c_int
                | mm.msg[3] as c_int) as u32; // Responder capabilities
            mm.ca = mm.msg[0] as c_int & 0x7 as c_int;
            mm.crcok = (0 as c_int as c_uint == mm.crc) as c_int;
            if mm.crcok != 0 {
                // DF 17 : if crc == 0 try to populate our ICAO addresses whitelist.
                mode_s.icao_cache.add_recently_seen_addr(mm.addr);
            }
        }
        18 => {
            // DF 18
            // This is currently identical to DF 17 as the ca field is being reused to store the
            // Control Field.
            mm.addr = ((mm.msg[1] as c_int) << 16 as c_int
                | (mm.msg[2] as c_int) << 8 as c_int
                | mm.msg[3] as c_int) as u32; // Control Field, should always be 0 for ADS-B
            mm.ca = mm.msg[0] as c_int & 0x7 as c_int;
            mm.crcok = (0 as c_int as c_uint == mm.crc) as c_int;
            if mm.crcok != 0 {
                // DF 18 : if crc == 0 try to populate our ICAO addresses whitelist.
                mode_s.icao_cache.add_recently_seen_addr(mm.addr);
            }
        }
        _ => {
            // All other DF's
            // Compare the checksum with the whitelist of recently seen ICAO
            // addresses. If it matches one, then declare the message as valid
            mm.addr = mm.crc;
            mm.crcok = mode_s.icao_cache.address_was_recently_seen(mm.addr)
        }
    }

    // If we're checking CRC and the CRC is invalid, then we can't trust any
    // of the data contents, so save time and give up now.
    if mode_s.check_crc != 0 && mm.crcok == 0 && mm.correctedbits == 0 {
        return;
    }

    // Fields for DF0, DF16
    if mm.msgtype == 0 as c_int || mm.msgtype == 16 as c_int {
        if mm.msg[0] as c_int & 0x4 as c_int != 0 {
            // VS Bit
            mm.b_flags |= (1 as c_int) << 12 as c_int | (1 as c_int) << 9 as c_int
        } else {
            mm.b_flags |= (1 as c_int) << 12 as c_int
        }
    }

    // Fields for DF11, DF17
    if mm.msgtype == 11 as c_int || mm.msgtype == 17 as c_int {
        if mm.ca == 4 as c_int {
            mm.b_flags |= (1 as c_int) << 12 as c_int | (1 as c_int) << 9 as c_int
        } else if mm.ca == 5 as c_int {
            mm.b_flags |= (1 as c_int) << 12 as c_int
        }
    }

    // Fields for DF5, DF21 = Gillham encoded Squawk
    if mm.msgtype == 5 as c_int || mm.msgtype == 21 as c_int {
        let id13field: c_int =
            ((mm.msg[2] as c_int) << 8 as c_int | mm.msg[3] as c_int) & 0x1fff as c_int;
        if id13field != 0 {
            mm.b_flags |= (1 as c_int) << 5 as c_int;
            mm.mode_a = decode_id13_field(id13field)
        }
    }

    // Fields for DF0, DF4, DF16, DF20 13 bit altitude
    if mm.msgtype == 0 as c_int
        || mm.msgtype == 4 as c_int
        || mm.msgtype == 16 as c_int
        || mm.msgtype == 20 as c_int
    {
        let ac13field: c_int =
            ((mm.msg[2] as c_int) << 8 as c_int | mm.msg[3] as c_int) & 0x1fff as c_int;
        if ac13field != 0 {
            // Only attempt to decode if a valid (non zero) altitude is present
            mm.b_flags |= (1 as c_int) << 1 as c_int;
            mm.altitude = decode_ac13_field(ac13field, &mut mm.unit)
        }
    }

    // Fields for DF4, DF5, DF20, DF21
    if mm.msgtype == 4 as c_int
        || mm.msgtype == 20 as c_int
        || mm.msgtype == 5 as c_int
        || mm.msgtype == 21 as c_int
    {
        mm.b_flags |= (1 as c_int) << 13 as c_int; // Flight status for DF4,5,20,21
        mm.fs = mm.msg[0] as c_int & 7 as c_int;
        if mm.fs <= 3 as c_int {
            mm.b_flags |= (1 as c_int) << 12 as c_int;
            if mm.fs & 1 as c_int != 0 {
                mm.b_flags |= (1 as c_int) << 9 as c_int
            }
        }
    }

    // Fields for DF17, DF18_CF0, DF18_CF1, DF18_CF6 squitters
    if mm.msgtype == 17 as c_int
        || mm.msgtype == 18 as c_int
            && (mm.ca == 0 as c_int || mm.ca == 1 as c_int || mm.ca == 6 as c_int)
    {
        mm.metype = mm.msg[4] as c_int >> 3 as c_int; // Extended squitter message type
        let metype: c_int = mm.metype; // Extended squitter message subtype
        mm.mesub = if metype == 29 as c_int {
            (mm.msg[4] as c_int & 6 as c_int) >> 1 as c_int
        } else {
            (mm.msg[4] as c_int) & 7 as c_int
        };
        let mesub: c_int = mm.mesub;
        // Decode the extended squitter message
        if metype >= 1 as c_int && metype <= 4 as c_int {
            // Aircraft Identification and Category
            mm.b_flags |= 1 << 6;
            let mut chars = ((mm.msg[5] as c_int) << 16
                | (mm.msg[6] as c_int) << 8
                | mm.msg[7] as c_int) as u32;
            mm.flight[3] = AIS_CHARSET[(chars & 0x3f) as usize];
            chars = chars >> 6;
            mm.flight[2] = AIS_CHARSET[(chars & 0x3f) as usize];
            chars = chars >> 6;
            mm.flight[1] = AIS_CHARSET[(chars & 0x3f) as usize];
            chars = chars >> 6;
            mm.flight[0] = AIS_CHARSET[(chars & 0x3f) as usize];
            chars = ((mm.msg[8] as c_int) << 16 | (mm.msg[9] as c_int) << 8 | mm.msg[10] as c_int)
                as u32;
            mm.flight[7] = AIS_CHARSET[(chars & 0x3f) as usize];
            chars = chars >> 6;
            mm.flight[6] = AIS_CHARSET[(chars & 0x3f) as usize];
            chars = chars >> 6;
            mm.flight[5] = AIS_CHARSET[(chars & 0x3f) as usize];
            chars = chars >> 6;
            mm.flight[4] = AIS_CHARSET[(chars & 0x3f) as usize];
        } else if metype == 19 as c_int {
            // Airborne Velocity Message
            // Presumably airborne if we get an Airborne Velocity Message
            mm.b_flags |= (1 as c_int) << 12 as c_int;
            if mesub >= 1 as c_int && mesub <= 4 as c_int {
                let mut vert_rate: c_int = (mm.msg[8] as c_int & 0x7 as c_int) << 6 as c_int
                    | mm.msg[9] as c_int >> 2 as c_int;
                if vert_rate != 0 {
                    vert_rate -= 1;
                    if mm.msg[8] as c_int & 0x8 as c_int != 0 {
                        vert_rate = 0 as c_int - vert_rate
                    }
                    mm.vert_rate = vert_rate * 64 as c_int;
                    mm.b_flags |= (1 as c_int) << 4 as c_int
                }
            }
            if mesub == 1 as c_int || mesub == 2 as c_int {
                let ew_raw: c_int =
                    (mm.msg[5] as c_int & 0x3 as c_int) << 8 as c_int | mm.msg[6] as c_int;
                let mut ew_vel: c_int = ew_raw - 1 as c_int;
                let ns_raw: c_int = (mm.msg[7] as c_int & 0x7f as c_int) << 3 as c_int
                    | mm.msg[8] as c_int >> 5 as c_int;
                let mut ns_vel: c_int = ns_raw - 1 as c_int;
                if mesub == 2 as c_int {
                    // If (supersonic) unit is 4 kts
                    ns_vel = ns_vel << 2 as c_int;
                    ew_vel = ew_vel << 2 as c_int
                }
                if ew_raw != 0 {
                    // Do East/West
                    mm.b_flags |= (1 as c_int) << 7 as c_int;
                    if mm.msg[5] as c_int & 0x4 as c_int != 0 {
                        ew_vel = 0 as c_int - ew_vel
                    }
                    mm.ew_velocity = ew_vel
                }
                if ns_raw != 0 {
                    // Do North/South
                    mm.b_flags |= (1 as c_int) << 8 as c_int;
                    if mm.msg[7] as c_int & 0x80 as c_int != 0 {
                        ns_vel = 0 as c_int - ns_vel
                    }
                    mm.ns_velocity = ns_vel
                }
                if ew_raw != 0 && ns_raw != 0 {
                    // Compute velocity and angle from the two speed components
                    mm.b_flags |= (1 as c_int) << 3 as c_int
                        | (1 as c_int) << 2 as c_int
                        | (1 as c_int) << 14 as c_int;
                    mm.velocity = ((ns_vel * ns_vel + ew_vel * ew_vel) as f64).sqrt() as c_int;
                    if mm.velocity != 0 {
                        mm.heading = ((ew_vel as f64).atan2(ns_vel as f64) * 180.0f64
                            / 3.14159265358979323846f64)
                            as c_int;
                        // We don't want negative values but a 0-360 scale
                        if mm.heading < 0 as c_int {
                            mm.heading += 360 as c_int
                        }
                    }
                }
            } else if mesub == 3 as c_int || mesub == 4 as c_int {
                let mut airspeed: c_int = (mm.msg[7] as c_int & 0x7f as c_int) << 3 as c_int
                    | mm.msg[8] as c_int >> 5 as c_int;
                if airspeed != 0 {
                    mm.b_flags |= (1 as c_int) << 3 as c_int;
                    airspeed -= 1;
                    if mesub == 4 as c_int {
                        // If (supersonic) unit is 4 kts
                        airspeed = airspeed << 2 as c_int
                    }
                    mm.velocity = airspeed
                }
                if mm.msg[5] as c_int & 0x4 as c_int != 0 {
                    mm.b_flags |= (1 as c_int) << 2 as c_int;
                    mm.heading = ((mm.msg[5] as c_int & 0x3 as c_int) << 8 as c_int
                        | mm.msg[6] as c_int)
                        * 45 as c_int
                        >> 7 as c_int
                }
            }
        } else if metype >= 5 as c_int && metype <= 22 as c_int {
            // Position Message
            mm.raw_latitude = (mm.msg[6] as c_int & 3 as c_int) << 15 as c_int
                | (mm.msg[7] as c_int) << 7 as c_int
                | mm.msg[8] as c_int >> 1 as c_int; // Ground
            mm.raw_longitude = (mm.msg[8] as c_int & 1 as c_int) << 16 as c_int
                | (mm.msg[9] as c_int) << 8 as c_int
                | mm.msg[10] as c_int;
            mm.b_flags |= if mm.msg[6 as c_int as usize] as c_int & 0x4 as c_int != 0 {
                (1 as c_int) << 11 as c_int
            } else {
                (1 as c_int) << 10 as c_int
            };
            if metype >= 9 as c_int {
                let ac12field: c_int = ((mm.msg[5] as c_int) << 4 as c_int
                    | mm.msg[6] as c_int >> 4 as c_int)
                    & 0xfff as c_int;
                mm.b_flags |= (1 as c_int) << 12 as c_int;
                if ac12field != 0 {
                    // Airborne
                    // Only attempt to decode if a valid (non zero) altitude is present
                    mm.b_flags |= (1 as c_int) << 1 as c_int;
                    mm.altitude = decode_ac12_field(ac12field, &mut mm.unit)
                }
            } else {
                let movement: c_int = ((mm.msg[4] as c_int) << 4 as c_int
                    | mm.msg[5] as c_int >> 4 as c_int)
                    & 0x7f as c_int;
                mm.b_flags |= (1 as c_int) << 12 as c_int | (1 as c_int) << 9 as c_int;
                if movement != 0 && movement < 125 as c_int {
                    mm.b_flags |= (1 as c_int) << 3 as c_int;
                    mm.velocity = decode_movement_field(movement)
                }
                if mm.msg[5] as c_int & 0x8 as c_int != 0 {
                    mm.b_flags |= (1 as c_int) << 2 as c_int;
                    mm.heading = (((mm.msg[5] as c_int) << 4 as c_int
                        | mm.msg[6] as c_int >> 4 as c_int)
                        & 0x7f as c_int)
                        * 45 as c_int
                        >> 4 as c_int
                }
            }
        } else if metype == 23 as c_int {
            // Test metype squawk field
            if mesub == 7 as c_int {
                // (see 1090-WP-15-20)
                let id13field_0: c_int =
                    (((mm.msg[5] as c_int) << 8 as c_int | mm.msg[6] as c_int) & 0xfff1 as c_int)
                        >> 3 as c_int;
                if id13field_0 != 0 {
                    mm.b_flags |= (1 as c_int) << 5 as c_int;
                    mm.mode_a = decode_id13_field(id13field_0)
                }
            }
        } else if !(metype == 24 as c_int) {
            if metype == 28 as c_int {
                // Extended Squitter Aircraft Status
                if mesub == 1 as c_int {
                    // Emergency status squawk field
                    let id13field_1: c_int =
                        ((mm.msg[5] as c_int) << 8 as c_int | mm.msg[6] as c_int) & 0x1fff as c_int;
                    if id13field_1 != 0 {
                        mm.b_flags |= (1 as c_int) << 5 as c_int;
                        mm.mode_a = decode_id13_field(id13field_1)
                    }
                }
            } else if !(metype == 29 as c_int) {
                if !(metype == 30 as c_int) {
                    (metype) == 31 as c_int;
                }
            }
        }
    }

    // Fields for DF20, DF21 Comm-B
    if mm.msgtype == 20 as c_int || mm.msgtype == 21 as c_int {
        if mm.msg[4] as c_int == 0x20 as c_int {
            // Aircraft Identification
            mm.b_flags |= (1 as c_int) << 6 as c_int;
            let mut chars_0 = ((mm.msg[5] as c_int) << 16 as c_int
                | (mm.msg[6] as c_int) << 8 as c_int
                | mm.msg[7] as c_int) as u32;
            mm.flight[3] = AIS_CHARSET[(chars_0 & 0x3f) as usize];
            chars_0 = chars_0 >> 6;
            mm.flight[2] = AIS_CHARSET[(chars_0 & 0x3f) as usize];
            chars_0 = chars_0 >> 6;
            mm.flight[1] = AIS_CHARSET[(chars_0 & 0x3f) as usize];
            chars_0 = chars_0 >> 6;
            mm.flight[0] = AIS_CHARSET[(chars_0 & 0x3f) as usize];
            chars_0 = ((mm.msg[8] as c_int) << 16 | (mm.msg[9] as c_int) << 8 | mm.msg[10] as c_int)
                as u32;
            mm.flight[7] = AIS_CHARSET[(chars_0 & 0x3f) as usize];
            chars_0 = chars_0 >> 6;
            mm.flight[6] = AIS_CHARSET[(chars_0 & 0x3f) as usize];
            chars_0 = chars_0 >> 6;
            mm.flight[5] = AIS_CHARSET[(chars_0 & 0x3f) as usize];
            chars_0 = chars_0 >> 6;
            mm.flight[4] = AIS_CHARSET[(chars_0 & 0x3f) as usize];
        }
    };
}

// This function gets a decoded Mode S Message and prints it on the screen
// in a human readable format.
//
fn display_modes_message(mode_s: &ModeS, mm: &ModesMessage) {
    // Handle only addresses mode first.
    if mode_s.onlyaddr != 0 {
        println!("{:06x}", mm.addr);
        return; // Enough for --onlyaddr mode
    }

    // Show the raw message.
    println!("{}", format_raw_message(mode_s, mm));

    if mode_s.raw != 0 {
        let _ = io::stdout().flush();
        return; // Enough for --raw mode
    }

    if mm.msgtype < 32 as c_int {
        println!(
            "CRC: {:06x} ({})",
            mm.crc as c_int,
            if mm.crcok != 0 { "ok" } else { "wrong" }
        );
    }

    if mm.correctedbits != 0 as c_int {
        println!("No. of bit errors fixed: {}", mm.correctedbits);
    }

    if mm.msgtype == 0 as c_int {
        // DF 0
        println!("DF 0: Short Air-Air Surveillance.");
        println!(
            "  VS             : {}",
            if mm.msg[0] as c_int & 0x4 as c_int != 0 {
                "Ground"
            } else {
                "Airborne"
            }
        );
        println!(
            "  CC             : {}",
            (mm.msg[0] as c_int & 0x2 as c_int) >> 1 as c_int
        );
        println!(
            "  SL             : {}",
            (mm.msg[1] as c_int & 0xe0 as c_int) >> 5 as c_int
        );
        println!("  Altitude       : {} {}", mm.altitude, mm.unit);
        println!("  ICAO Address   : {:06x}", mm.addr);
    } else if mm.msgtype == 4 as c_int || mm.msgtype == 20 as c_int {
        println!(
            "DF {}: {}, Altitude Reply.",
            mm.msgtype,
            if mm.msgtype == 4 as c_int {
                "Surveillance"
            } else {
                "Comm-B"
            }
        );
        println!("  Flight Status  : {}", FLIGHT_STATUSES[mm.fs as usize]);
        println!(
            "  DR             : {}",
            mm.msg[1] as c_int >> 3 as c_int & 0x1f as c_int
        );
        println!(
            "  UM             : {}",
            (mm.msg[1] as c_int & 7 as c_int) << 3 as c_int | mm.msg[2] as c_int >> 5 as c_int
        );
        println!("  Altitude       : {} {}", mm.altitude, mm.unit);
        println!("  ICAO Address   : {:06x}", mm.addr);
        if mm.msgtype == 20 as c_int {
            println!("  Comm-B BDS     : {:x}", mm.msg[4] as c_int);
            // Decode the extended squitter message
            if mm.msg[4] as c_int == 0x20 as c_int {
                // BDS 2,0 Aircraft identification
                println!(
                    "    BDS 2,0 Aircraft Identification : {}",
                    mm.flight_number_str()
                );
                /*
                            } else if ( mm->msg[4]       == 0x10) { // BDS 1,0 Datalink Capability report
                                print!("    BDS 1,0 Datalink Capability report\n");

                            } else if ( mm->msg[4]       == 0x30) { // BDS 3,0 ACAS Active Resolution Advisory
                                print!("    BDS 3,0 ACAS Active Resolution Advisory\n");

                            } else if ((mm->msg[4] >> 3) ==   28) { // BDS 6,1 Extended Squitter Emergency/Priority Status
                                print!("    BDS 6,1 Emergency/Priority Status\n");

                            } else if ((mm->msg[4] >> 3) ==   29) { // BDS 6,2 Target State and Status
                                print!("    BDS 6,2 Target State and Status\n");

                            } else if ((mm->msg[4] >> 3) ==   31) { // BDS 6,5 Extended Squitter Aircraft Operational Status
                                print!("    BDS 6,5 Aircraft Operational Status\n");
                */
            }
        }
    } else if mm.msgtype == 5 as c_int || mm.msgtype == 21 as c_int {
        println!(
            "DF {}: {}, Identity Reply.",
            mm.msgtype,
            if mm.msgtype == 5 as c_int {
                "Surveillance"
            } else {
                "Comm-B"
            }
        );
        println!("  Flight Status  : {}", FLIGHT_STATUSES[mm.fs as usize]);
        println!(
            "  DR             : {}",
            mm.msg[1] as c_int >> 3 as c_int & 0x1f as c_int
        );
        println!(
            "  UM             : {}",
            (mm.msg[1] as c_int & 7 as c_int) << 3 as c_int | mm.msg[2] as c_int >> 5 as c_int
        );
        println!("  Squawk         : {:04x}", mm.mode_a);
        println!("  ICAO Address   : {:06x}", mm.addr);
        if mm.msgtype == 21 as c_int {
            println!("  Comm-B BDS     : {:x}", mm.msg[4] as c_int);
            // Decode the extended squitter message
            if mm.msg[4] as c_int == 0x20 as c_int {
                // BDS 2,0 Aircraft identification
                println!(
                    "    BDS 2,0 Aircraft Identification : {}",
                    mm.flight_number_str()
                );
                /*
                            } else if ( mm->msg[4]       == 0x10) { // BDS 1,0 Datalink Capability report
                                print!("    BDS 1,0 Datalink Capability report\n");

                            } else if ( mm->msg[4]       == 0x30) { // BDS 3,0 ACAS Active Resolution Advisory
                                print!("    BDS 3,0 ACAS Active Resolution Advisory\n");

                            } else if ((mm->msg[4] >> 3) ==   28) { // BDS 6,1 Extended Squitter Emergency/Priority Status
                                print!("    BDS 6,1 Emergency/Priority Status\n");

                            } else if ((mm->msg[4] >> 3) ==   29) { // BDS 6,2 Target State and Status
                                print!("    BDS 6,2 Target State and Status\n");

                            } else if ((mm->msg[4] >> 3) ==   31) { // BDS 6,5 Extended Squitter Aircraft Operational Status
                                print!("    BDS 6,5 Aircraft Operational Status\n");
                */
            }
        }
    } else if mm.msgtype == 11 as c_int {
        // DF 11
        println!("DF 11: All Call Reply.");
        println!(
            "  Capability  : {} ({})",
            mm.ca, CAPABILITIES[mm.ca as usize]
        );
        println!("  ICAO Address: {:06x}", mm.addr);
        if mm.iid > 16 as c_int {
            println!("  IID         : SI-{:02}", mm.iid - 16 as c_int);
        } else {
            println!("  IID         : II-{:02}", mm.iid);
        }
    } else if mm.msgtype == 16 as c_int {
        // DF 16
        println!("DF 16: Long Air to Air ACAS");
        println!(
            "  VS             : {}",
            if mm.msg[0] as c_int & 0x4 as c_int != 0 {
                "Ground"
            } else {
                "Airborne"
            }
        );
        println!(
            "  CC             : {}",
            (mm.msg[0] as c_int & 0x2 as c_int) >> 1 as c_int
        );
        println!(
            "  SL             : {}",
            (mm.msg[1] as c_int & 0xe0 as c_int) >> 5 as c_int
        );
        println!("  Altitude       : {} {}", mm.altitude, mm.unit);
        println!("  ICAO Address   : {:06x}", mm.addr);
    } else if mm.msgtype == 17 as c_int {
        // DF 17
        println!("DF 17: ADS-B message.");
        println!(
            "  Capability     : {} ({})",
            mm.ca, CAPABILITIES[mm.ca as usize]
        );
        println!("  ICAO Address   : {:06x}", mm.addr);
        println!("  Extended Squitter  Type: {}", mm.metype);
        println!("  Extended Squitter  Sub : {}", mm.mesub);
        println!(
            "  Extended Squitter  Name: {}",
            get_me_description(mm.metype, mm.mesub)
        );

        // Decode the extended squitter message
        if mm.metype >= 1 as c_int && mm.metype <= 4 as c_int {
            // Aircraft identification
            println!(
                "    Aircraft Type  : {}{}",
                std::char::from_u32('A' as u32 + 4 - mm.metype as u32).unwrap_or('?'),
                mm.mesub
            );
            println!("    Identification : {}", mm.flight_number_str());
        } else if mm.metype == 19 as c_int {
            // Airborne Velocity
            if mm.mesub == 1 as c_int || mm.mesub == 2 as c_int {
                println!(
                    "    EW status         : {}",
                    if mm.b_flags & MODES_ACFLAGS_EWSPEED_VALID != 0 {
                        "Valid"
                    } else {
                        "Unavailable"
                    }
                );
                println!("    EW velocity       : {}", mm.ew_velocity);
                println!(
                    "    NS status         : {}",
                    if mm.b_flags & MODES_ACFLAGS_NSSPEED_VALID != 0 {
                        "Valid"
                    } else {
                        "Unavailable"
                    }
                );
                println!("    NS velocity       : {}", mm.ns_velocity);
                println!(
                    "    Vertical status   : {}",
                    if mm.b_flags & MODES_ACFLAGS_VERTRATE_VALID != 0 {
                        "Valid"
                    } else {
                        "Unavailable"
                    }
                );
                println!(
                    "    Vertical rate src : {}",
                    mm.msg[8] as c_int >> 4 as c_int & 1 as c_int
                );
                println!("    Vertical rate     : {}", mm.vert_rate);
            } else if mm.mesub == 3 as c_int || mm.mesub == 4 as c_int {
                println!(
                    "    Heading status    : {}",
                    if mm.b_flags & MODES_ACFLAGS_HEADING_VALID != 0 {
                        "Valid"
                    } else {
                        "Unavailable"
                    }
                );
                println!("    Heading           : {}", mm.heading);
                println!(
                    "    Airspeed status   : {}",
                    if mm.b_flags & MODES_ACFLAGS_SPEED_VALID != 0 {
                        "Valid"
                    } else {
                        "Unavailable"
                    }
                );
                println!("    Airspeed          : {}", mm.velocity);
                println!(
                    "    Vertical status   : {}",
                    if mm.b_flags & MODES_ACFLAGS_VERTRATE_VALID != 0 {
                        "Valid"
                    } else {
                        "Unavailable"
                    }
                );
                println!(
                    "    Vertical rate src : {}",
                    mm.msg[8] as c_int >> 4 as c_int & 1 as c_int
                );
                println!("    Vertical rate     : {}", mm.vert_rate);
            } else {
                println!(
                    "    Unrecognized ME subtype: {} subtype: {}",
                    mm.metype, mm.mesub
                );
            }
        } else if mm.metype >= 5 as c_int && mm.metype <= 22 as c_int {
            // Airborne position Baro
            println!(
                "    F flag   : {}",
                if mm.msg[6] as c_int & 0x4 as c_int != 0 {
                    "odd"
                } else {
                    "even"
                }
            );
            println!(
                "    T flag   : {}",
                if mm.msg[6] as c_int & 0x8 as c_int != 0 {
                    "UTC"
                } else {
                    "non-UTC"
                }
            );
            println!("    Altitude : {} feet", mm.altitude);
            if mm.b_flags & MODES_ACFLAGS_LATLON_VALID != 0 {
                println!("    Latitude : {:.6}", mm.f_lat);
                println!("    Longitude: {:.6}", mm.f_lon);
            } else {
                println!("    Latitude : {} (not decoded)", mm.raw_latitude);
                println!("    Longitude: {} (not decoded)", mm.raw_longitude);
            }
        } else if mm.metype == 28 as c_int {
            // Extended Squitter Aircraft Status
            if mm.mesub == 1 as c_int {
                println!(
                    "    Emergency State: {}",
                    EMERGENCY_STATES[((mm.msg[5] as c_int & 0xe0 as c_int) >> 5 as c_int) as usize]
                );
                println!("    Squawk: {:04x}", mm.mode_a);
            } else {
                println!(
                    "    Unrecognized ME subtype: {} subtype: {}",
                    mm.metype, mm.mesub
                );
            }
        } else if mm.metype == 23 as c_int {
            // Test Message
            if mm.mesub == 7 as c_int {
                println!("    Squawk: {:04x}", mm.mode_a);
            } else {
                println!(
                    "    Unrecognized ME subtype: {} subtype: {}",
                    mm.metype, mm.mesub
                );
            }
        } else {
            println!(
                "    Unrecognized ME type: {} subtype: {}",
                mm.metype, mm.mesub
            );
        }
    } else if mm.msgtype == 18 as c_int {
        // DF 18
        println!("DF 18: Extended Squitter.");
        println!(
            "  Control Field : {} ({})",
            mm.ca, CONTROL_FIELDS[mm.ca as usize]
        );
        if mm.ca == 0 as c_int || mm.ca == 1 as c_int || mm.ca == 6 as c_int {
            if mm.ca == 1 as c_int {
                println!("  Other Address : {:06x}", mm.addr);
            } else {
                println!("  ICAO Address  : {:06x}", mm.addr);
            }
            println!("  Extended Squitter  Type: {}", mm.metype);
            println!("  Extended Squitter  Sub : {}", mm.mesub);
            println!(
                "  Extended Squitter  Name: {}",
                get_me_description(mm.metype, mm.mesub)
            );

            // Decode the extended squitter message
            if mm.metype >= 1 as c_int && mm.metype <= 4 as c_int {
                // Aircraft identification
                println!(
                    "    Aircraft Type  : {}{}",
                    std::char::from_u32('A' as u32 + 4 - mm.metype as u32).unwrap_or('?'),
                    mm.mesub
                );
                println!("    Identification : {}", mm.flight_number_str());
            } else if mm.metype == 19 as c_int {
                // Airborne Velocity
                if mm.mesub == 1 as c_int || mm.mesub == 2 as c_int {
                    println!(
                        "    EW status         : {}",
                        if mm.b_flags & MODES_ACFLAGS_EWSPEED_VALID != 0 {
                            "Valid"
                        } else {
                            "Unavailable"
                        }
                    );
                    println!("    EW velocity       : {}", mm.ew_velocity);
                    println!(
                        "    NS status         : {}",
                        if mm.b_flags & MODES_ACFLAGS_NSSPEED_VALID != 0 {
                            "Valid"
                        } else {
                            "Unavailable"
                        }
                    );
                    println!("    NS velocity       : {}", mm.ns_velocity);
                    println!(
                        "    Vertical status   : {}",
                        if mm.b_flags & MODES_ACFLAGS_VERTRATE_VALID != 0 {
                            "Valid"
                        } else {
                            "Unavailable"
                        }
                    );
                    println!(
                        "    Vertical rate src : {}",
                        mm.msg[8] as c_int >> 4 as c_int & 1 as c_int
                    );
                    println!("    Vertical rate     : {}", mm.vert_rate);
                } else if mm.mesub == 3 as c_int || mm.mesub == 4 as c_int {
                    println!(
                        "    Heading status    : {}",
                        if mm.b_flags & MODES_ACFLAGS_HEADING_VALID != 0 {
                            "Valid"
                        } else {
                            "Unavailable"
                        }
                    );
                    println!("    Heading           : {}", mm.heading);
                    println!(
                        "    Airspeed status   : {}",
                        if mm.b_flags & MODES_ACFLAGS_SPEED_VALID != 0 {
                            "Valid"
                        } else {
                            "Unavailable"
                        }
                    );
                    println!("    Airspeed          : {}", mm.velocity);
                    println!(
                        "    Vertical status   : {}",
                        if mm.b_flags & MODES_ACFLAGS_VERTRATE_VALID != 0 {
                            "Valid"
                        } else {
                            "Unavailable"
                        }
                    );
                    println!(
                        "    Vertical rate src : {}",
                        mm.msg[8] as c_int >> 4 as c_int & 1 as c_int
                    );
                    println!("    Vertical rate     : {}", mm.vert_rate);
                } else {
                    println!(
                        "    Unrecognized ME subtype: {} subtype: {}",
                        mm.metype, mm.mesub
                    );
                }
            } else if mm.metype >= 5 as c_int && mm.metype <= 22 as c_int {
                // Ground or Airborne position, Baro or GNSS
                println!(
                    "    F flag   : {}",
                    if mm.msg[6] as c_int & 0x4 as c_int != 0 {
                        "odd"
                    } else {
                        "even"
                    }
                );
                println!(
                    "    T flag   : {}",
                    if mm.msg[6] as c_int & 0x8 as c_int != 0 {
                        "UTC"
                    } else {
                        "non-UTC"
                    }
                );
                println!("    Altitude : {} feet", mm.altitude);
                if mm.b_flags & MODES_ACFLAGS_LATLON_VALID != 0 {
                    println!("    Latitude : {}", mm.f_lat);
                    println!("    Longitude: {}", mm.f_lon);
                } else {
                    println!("    Latitude : {} (not decoded)", mm.raw_latitude);
                    println!("    Longitude: {} (not decoded)", mm.raw_longitude);
                }
            } else {
                println!(
                    "    Unrecognized ME type: {} subtype: {}",
                    mm.metype, mm.mesub
                );
            }
        }
    } else if mm.msgtype == 19 as c_int {
        // DF 19
        println!("DF 19: Military Extended Squitter.");
    } else if mm.msgtype == 22 as c_int {
        // DF 22
        println!("DF 22: Military Use.");
    } else if mm.msgtype == 24 as c_int {
        // DF 24
        println!("DF 24: Comm D Extended Length Message.");
    } else if mm.msgtype == 32 as c_int {
        // DF 32 is special code we use for Mode A/C
        println!("SSR : Mode A/C Reply.");
        if mm.fs & 0x80 as c_int != 0 {
            println!("  Mode A : {:04x} IDENT", mm.mode_a);
        } else {
            println!("  Mode A : {:04x}", mm.mode_a);
            if mm.b_flags & MODES_ACFLAGS_ALTITUDE_VALID != 0 {
                println!("  Mode C : {} feet", mm.altitude);
            }
        }
    } else {
        println!("DF {}: Unknown DF Format.", mm.msgtype);
    }
    println!();
}

fn format_raw_message(mode_s: &ModeS, mm: &ModesMessage) -> String {
    let mut s = String::with_capacity(mm.msgbits as usize / 4);
    if mode_s.mlat != 0 && mm.timestamp_msg != 0 {
        s.push('@');
        let bytes = mm.timestamp_msg.to_le_bytes();
        for byte in bytes[0..6].into_iter().rev() {
            s.push_str(&format!("{:02X}", byte))
        }
    } else {
        s.push('*')
    }

    let mut j = 0;
    while j < mm.msgbits / 8 as c_int {
        s.push_str(&format!("{:02x}", mm.msg[j as usize] as c_int));
        j += 1
    }
    s.push(';');
    s
}

// Capability table
static CAPABILITIES: [&str; 8] = [
    "Level 1 (Surveillance Only)",
    "Level 2 (DF0,4,5,11)",
    "Level 3 (DF0,4,5,11,20,21)",
    "Level 4 (DF0,4,5,11,20,21,24)",
    "Level 2+3+4 (DF0,4,5,11,20,21,24,code7 - is on ground)",
    "Level 2+3+4 (DF0,4,5,11,20,21,24,code7 - is airborne)",
    "Level 2+3+4 (DF0,4,5,11,20,21,24,code7)",
    "Level 7 ???",
];

// DF 18 Control field table.
static CONTROL_FIELDS: [&str; 8] = [
    "ADS-B ES/NT device with ICAO 24-bit address",
    "ADS-B ES/NT device with other address",
    "Fine format TIS-B",
    "Coarse format TIS-B",
    "TIS-B management message",
    "TIS-B relay of ADS-B message with other address",
    "ADS-B rebroadcast using DF-17 message format",
    "Reserved",
];

// Flight status table
static FLIGHT_STATUSES: [&str; 8] = [
    "Normal, Airborne",
    "Normal, On the ground",
    "ALERT,  Airborne",
    "ALERT,  On the ground",
    "ALERT & Special Position Identification. Airborne or Ground",
    "Special Position Identification. Airborne or Ground",
    "Value 6 is not assigned",
    "Value 7 is not assigned",
];

// Emergency state table
// from https://www.ll.mit.edu/mission/aviation/publications/publication-files/atc-reports/Grappel_2007_ATC-334_WW-15318.pdf
// and 1090-DO-260B_FRAC
static EMERGENCY_STATES: [&str; 8] = [
    "No emergency",
    "General emergency (squawk 7700)",
    "Lifeguard/Medical",
    "Minimum fuel",
    "No communications (squawk 7600)",
    "Unlawful interference (squawk 7500)",
    "Downed Aircraft",
    "Reserved",
];

fn get_me_description(metype: c_int, mesub: c_int) -> &'static str {
    if metype >= 1 && metype <= 4 {
        "Aircraft Identification and Category"
    } else if metype >= 5 && metype <= 8 {
        "Surface Position"
    } else if metype >= 9 && metype <= 18 {
        "Airborne Position (Baro Altitude)"
    } else if metype == 19 && mesub >= 1 && mesub <= 4 {
        "Airborne Velocity"
    } else if metype >= 20 && metype <= 22 {
        "Airborne Position (GNSS Height)"
    } else if metype == 23 && mesub == 0 {
        "Test Message"
    } else if metype == 23 && mesub == 7 {
        "Test Message -- Squawk"
    } else if metype == 24 && mesub == 1 {
        "Surface System Status"
    } else if metype == 28 && mesub == 1 {
        "Extended Squitter Aircraft Status (Emergency)"
    } else if metype == 28 && mesub == 2 {
        "Extended Squitter Aircraft Status (1090ES TCAS RA)"
    } else if metype == 29 && (mesub == 0 || mesub == 1) {
        "Target State and Status Message"
    } else if metype == 31 && (mesub == 0 || mesub == 1) {
        "Aircraft Operational Status Message"
    } else {
        "Unknown"
    }
}

impl ModesMessage {
    fn flight_number_str(&self) -> &str {
        // unwrap should never panic as flight is constructed from AIS_CHARSET,
        // all of which are valid UTF-8.
        str::from_utf8(&self.flight).unwrap()
    }
}

// Turn I/Q samples pointed by Modes.data into the magnitude vector
// pointed by Modes.magnitude.
//
pub unsafe fn compute_magnitude_vector(mut p: *mut u16, mode_s: *mut ModeS) {
    let mut m: *mut u16 = (*mode_s)
        .magnitude
        .as_mut_ptr()
        .offset((MODES_PREAMBLE_SAMPLES + MODES_LONG_MSG_SAMPLES) as isize);

    ptr::copy_nonoverlapping(
        (*mode_s)
            .magnitude
            .as_ptr()
            .offset(MODES_ASYNC_BUF_SAMPLES as isize),
        (*mode_s).magnitude.as_mut_ptr(),
        MODES_PREAMBLE_SIZE + MODES_LONG_MSG_SIZE,
    );

    // Compute the magnitude vector. It's just SQRT(I^2 + Q^2), but
    // we rescale to the 0-255 range to exploit the full resolution.
    let mut j = 0;
    while j < MODES_ASYNC_BUF_SAMPLES {
        let fresh3 = p;
        p = p.offset(1);
        let fresh4 = m;
        m = m.offset(1);
        *fresh4 = *(*mode_s).maglut.offset(*fresh3 as isize);
        j = j.wrapping_add(1)
    }
}

// Return -1 if the message is out of phase left-side
// Return  1 if the message is out of phase right-size
// Return  0 if the message is not particularly out of phase.
//
// Note: this function assumes that preamble starts one sample before the current sample, so all
// offsets are +1. Previously the C code took a pointer and accessed p_preamble[-1].
//
fn detect_out_of_phase(preamble: &[u16]) -> c_int {
    if preamble[4] > preamble[3] / 3 {
        return 1;
    }
    if preamble[11] > preamble[10] / 3 {
        return 1;
    }
    if preamble[7] > preamble[8] / 3 {
        return -1;
    }
    if preamble[0] > preamble[2] / 3 {
        return -1;
    }

    0
}

fn clamped_scale(v: u16, scale: u16) -> u16 {
    u16::try_from(u32::from(v) * u32::from(scale) / 16384).unwrap_or(std::u16::MAX)
}

// This function decides whether we are sampling early or late,
// and by approximately how much, by looking at the energy in
// preamble bits before and after the expected pulse locations.
//
// It then deals with one sample pair at a time, comparing samples
// to make a decision about the bit value. Based on this decision it
// modifies the sample value of the *adjacent* sample which will
// contain some of the energy from the bit we just inspected.
//
// p_payload[0] should be the start of the preamble,
// p_payload[-1 .. MODES_PREAMBLE_SAMPLES + MODES_LONG_MSG_SAMPLES - 1] should be accessible.
// p_payload[MODES_PREAMBLE_SAMPLES .. MODES_PREAMBLE_SAMPLES + MODES_LONG_MSG_SAMPLES - 1] will be updated.
unsafe fn apply_phase_correction(p_payload: *mut u16) {
    // we expect 1 bits at 0, 2, 7, 9
    // and 0 bits at -1, 1, 3, 4, 5, 6, 8, 10, 11, 12, 13, 14
    // use bits -1,6 for early detection (bit 0/7 arrived a little early, our sample period starts after the bit phase so we include some of the next bit)
    // use bits 3,10 for late detection (bit 2/9 arrived a little late, our sample period starts before the bit phase so we include some of the last bit)

    let on_time: u32 = (*p_payload.offset(0) as c_int
        + *p_payload.offset(2) as c_int
        + *p_payload.offset(7) as c_int
        + *p_payload.offset(9) as c_int) as u32;
    let early: u32 = ((*p_payload.offset(-(1 as c_int) as isize) as c_int
        + *p_payload.offset(6) as c_int)
        << 1 as c_int) as u32;
    let late: u32 =
        ((*p_payload.offset(3) as c_int + *p_payload.offset(10) as c_int) << 1 as c_int) as u32;

    if early > late {
        // Our sample period starts late and so includes some of the next bit.
        let scale_up: u16 = (16384 as c_int as c_uint).wrapping_add(
            (16384 as c_int as c_uint)
                .wrapping_mul(early)
                .wrapping_div(early.wrapping_add(on_time)),
        ) as u16; // 1 + early / (early+on_time)
        let scale_down: u16 = (16384 as c_int as c_uint).wrapping_sub(
            (16384 as c_int as c_uint)
                .wrapping_mul(early)
                .wrapping_div(early.wrapping_add(on_time)),
        ) as u16; // 1 - early / (early+on_time)

        // trailing bits are 0; final data sample will be a bit low.
        *p_payload.offset(
            (8 as c_int * 2 as c_int + 14 as c_int * 8 as c_int * 2 as c_int - 1 as c_int) as isize,
        ) = clamped_scale(
            *p_payload.offset(
                (8 as c_int * 2 as c_int + 14 as c_int * 8 as c_int * 2 as c_int - 1 as c_int)
                    as isize,
            ),
            scale_up,
        );

        let mut j = MODES_PREAMBLE_SAMPLES + MODES_LONG_MSG_SAMPLES - 2;
        while j > MODES_PREAMBLE_SAMPLES {
            if *p_payload.offset(j as isize) as c_int > *p_payload.offset((j + 1) as isize) as c_int
            {
                // x [1 0] y
                // x overlapped with the "1" bit and is slightly high
                *p_payload.offset((j - 1) as isize) =
                    clamped_scale(*p_payload.offset(j as isize - 1), scale_down)
            } else {
                // x [0 1] y
                // x overlapped with the "0" bit and is slightly low
                *p_payload.offset((j - 1) as isize) =
                    clamped_scale(*p_payload.offset(j as isize - 1), scale_up)
            }
            j -= 2
        }
    } else {
        // Our sample period starts early and so includes some of the previous bit.
        let scale_up_0: u16 = (16384 as c_int as c_uint).wrapping_add(
            (16384 as c_int as c_uint)
                .wrapping_mul(late)
                .wrapping_div(late.wrapping_add(on_time)),
        ) as u16; // 1 + late / (late+on_time)
        let scale_down_0: u16 = (16384 as c_int as c_uint).wrapping_sub(
            (16384 as c_int as c_uint)
                .wrapping_mul(late)
                .wrapping_div(late.wrapping_add(on_time)),
        ) as u16; // 1 - late / (late+on_time)

        // leading bits are 0; first data sample will be a bit low.
        *p_payload.offset(MODES_PREAMBLE_SAMPLES as isize) = clamped_scale(
            *p_payload.offset(MODES_PREAMBLE_SAMPLES as isize),
            scale_up_0,
        );
        let mut j = MODES_PREAMBLE_SAMPLES;
        while j < MODES_PREAMBLE_SAMPLES + MODES_LONG_MSG_SAMPLES - 2 {
            if *p_payload.offset(j as isize) as c_int > *p_payload.offset(j as isize + 1) as c_int {
                // x [1 0] y
                // y overlapped with the "0" bit and is slightly low
                *p_payload.offset(j as isize + 2) =
                    clamped_scale(*p_payload.offset(j as isize + 2), scale_up_0)
            } else {
                // x [0 1] y
                // y overlapped with the "1" bit and is slightly high
                *p_payload.offset(j as isize + 2) =
                    clamped_scale(*p_payload.offset(j as isize + 2), scale_down_0)
            }
            j += 2
        }
    };
}

// Detect a Mode S messages inside the magnitude buffer pointed by 'm' and of
// size 'mlen' bytes. Every detected Mode S message is convert it into a
// stream of bits and passed to the function to display it.
//
pub unsafe fn detect_mode_s(
    m_slice: &mut [u16],
    mlen: u32,
    mode_s: &mut ModeS,
    bit_errors: &[ErrorInfo],
) {
    let m = m_slice.as_mut_ptr();
    let mut mm: ModesMessage = ModesMessage::default();
    let mut msg = [0; MODES_LONG_MSG_BYTES];
    let mut p_msg: *mut c_uchar;
    let mut aux = [0; MODES_PREAMBLE_SAMPLES + MODES_LONG_MSG_SAMPLES + 1];
    let mut use_correction: c_int = 0;
    let mut current_block_183: u64;
    // The Mode S preamble is made of impulses of 0.5 microseconds at
    // the following time offsets:
    //
    // 0   - 0.5 usec: first impulse.
    // 1.0 - 1.5 usec: second impulse.
    // 3.5 - 4   usec: third impulse.
    // 4.5 - 5   usec: last impulse.
    //
    // Since we are sampling at 2 Mhz every sample in our magnitude vector
    // is 0.5 usec, so the preamble will look like this, assuming there is
    // an impulse at offset 0 in the array:
    //
    // 0   -----------------
    // 1   -
    // 2   ------------------
    // 3   --
    // 4   -
    // 5   --
    // 6   -
    // 7   ------------------
    // 8   --
    // 9   -------------------
    //
    let mut j = 0u32;
    while j < mlen {
        let high: c_int;
        let mut i: c_int;
        let mut errors: c_int;
        let mut errors56: c_int;
        let mut errors_ty: c_int;
        let mut p_ptr: *mut u16;
        let mut the_byte: u8;
        let mut the_errs: u8;
        let mut msglen: c_int;
        let mut scanlen: c_int;
        let mut sig_strength: c_int;

        let preamble = &m_slice[j as usize..];
        let mut p_payload = m.offset(j as isize + MODES_PREAMBLE_SAMPLES as isize);

        // Rather than clear the whole mm structure, just clear the parts which are required. The clear
        // is required for every bit of the input stream, and we don't want to be memset-ing the whole
        // ModesMessage structure two million times per second if we don't have to..
        mm.correctedbits = 0;
        mm.crcok = mm.correctedbits;
        mm.b_flags = mm.crcok;

        if use_correction == 0 {
            // This is not a re-try with phase correction
            // so try to find a new preamble
            if (*mode_s).mode_ac != 0 {
                let mode_a = detect_mode_a(preamble, &mut mm);
                if mode_a != 0 {
                    // We have found a valid mode_a/C in the data
                    mm.timestamp_msg = (*mode_s)
                        .timestamp_blk
                        .wrapping_add(j.wrapping_add(1).wrapping_mul(6) as c_ulong);

                    // Decode the received message
                    decode_mode_a_message(&mut mm, mode_a);

                    // Pass data to the next layer
                    use_modes_message(mode_s, &mut mm);
                    j += MODEAC_MSG_SAMPLES;
                    (*mode_s).stat_mode_ac = (*mode_s).stat_mode_ac.wrapping_add(1);
                    current_block_183 = 735147466149431745;
                } else {
                    current_block_183 = 7175849428784450219;
                }
            } else {
                current_block_183 = 7175849428784450219;
            }

            match current_block_183 {
                735147466149431745 => {}
                _ =>
                // First check of relations between the first 10 samples
                // representing a valid preamble. We don't even investigate further
                // if this simple test is not passed
                {
                    if !(preamble[0] > preamble[1]
                        && preamble[1] < preamble[2]
                        && preamble[2] > preamble[3]
                        && preamble[3] < preamble[0]
                        && preamble[4] < preamble[0]
                        && preamble[5] < preamble[0]
                        && preamble[6] < preamble[0]
                        && preamble[7] > preamble[8]
                        && preamble[8] < preamble[9]
                        && preamble[9] > preamble[6])
                    {
                        if (*mode_s).debug & MODES_DEBUG_NOPREAMBLE != 0
                            && preamble[0] as c_int > MODES_DEBUG_NOPREAMBLE_LEVEL
                        {
                            dump_raw_message(
                                b"Unexpected ratio among first 10 samples\x00".as_ptr()
                                    as *const c_char,
                                msg.as_mut_ptr(),
                                m,
                                j,
                            );
                        }
                        current_block_183 = 735147466149431745;
                    } else {
                        // The samples between the two spikes must be < than the average
                        // of the high spikes level. We don't test bits too near to
                        // the high levels as signals can be out of phase so part of the
                        // energy can be in the near samples
                        high = (preamble[0] as c_int
                            + preamble[2] as c_int
                            + preamble[7] as c_int
                            + preamble[9] as c_int)
                            / 6 as c_int;
                        if preamble[4] as c_int >= high || preamble[5] as c_int >= high {
                            if (*mode_s).debug & MODES_DEBUG_NOPREAMBLE != 0
                                && preamble[0] as c_int > MODES_DEBUG_NOPREAMBLE_LEVEL
                            {
                                dump_raw_message(
                                    b"Too high level in samples between 3 and 6\x00".as_ptr()
                                        as *const c_char,
                                    msg.as_mut_ptr(),
                                    m,
                                    j,
                                );
                            }
                            current_block_183 = 735147466149431745;
                        } else if preamble[11] as c_int >= high
                            || preamble[12] as c_int >= high
                            || preamble[13] as c_int >= high
                            || preamble[14] as c_int >= high
                        {
                            // Similarly samples in the range 11-14 must be low, as it is the
                            // space between the preamble and real data. Again we don't test
                            // bits too near to high levels, see above
                            if (*mode_s).debug & MODES_DEBUG_NOPREAMBLE != 0
                                && preamble[0] as c_int > MODES_DEBUG_NOPREAMBLE_LEVEL
                            {
                                dump_raw_message(
                                    b"Too high level in samples between 10 and 15\x00".as_ptr()
                                        as *const c_char,
                                    msg.as_mut_ptr(),
                                    m,
                                    j,
                                );
                            }
                            current_block_183 = 735147466149431745;
                        } else {
                            (*mode_s).stat_valid_preamble =
                                (*mode_s).stat_valid_preamble.wrapping_add(1);
                            current_block_183 = 6450636197030046351;
                        }
                    }
                }
            }
        } else {
            // If the previous attempt with this message failed, retry using
            // magnitude correction
            // Make a copy of the Payload, and phase correct the copy
            let src = &m_slice[usize::try_from(j - 1).unwrap()..];
            ptr::copy_nonoverlapping(src.as_ptr(), aux.as_mut_ptr(), aux.len()); // FIXME: do this without ptr::
            apply_phase_correction(&mut *aux.as_mut_ptr().offset(1));
            (*mode_s).stat_out_of_phase = (*mode_s).stat_out_of_phase.wrapping_add(1);
            p_payload = aux.as_mut_ptr().offset(1 + MODES_PREAMBLE_SAMPLES as isize) as *mut u16;
            current_block_183 = 6450636197030046351;
            // TODO ... apply other kind of corrections
        }

        match current_block_183 {
            6450636197030046351 => {
                // Decode all the next 112 bits, regardless of the actual message
                // size. We'll check the actual message type later
                p_msg = &mut *msg.as_mut_ptr().offset(0) as *mut c_uchar;
                p_ptr = p_payload;
                the_byte = 0;
                the_errs = 0;
                errors_ty = 0;
                errors = 0;
                errors56 = 0;

                // We should have 4 'bits' of 0/1 and 1/0 samples in the preamble,
                // so include these in the signal strength
                sig_strength = preamble[0] as c_int - preamble[1] as c_int
                    + (preamble[2] as c_int - preamble[3] as c_int)
                    + (preamble[7] as c_int - preamble[6] as c_int)
                    + (preamble[9] as c_int - preamble[8] as c_int);

                scanlen = MODES_LONG_MSG_BITS;
                msglen = scanlen;
                i = 0 as c_int;
                while i < scanlen {
                    let fresh5 = p_ptr;
                    p_ptr = p_ptr.offset(1);
                    let a = *fresh5 as u32;
                    let fresh6 = p_ptr;
                    p_ptr = p_ptr.offset(1);
                    let b = *fresh6 as u32;
                    if a > b {
                        the_byte |= 1;
                        if i < 56 {
                            sig_strength =
                                (sig_strength as c_uint).wrapping_add(a.wrapping_sub(b)) as c_int
                        }
                    } else if a < b {
                        /*the_byte |= 0;*/
                        if i < 56 {
                            sig_strength =
                                (sig_strength as c_uint).wrapping_add(b.wrapping_sub(a)) as c_int
                        }
                    } else if i >= MODES_SHORT_MSG_BITS {
                        //(a == b), and we're in the long part of a frame
                        errors += 1
                    /*the_byte |= 0;*/
                    } else if i >= 5 {
                        //(a == b), and we're in the short part of a frame
                        scanlen = MODES_LONG_MSG_BITS;
                        errors += 1;
                        errors56 = errors
                    /*the_byte |= 0;*/
                    } else if i != 0 {
                        //(a == b), and we're in the message type part of a frame
                        errors += 1;
                        errors56 = errors;
                        errors_ty = errors56;
                        the_errs |= 1;
                    /*the_byte |= 0;*/
                    } else {
                        //(a == b), and we're in the first bit of the message type part of a frame
                        errors += 1;
                        errors56 = errors;
                        errors_ty = errors56;
                        the_errs |= 1;
                        the_byte |= 1;
                    }

                    if i & 7 == 7 {
                        let fresh7 = p_msg;
                        p_msg = p_msg.offset(1);
                        *fresh7 = the_byte
                    } else if i == 4 as c_int {
                        msglen = mode_s_message_len_by_type(the_byte as c_int);
                        if errors == 0 as c_int {
                            scanlen = msglen
                        }
                    }

                    the_byte = ((the_byte as c_int) << 1 as c_int) as u8;
                    if i < 7 {
                        the_errs = the_errs << 1;
                    }

                    // If we've exceeded the permissible number of encoding errors, abandon ship now
                    if errors > MODES_MSG_ENCODER_ERRS {
                        if i < MODES_SHORT_MSG_BITS {
                            msglen = 0;
                        } else if errors_ty == 1 && the_errs == 0x80 {
                            // If we only saw one error in the first bit of the byte of the frame, then it's possible
                            // we guessed wrongly about the value of the bit. We may be able to correct it by guessing
                            // the other way.
                            //
                            // We guessed a '1' at bit 7, which is the DF length bit == 112 Bits.
                            // Inverting bit 7 will change the message type from a long to a short.
                            // Invert the bit, cross your fingers and carry on.
                            msglen = MODES_SHORT_MSG_BITS; // revert to the number of errors prior to bit 56
                            msg[0] = (msg[0] as c_int ^ the_errs as c_int) as c_uchar;
                            errors_ty = 0;
                            errors = errors56;
                            (*mode_s).stat_df_len_corrected =
                                (*mode_s).stat_df_len_corrected.wrapping_add(1);
                        } else if i < MODES_LONG_MSG_BITS {
                            msglen = MODES_SHORT_MSG_BITS;
                            errors = errors56
                        } else {
                            msglen = MODES_LONG_MSG_BITS
                        }
                        break;
                    } else {
                        i += 1
                    }
                }

                // Ensure msglen is consistent with the DF type
                i = mode_s_message_len_by_type(msg[0] as c_int >> 3 as c_int);
                if msglen > i {
                    msglen = i
                } else if msglen < i {
                    msglen = 0 as c_int
                }

                // If we guessed at any of the bits in the DF type field, then look to see if our guess was sensible.
                // Do this by looking to see if the original guess results in the DF type being one of the ICAO defined
                // message types. If it isn't then toggle the guessed bit and see if this new value is ICAO defined.
                // if the new value is ICAO defined, then update it in our message.
                if msglen != 0 && errors_ty == 1 as c_int && the_errs as c_int & 0x78 as c_int != 0
                {
                    // We guessed at one (and only one) of the message type bits. See if our guess is "likely"
                    // to be correct by comparing the DF against a list of known good DF's
                    the_byte = msg[0 as c_int as usize]; // One bit per 32 possible DF's. Set bits 0,4,5,11,16.17.18.19,20,21,22,24
                    let mut this_df = the_byte as c_int >> 3 as c_int & 0x1f as c_int;
                    let valid_dfbits = 0x17f0831 as c_int as u32;
                    let mut this_dfbit = ((1 as c_int) << this_df) as u32;
                    if 0 as c_int as c_uint == valid_dfbits & this_dfbit {
                        // The current DF is not ICAO defined, so is probably an errors.
                        // Toggle the bit we guessed at and see if the resultant DF is more likely
                        the_byte = (the_byte as c_int ^ the_errs as c_int) as u8;
                        this_df = the_byte as c_int >> 3 as c_int & 0x1f as c_int;
                        this_dfbit = ((1 as c_int) << this_df) as u32;
                        // if this DF any more likely?
                        if valid_dfbits & this_dfbit != 0 {
                            // Yep, more likely, so update the main message
                            msg[0 as c_int as usize] = the_byte;
                            (*mode_s).stat_df_type_corrected =
                                (*mode_s).stat_df_type_corrected.wrapping_add(1);
                            errors -= 1
                            // decrease the error count so we attempt to use the modified DF.
                        }
                    }
                }

                // We measured signal strength over the first 56 bits. Don't forget to add 4
                // for the preamble samples, so round up and divide by 60.
                sig_strength = (sig_strength + 29) / 60;

                // When we reach this point, if error is small, and the signal strength is large enough
                // we may have a Mode S message on our hands. It may still be broken and the CRC may not
                // be correct, but this can be handled by the next layer.
                if msglen != 0
                    && sig_strength > MODES_MSG_SQUELCH_LEVEL
                    && errors <= MODES_MSG_ENCODER_ERRS
                {
                    // Set initial mm structure details
                    mm.timestamp_msg = (*mode_s)
                        .timestamp_blk
                        .wrapping_add(j.wrapping_mul(6) as c_ulong);
                    sig_strength = sig_strength + 0x7f >> 8;
                    mm.signal_level = u8::try_from(sig_strength).unwrap_or(std::u8::MAX);
                    mm.phase_corrected = use_correction;

                    // Decode the received message
                    decode_mode_s_message(&mut mm, msg, mode_s, bit_errors);

                    // Update statistics
                    if (*mode_s).stats != 0 {
                        if mm.crcok != 0 || use_correction != 0 || mm.correctedbits != 0 {
                            if use_correction != 0 {
                                match errors {
                                    0 => {
                                        (*mode_s).stat_ph_demodulated0 =
                                            (*mode_s).stat_ph_demodulated0.wrapping_add(1)
                                    }
                                    1 => {
                                        (*mode_s).stat_ph_demodulated1 =
                                            (*mode_s).stat_ph_demodulated1.wrapping_add(1)
                                    }
                                    2 => {
                                        (*mode_s).stat_ph_demodulated2 =
                                            (*mode_s).stat_ph_demodulated2.wrapping_add(1)
                                    }
                                    _ => {
                                        (*mode_s).stat_ph_demodulated3 =
                                            (*mode_s).stat_ph_demodulated3.wrapping_add(1)
                                    }
                                }
                            } else {
                                match errors {
                                    0 => {
                                        (*mode_s).stat_demodulated0 =
                                            (*mode_s).stat_demodulated0.wrapping_add(1)
                                    }
                                    1 => {
                                        (*mode_s).stat_demodulated1 =
                                            (*mode_s).stat_demodulated1.wrapping_add(1)
                                    }
                                    2 => {
                                        (*mode_s).stat_demodulated2 =
                                            (*mode_s).stat_demodulated2.wrapping_add(1)
                                    }
                                    _ => {
                                        (*mode_s).stat_demodulated3 =
                                            (*mode_s).stat_demodulated3.wrapping_add(1)
                                    }
                                }
                            }

                            if mm.correctedbits == 0 as c_int {
                                if use_correction != 0 {
                                    if mm.crcok != 0 {
                                        (*mode_s).stat_ph_goodcrc =
                                            (*mode_s).stat_ph_goodcrc.wrapping_add(1)
                                    } else {
                                        (*mode_s).stat_ph_badcrc =
                                            (*mode_s).stat_ph_badcrc.wrapping_add(1)
                                    }
                                } else if mm.crcok != 0 {
                                    (*mode_s).stat_goodcrc = (*mode_s).stat_goodcrc.wrapping_add(1)
                                } else {
                                    (*mode_s).stat_badcrc = (*mode_s).stat_badcrc.wrapping_add(1)
                                }
                            } else if use_correction != 0 {
                                (*mode_s).stat_ph_badcrc = (*mode_s).stat_ph_badcrc.wrapping_add(1);
                                (*mode_s).stat_ph_fixed = (*mode_s).stat_ph_fixed.wrapping_add(1);
                                if mm.correctedbits != 0
                                    && mm.correctedbits <= MODES_MAX_BITERRORS as c_int
                                {
                                    (*mode_s).stat_ph_bit_fix[(mm.correctedbits - 1) as usize] =
                                        (*mode_s).stat_ph_bit_fix[(mm.correctedbits - 1) as usize]
                                            .wrapping_add(1)
                                }
                            } else {
                                (*mode_s).stat_badcrc = (*mode_s).stat_badcrc.wrapping_add(1);
                                (*mode_s).stat_fixed = (*mode_s).stat_fixed.wrapping_add(1);
                                if mm.correctedbits != 0
                                    && mm.correctedbits <= MODES_MAX_BITERRORS as c_int
                                {
                                    (*mode_s).stat_bit_fix[(mm.correctedbits - 1) as usize] =
                                        (*mode_s).stat_bit_fix[(mm.correctedbits - 1) as usize]
                                            .wrapping_add(1)
                                }
                            }
                        }
                    }

                    // Output debug mode info if needed
                    if use_correction != 0 {
                        if (*mode_s).debug & MODES_DEBUG_DEMOD != 0 {
                            dump_raw_message(
                                b"Demodulated with 0 errors\x00".as_ptr() as *const c_char,
                                msg.as_mut_ptr(),
                                m,
                                j,
                            );
                        } else if (*mode_s).debug & MODES_DEBUG_BADCRC != 0
                            && mm.msgtype == 17 as c_int
                            && (mm.crcok == 0 || mm.correctedbits != 0 as c_int)
                        {
                            dump_raw_message(
                                b"Decoded with bad CRC\x00".as_ptr() as *const c_char,
                                msg.as_mut_ptr(),
                                m,
                                j,
                            );
                        } else if (*mode_s).debug & MODES_DEBUG_GOODCRC != 0
                            && mm.crcok != 0
                            && mm.correctedbits == 0 as c_int
                        {
                            dump_raw_message(
                                b"Decoded with good CRC\x00".as_ptr() as *const c_char,
                                msg.as_mut_ptr(),
                                m,
                                j,
                            );
                        }
                    }

                    // Skip this message if we are sure it's fine
                    if mm.crcok != 0 {
                        j += (MODES_PREAMBLE_US as u32 + msglen as u32) * 2 - 1;
                    }

                    // Pass data to the next layer
                    use_modes_message(mode_s, &mut mm);
                } else if (*mode_s).debug & MODES_DEBUG_DEMODERR != 0 && use_correction != 0 {
                    println!("The following message has {} demod errors", errors);
                    dump_raw_message(
                        b"Demodulated with errors\x00".as_ptr() as *const c_char,
                        msg.as_mut_ptr(),
                        m,
                        j,
                    );
                }

                // Retry with phase correction if enabled, necessary and possible.
                if (*mode_s).phase_enhance != 0
                    && mm.crcok == 0
                    && mm.correctedbits == 0
                    && use_correction == 0
                    && j != 0
                {
                    // FIXME: This is terrible (j - 1 business)
                    let src = &m_slice[usize::try_from(j - 1).unwrap()..];
                    if detect_out_of_phase(src) != 0 {
                        use_correction = 1;
                        j = j.wrapping_sub(1)
                    } else {
                        use_correction = 0;
                    }
                } else {
                    use_correction = 0;
                }
            }
            _ => {}
        }
        j = j.wrapping_add(1)
    }

    // Send any remaining partial raw buffers now
    if (*mode_s).raw_out_used != 0 || (*mode_s).beast_out_used != 0 {
        (*mode_s).net_output_raw_rate_count += 1;
        if (*mode_s).net_output_raw_rate_count > (*mode_s).net_output_raw_rate {
            if (*mode_s).raw_out_used != 0 {
                // modesSendAllClients(
                //     (*Modes).ros,
                //     (*Modes).raw_out as *mut c_void,
                //     (*Modes).raw_out_used,
                // );
                (*mode_s).raw_out_used = 0 as c_int
            }

            if (*mode_s).beast_out_used != 0 {
                // modesSendAllClients(
                //     (*Modes).bos,
                //     (*Modes).beast_out as *mut c_void,
                //     (*Modes).beast_out_used,
                // );
                (*mode_s).beast_out_used = 0 as c_int
            }
            (*mode_s).net_output_raw_rate_count = 0 as c_int
        }
    }
}

// When a new message is available, because it was decoded from the RTL device,
// file, or received in the TCP input port, or any other way we can receive a
// decoded message, we call this function in order to use the message.
//
// Basically this function passes a raw message to the upper layers for further
// processing and visualization
//
fn use_modes_message(mode_s: &mut ModeS, mm: &mut ModesMessage) {
    if mode_s.check_crc == 0 || mm.crcok != 0 || mm.correctedbits != 0 {
        // not checking, ok or fixed

        // Always track Aircraft
        interactive_receive_data(mode_s, mm);

        // In non-interactive non-quiet mode, display messages on standard output
        if mode_s.interactive == 0 && mode_s.quiet == 0 {
            display_modes_message(mode_s, mm);
        }
    };
}

// Always positive MOD operation, used for CPR decoding.
//
fn cpr_mod_function(a: c_int, b: c_int) -> c_int {
    let res = a % b;
    if res < 0 {
        res + b
    } else {
        res
    }
}

// The NL function uses the precomputed table from 1090-WP-9-14
//
fn cpr_nl_function(mut lat: c_double) -> c_int {
    if lat < 0.0 {
        // Table is symmetric about the equator
        lat = -lat
    }
    if lat < 10.47047130f64 {
        return 59;
    }
    if lat < 14.82817437f64 {
        return 58;
    }
    if lat < 18.18626357f64 {
        return 57;
    }
    if lat < 21.02939493f64 {
        return 56;
    }
    if lat < 23.54504487f64 {
        return 55;
    }
    if lat < 25.82924707f64 {
        return 54;
    }
    if lat < 27.93898710f64 {
        return 53;
    }
    if lat < 29.91135686f64 {
        return 52;
    }
    if lat < 31.77209708f64 {
        return 51;
    }
    if lat < 33.53993436f64 {
        return 50;
    }
    if lat < 35.22899598f64 {
        return 49;
    }
    if lat < 36.85025108f64 {
        return 48;
    }
    if lat < 38.41241892f64 {
        return 47;
    }
    if lat < 39.92256684f64 {
        return 46;
    }
    if lat < 41.38651832f64 {
        return 45;
    }
    if lat < 42.80914012f64 {
        return 44;
    }
    if lat < 44.19454951f64 {
        return 43;
    }
    if lat < 45.54626723f64 {
        return 42;
    }
    if lat < 46.86733252f64 {
        return 41;
    }
    if lat < 48.16039128f64 {
        return 40;
    }
    if lat < 49.42776439f64 {
        return 39;
    }
    if lat < 50.67150166f64 {
        return 38;
    }
    if lat < 51.89342469f64 {
        return 37;
    }
    if lat < 53.09516153f64 {
        return 36;
    }
    if lat < 54.27817472f64 {
        return 35;
    }
    if lat < 55.44378444f64 {
        return 34;
    }
    if lat < 56.59318756f64 {
        return 33;
    }
    if lat < 57.72747354f64 {
        return 32;
    }
    if lat < 58.84763776f64 {
        return 31;
    }
    if lat < 59.95459277f64 {
        return 30;
    }
    if lat < 61.04917774f64 {
        return 29;
    }
    if lat < 62.13216659f64 {
        return 28;
    }
    if lat < 63.20427479f64 {
        return 27;
    }
    if lat < 64.26616523f64 {
        return 26;
    }
    if lat < 65.31845310f64 {
        return 25;
    }
    if lat < 66.36171008f64 {
        return 24;
    }
    if lat < 67.39646774f64 {
        return 23;
    }
    if lat < 68.42322022f64 {
        return 22;
    }
    if lat < 69.44242631f64 {
        return 21;
    }
    if lat < 70.45451075f64 {
        return 20;
    }
    if lat < 71.45986473f64 {
        return 19;
    }
    if lat < 72.45884545f64 {
        return 18;
    }
    if lat < 73.45177442f64 {
        return 17;
    }
    if lat < 74.43893416f64 {
        return 16;
    }
    if lat < 75.42056257f64 {
        return 15;
    }
    if lat < 76.39684391f64 {
        return 14;
    }
    if lat < 77.36789461f64 {
        return 13;
    }
    if lat < 78.33374083f64 {
        return 12;
    }
    if lat < 79.29428225f64 {
        return 11;
    }
    if lat < 80.24923213f64 {
        return 10;
    }
    if lat < 81.19801349f64 {
        return 9;
    }
    if lat < 82.13956981f64 {
        return 8;
    }
    if lat < 83.07199445f64 {
        return 7;
    }
    if lat < 83.99173563f64 {
        return 6;
    }
    if lat < 84.89166191f64 {
        return 5;
    }
    if lat < 85.75541621f64 {
        return 4;
    }
    if lat < 86.53536998f64 {
        return 3;
    }
    if lat < 87.00000000f64 {
        return 2;
    } else {
        return 1;
    };
}

fn cpr_n_function(lat: c_double, fflag: c_int) -> c_int {
    let nl = cpr_nl_function(lat) - if fflag != 0 { 1 } else { 0 };
    if nl < 1 {
        1
    } else {
        nl
    }
}

fn cpr_dlon_function(lat: c_double, fflag: c_int, surface: c_int) -> c_double {
    (if surface != 0 { 90.0f64 } else { 360.0f64 }) / cpr_n_function(lat, fflag) as c_double
}

pub(crate) fn decode_cpr(mode_s: &ModeS, a: &mut Aircraft, fflag: c_int, surface: c_int) -> c_int {
    let air_dlat0 = (if surface != 0 { 90.0f64 } else { 360.0f64 }) / 60.0f64;
    let air_dlat1 = (if surface != 0 { 90.0f64 } else { 360.0f64 }) / 59.0f64;
    let lat0 = a.even_cprlat as c_double;
    let lat1 = a.odd_cprlat as c_double;
    let lon0 = a.even_cprlon as c_double;
    let lon1 = a.odd_cprlon as c_double;

    // Compute the Latitude Index "j"
    let j = ((59 as c_int as c_double * lat0 - 60 as c_int as c_double * lat1)
        / 131072 as c_int as c_double
        + 0.5f64)
        .floor() as c_int;
    let mut rlat0 = air_dlat0
        * (cpr_mod_function(j, 60 as c_int) as c_double + lat0 / 131072 as c_int as c_double);
    let mut rlat1 = air_dlat1
        * (cpr_mod_function(j, 59 as c_int) as c_double + lat1 / 131072 as c_int as c_double);

    let now = crate::now() as i64;
    let mut surface_rlat = MODES_USER_LATITUDE_DFLT;
    let mut surface_rlon = MODES_USER_LONGITUDE_DFLT;

    if surface != 0 {
        // If we're on the ground, make sure we have a (likely) valid Lat/Lon
        if a.b_flags & MODES_ACFLAGS_LATLON_VALID != 0
            && ((now - a.seen_lat_lon) as c_int) < mode_s.interactive_display_ttl
        {
            surface_rlat = a.lat;
            surface_rlon = a.lon
        } else if mode_s.b_user_flags & MODES_USER_LATLON_VALID != 0 {
            surface_rlat = mode_s.f_user_lat;
            surface_rlon = mode_s.f_user_lon
        } else {
            // No local reference, give up
            return -1;
        }
        rlat0 += (surface_rlat / 90.0f64).floor() * 90.0f64; // Move from 1st quadrant to our quadrant
        rlat1 += (surface_rlat / 90.0f64).floor() * 90.0f64
    } else {
        if rlat0 >= 270 as c_int as c_double {
            rlat0 -= 360 as c_int as c_double
        }
        if rlat1 >= 270 as c_int as c_double {
            rlat1 -= 360 as c_int as c_double
        }
    }

    // Check to see that the latitude is in range: -90 .. +90
    if rlat0 < -(90 as c_int) as c_double
        || rlat0 > 90 as c_int as c_double
        || rlat1 < -(90 as c_int) as c_double
        || rlat1 > 90 as c_int as c_double
    {
        return -1;
    }

    // Check that both are in the same latitude zone, or abort.
    if cpr_nl_function(rlat0) != cpr_nl_function(rlat1) {
        return -1;
    }

    // Compute ni and the Longitude Index "m"
    if fflag != 0 {
        // Use odd packet.
        let ni = cpr_n_function(rlat1, 1 as c_int); // Use even packet.
        let m = ((lon0 * (cpr_nl_function(rlat1) - 1 as c_int) as c_double
            - lon1 * cpr_nl_function(rlat1) as c_double)
            / 131072.0f64
            + 0.5f64)
            .floor() as c_int;
        a.lon = cpr_dlon_function(rlat1, 1 as c_int, surface)
            * (cpr_mod_function(m, ni) as c_double + lon1 / 131072 as c_int as c_double);
        a.lat = rlat1
    } else {
        // Use even packet
        let ni_0 = cpr_n_function(rlat0, 0 as c_int);
        let m_0 = ((lon0 * (cpr_nl_function(rlat0) - 1 as c_int) as c_double
            - lon1 * cpr_nl_function(rlat0) as c_double)
            / 131072 as c_int as c_double
            + 0.5f64)
            .floor() as c_int;
        a.lon = cpr_dlon_function(rlat0, 0 as c_int, surface)
            * (cpr_mod_function(m_0, ni_0) as c_double + lon0 / 131072 as c_int as c_double);
        a.lat = rlat0
    }

    if surface != 0 {
        // Move from 1st quadrant to our quadrant
        a.lon += (surface_rlon / 90.0f64).floor() * 90.0f64
    } else if a.lon > 180 as c_int as c_double {
        a.lon -= 360 as c_int as c_double
    }

    a.seen_lat_lon = a.seen;
    a.timestamp_lat_lon = a.timestamp;
    a.b_flags |= MODES_ACFLAGS_LATLON_VALID | MODES_ACFLAGS_LATLON_REL_OK;

    0
}

// This algorithm comes from:
// 1090-WP29-07-Draft_CPR101 (which also defines decode_cpr() )
//
// There is an error in this document related to CPR relative decode.
// Should use trunc() rather than the floor() function in Eq 38 and related for deltaZI.
// floor() returns integer less than argument
// trunc() returns integer closer to zero than argument.
// Note:   text of document describes trunc() functionality for deltaZI calculation
//         but the formulae use floor().
//
pub(crate) fn decode_cpr_relative(
    mode_s: &ModeS,
    a: &mut Aircraft,
    fflag: c_int,
    surface: c_int,
) -> c_int {
    let air_dlat: c_double;
    let air_dlon: c_double;
    let lat: c_double;
    let lon: c_double;
    let lonr: c_double;
    let latr: c_double;
    let mut rlon: c_double;
    let mut rlat: c_double;
    let j: c_int;
    let m: c_int;

    if a.b_flags & MODES_ACFLAGS_LATLON_REL_OK != 0 {
        // Ok to try Aircraft relative first
        latr = a.lat;
        lonr = a.lon
    } else if mode_s.b_user_flags & MODES_USER_LATLON_VALID != 0 {
        // Try ground station relative next
        latr = mode_s.f_user_lat;
        lonr = mode_s.f_user_lon
    } else {
        // Exit with error - can't do relative if we don't have ref.
        return -1;
    }

    if fflag != 0 {
        // odd
        air_dlat = (if surface != 0 { 90.0f64 } else { 360.0f64 }) / 59.0f64;
        lat = a.odd_cprlat as c_double;
        lon = a.odd_cprlon as c_double
    } else {
        // even
        air_dlat = (if surface != 0 { 90.0f64 } else { 360.0f64 }) / 60.0f64;
        lat = a.even_cprlat as c_double;
        lon = a.even_cprlon as c_double
    }

    // Compute the Latitude Index "j"
    j = ((latr / air_dlat).floor()
        + (0.5f64 + cpr_mod_function(latr as c_int, air_dlat as c_int) as c_double / air_dlat
            - lat / 131072 as c_int as c_double)
            .trunc()) as c_int;
    rlat = air_dlat * (j as c_double + lat / 131072 as c_int as c_double);
    if rlat >= 270 as c_int as c_double {
        rlat -= 360 as c_int as c_double
    }

    // Check to see that the latitude is in range: -90 .. +90
    if rlat < -(90 as c_int) as c_double || rlat > 90 as c_int as c_double {
        // This will cause a quick exit next time if no global has been done
        // Time to give up - Latitude error
        a.b_flags &= !MODES_ACFLAGS_LATLON_REL_OK;
        return -1;
    }

    // Check to see that answer is reasonable - ie no more than 1/2 cell away
    if (rlat - a.lat).abs() > air_dlat / 2 as c_int as c_double {
        // This will cause a quick exit next time if no global has been done
        // Time to give up - Latitude error
        a.b_flags &= !MODES_ACFLAGS_LATLON_REL_OK;
        return -1;
    }

    // Compute the Longitude Index "m"
    air_dlon = cpr_dlon_function(rlat, fflag, surface);
    m = ((lonr / air_dlon).floor()
        + (0.5f64 + cpr_mod_function(lonr as c_int, air_dlon as c_int) as c_double / air_dlon
            - lon / 131072 as c_int as c_double)
            .trunc()) as c_int;
    rlon = air_dlon * (m as c_double + lon / 131072 as c_int as c_double);
    if rlon > 180 as c_int as c_double {
        rlon -= 360 as c_int as c_double
    }

    // Check to see that answer is reasonable - ie no more than 1/2 cell away
    if (rlon - a.lon).abs() > air_dlon / 2 as c_int as c_double {
        // This will cause a quick exit next time if no global has been done
        // Time to give up - Longitude error
        a.b_flags &= !MODES_ACFLAGS_LATLON_REL_OK;
        return -1;
    }

    a.lat = rlat;
    a.lon = rlon;

    a.seen_lat_lon = a.seen;
    a.timestamp_lat_lon = a.timestamp;
    a.b_flags |= MODES_ACFLAGS_LATLON_VALID | MODES_ACFLAGS_LATLON_REL_OK;

    0
}

fn dump_raw_message(descr: *const c_char, _msg: *mut c_uchar, _m: *const u16, _offset: u32) {
    // printf("\n--- %s\n    ", descr);
    // for (j = 0; j < MODES_LONG_MSG_BYTES; j++) {
    //     printf("%02x",msg[j]);
    //     if (j == MODES_SHORT_MSG_BYTES-1) printf(" ... ");
    // }
    // printf(" (DF %d, Fixable: %d)\n", msgtype, fixable);
    // dumpMagnitudeVector(m,offset);
    // printf("---\n\n");
    let desc = unsafe { CStr::from_ptr(descr) }
        .to_str()
        .unwrap_or("{invalid}");
    eprintln!("dump_raw_message: {}", desc);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_format_raw_message_at() {
        let mode_s = ModeS {
            mlat: 1,
            ..ModeS::default()
        };
        let mm = ModesMessage {
            timestamp_msg: 0xABCDEF12,
            msgbits: 16,
            msg: [0xC, 0xA, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            ..ModesMessage::default()
        };
        assert_eq!(&format_raw_message(&mode_s, &mm), "@0000ABCDEF120c0a;");
    }

    #[test]
    fn test_format_raw_message_star() {
        let mode_s = ModeS {
            mlat: 0,
            ..ModeS::default()
        };
        let mm = ModesMessage {
            timestamp_msg: 0xABCDEF12,
            msgbits: 112,
            msg: [
                0x8d, 0x4d, 0x20, 0x23, 0x99, 0x10, 0x94, 0xad, 0x48, 0x7c, 0x14, 0xfc, 0x9e, 0x3d,
            ],
            ..ModesMessage::default()
        };
        assert_eq!(
            &format_raw_message(&mode_s, &mm),
            "*8d4d2023991094ad487c14fc9e3d;"
        );
    }

    #[test]
    fn test_mode_s_checksum() {
        let crc = mode_s_checksum(
            [141, 124, 106, 87, 88, 15, 138, 214, 205, 218, 12, 36, 8, 66],
            112,
        );
        assert_eq!(crc, 0);
        let crc = mode_s_checksum(
            [
                93, 124, 106, 87, 253, 220, 242, 214, 205, 218, 12, 36, 8, 66,
            ],
            112,
        );
        assert_eq!(crc, 10540941);
        let crc = mode_s_checksum(
            [
                143, 77, 32, 35, 88, 117, 240, 185, 87, 153, 244, 39, 139, 226,
            ],
            112,
        );
        assert_eq!(crc, 0);
        let crc = mode_s_checksum(
            [95, 77, 32, 35, 45, 175, 0, 185, 61, 153, 252, 173, 217, 159],
            112,
        );
        assert_eq!(crc, 11946508);
    }
}
