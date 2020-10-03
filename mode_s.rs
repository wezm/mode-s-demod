// ===================== Mode S detection and decoding  ===================

use std::cmp::Ordering;
use std::os::raw::{c_char, c_int, c_uchar, c_uint};
use std::time::SystemTime;
use std::{ptr, time};

use crate::mode_ac::ModeAToModeC;
use crate::{modes, modesMessage};

const MODES_LONG_MSG_BYTES: c_int = 14;
const MODES_SHORT_MSG_BYTES: c_int = 7;
const MODES_LONG_MSG_BITS: c_int = MODES_LONG_MSG_BYTES * 8;
const MODES_SHORT_MSG_BITS: c_int = MODES_SHORT_MSG_BYTES * 8;

const MODES_ICAO_CACHE_LEN: u32 = 1024; // Value must be a power of two
const MODES_ICAO_CACHE_TTL: u64 = 60; // Time to live of cached addresses
const MODES_UNIT_FEET: c_int = 0;
const MODES_UNIT_METERS: c_int = 1;

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

extern "C" {}

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
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(C)]
pub struct errorinfo {
    pub syndrome: u32,
    pub bits: c_int,
    pub pos: [c_int; 2],
}

// TODO: Change input to have a known length so we can get rid of pointer derefs and unsafe
#[no_mangle]
pub unsafe extern "C" fn modesChecksum(mut msg: *mut c_uchar, mut bits: c_int) -> u32 {
    let mut crc: u32 = 0;
    let mut offset = if bits == 112 { 0 } else { 112 - 56 };
    let mut the_byte: u8 = *msg;
    let mut j: c_int = 0;

    // We don't really need to include the checksum itself
    bits -= 24;
    while j < bits {
        if j & 7 == 0 {
            let fresh0 = msg;
            msg = msg.offset(1);
            the_byte = *fresh0
        }
        // If bit is set, xor with corresponding table entry.
        if the_byte as c_int & 0x80 != 0 {
            crc ^= MODES_CHECKSUM_TABLE[offset] // FIXME: bounds
        } // message checksum
        offset += 1;
        the_byte = ((the_byte as c_int) << 1 as c_int) as u8;
        j += 1
    }

    let rem = ((*msg.offset(0) as c_int) << 16 as c_int
        | (*msg.offset(1) as c_int) << 8 as c_int
        | *msg.offset(2) as c_int) as u32;
    return (crc ^ rem) & 0xffffff as c_int as c_uint; // 24 bit checksum syndrome.
}

// Given the Downlink Format (DF) of the message, return the message length in bits.
//
// All known DF's 16 or greater are long. All known DF's 15 or less are short.
// There are lots of unused codes in both category, so we can assume ICAO will stick to
// these rules, meaning that the most significant bit of the DF indicates the length.
#[no_mangle]
pub extern "C" fn modesMessageLenByType(type_: c_int) -> c_int {
    if type_ & 0x10 == 0x10 {
        MODES_LONG_MSG_BITS
    } else {
        MODES_SHORT_MSG_BITS
    }
}

fn cmp_errorinfo(e0: &errorinfo, e1: &errorinfo) -> Ordering {
    e0.syndrome.cmp(&e1.syndrome)
}

// Compute the table of all syndromes for 1-bit and 2-bit error vectors
#[no_mangle]
pub unsafe extern "C" fn modesInitErrorInfoImpl(
    table_ptr: *mut errorinfo,
    table_len: c_int,
    nfix_crc: c_int,
) {
    let bitErrorTable = &mut *ptr::slice_from_raw_parts_mut(table_ptr, table_len as usize);
    let mut msg: [c_uchar; 14] = [0; MODES_LONG_MSG_BYTES as usize];
    let mut j: c_int;
    let mut n: c_int = 0;
    let mut crc: u32;

    // Add all possible single and double bit errors
    // don't include errors in first 5 bits (DF type)
    let mut i = 5 as c_int;
    while i < MODES_LONG_MSG_BITS {
        let bytepos0: c_int = i >> 3 as c_int;
        let mask0: c_int = (1 as c_int) << 7 as c_int - (i & 7 as c_int);
        // revert error0
        msg[bytepos0 as usize] = (msg[bytepos0 as usize] as c_int ^ mask0) as c_uchar; // create error0
        crc = modesChecksum(msg.as_mut_ptr(), MODES_LONG_MSG_BITS); // single bit error case
        bitErrorTable[n as usize].syndrome = crc;
        bitErrorTable[n as usize].bits = 1 as c_int;
        bitErrorTable[n as usize].pos[0 as c_int as usize] = i;
        bitErrorTable[n as usize].pos[1 as c_int as usize] = -(1 as c_int);
        n += 1 as c_int;
        if nfix_crc > 1 as c_int {
            j = i + 1 as c_int;
            while j < MODES_LONG_MSG_BITS {
                let bytepos1: c_int = j >> 3 as c_int;
                let mask1: c_int = (1 as c_int) << 7 as c_int - (j & 7 as c_int);
                // revert error1
                msg[bytepos1 as usize] = (msg[bytepos1 as usize] as c_int ^ mask1) as c_uchar; // create error1
                crc = modesChecksum(msg.as_mut_ptr(), MODES_LONG_MSG_BITS); // two bit error case
                if n >= bitErrorTable.len() as c_int {
                    break;
                }
                bitErrorTable[n as usize].syndrome = crc;
                bitErrorTable[n as usize].bits = 2 as c_int;
                bitErrorTable[n as usize].pos[0 as c_int as usize] = i;
                bitErrorTable[n as usize].pos[1 as c_int as usize] = j;
                n += 1 as c_int;
                msg[bytepos1 as usize] = (msg[bytepos1 as usize] as c_int ^ mask1) as c_uchar;
                j += 1
            }
        }
        msg[bytepos0 as usize] = (msg[bytepos0 as usize] as c_int ^ mask0) as c_uchar;
        i += 1
    }

    bitErrorTable.sort_by(cmp_errorinfo)
}

// Search for syndrome in table and if an entry is found, flip the necessary
// bits. Make sure the indices fit into the array
// Additional parameter: fix only less than maxcorrected bits, and record
// fixed bit positions in corrected[]. This array can be NULL, otherwise
// must be of length at least maxcorrected.
// Return number of fixed bits.
//
#[no_mangle]
pub unsafe extern "C" fn fixBitErrorsImpl(
    msg: *mut c_uchar,
    bits: c_int,
    maxfix: c_int,
    fixedbits: *mut c_char,
    table_ptr: *const errorinfo,
    table_len: c_int,
) -> c_int {
    let bitErrorTable = &*ptr::slice_from_raw_parts(table_ptr, table_len as usize);
    let mut bitpos;
    let syndrome = modesChecksum(msg, bits);
    let ei = match bitErrorTable.binary_search_by(|e| e.syndrome.cmp(&syndrome)) {
        Ok(index) => &bitErrorTable[index],
        Err(_) => return 0, // No syndrome found
    };

    // Check if the syndrome fixes more bits than we allow
    if maxfix < ei.bits {
        return 0;
    }

    // Check that all bit positions lie inside the message length
    let offset = MODES_LONG_MSG_BITS - bits;
    let mut i = 0 as c_int;
    while i < ei.bits {
        bitpos = ei.pos[i as usize] - offset;
        if bitpos < 0 as c_int || bitpos >= bits {
            return 0 as c_int;
        }
        i += 1
    }

    // Fix the bits
    let mut res = 0 as c_int;
    i = res;
    while i < ei.bits {
        bitpos = ei.pos[i as usize] - offset;
        let ref mut fresh1 = *msg.offset((bitpos >> 3 as c_int) as isize);
        *fresh1 =
            (*fresh1 as c_int ^ (1 as c_int) << 7 as c_int - (bitpos & 7 as c_int)) as c_uchar;
        if !fixedbits.is_null() {
            let fresh2 = res;
            res = res + 1;
            *fixedbits.offset(fresh2 as isize) = bitpos as c_char
        }
        i += 1
    }
    return res;
}

// Hash the ICAO address to index our cache of MODES_ICAO_CACHE_LEN
// elements, that is assumed to be a power of two
#[no_mangle]
pub unsafe extern "C" fn ICAOCacheHashAddress(mut a: u32) -> u32 {
    // The following three rounds wil make sure that every bit affects
    // every output bit with ~ 50% of probability.
    a = (a >> 16 as c_int ^ a).wrapping_mul(0x45d9f3b as c_int as c_uint);
    a = (a >> 16 as c_int ^ a).wrapping_mul(0x45d9f3b as c_int as c_uint);
    a = a >> 16 as c_int ^ a;
    a & (MODES_ICAO_CACHE_LEN - 1)
}

// Add the specified entry to the cache of recently seen ICAO addresses.
// Note that we also add a timestamp so that we can make sure that the
// entry is only valid for MODES_ICAO_CACHE_TTL seconds.
//
#[no_mangle]
pub unsafe extern "C" fn addRecentlySeenICAOAddrImpl(this: *mut modes, addr: u32) {
    let h: u32 = ICAOCacheHashAddress(addr);
    *(*this)
        .icao_cache
        .offset(h.wrapping_mul(2 as c_int as c_uint) as isize) = addr;
    // Need seconds since epoch as a u32 to replace time(NULL) call.
    let now = match SystemTime::now().duration_since(time::UNIX_EPOCH) {
        Ok(n) => n.as_secs(),
        Err(_) => panic!("SystemTime before UNIX EPOCH!"),
    };
    *(*this).icao_cache.offset(
        h.wrapping_mul(2 as c_int as c_uint)
            .wrapping_add(1 as c_int as c_uint) as isize,
    ) = now as u32; // FIXME: change to 64-bit time
}

// Returns 1 if the specified ICAO address was seen in a DF format with
// proper checksum (not xored with address) no more than * MODES_ICAO_CACHE_TTL
// seconds ago. Otherwise returns 0.
//
#[no_mangle]
pub unsafe extern "C" fn ICAOAddressWasRecentlySeenImpl(this: *const modes, addr: u32) -> c_int {
    let h: u32 = ICAOCacheHashAddress(addr);
    let a: u32 = *(*this).icao_cache.offset(h.wrapping_mul(2) as isize);
    let t: u32 = *(*this)
        .icao_cache
        .offset(h.wrapping_mul(2).wrapping_add(1) as isize);
    let tn = match SystemTime::now().duration_since(time::UNIX_EPOCH) {
        Ok(n) => n.as_secs(),
        Err(_) => panic!("SystemTime before UNIX EPOCH!"),
    };
    (a != 0 && a == addr && tn.wrapping_sub(u64::from(t)) <= MODES_ICAO_CACHE_TTL) as c_int
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
#[no_mangle]
pub extern "C" fn decodeID13Field(ID13Field: c_int) -> c_int {
    let mut hexGillham = 0;
    if ID13Field & 0x1000 != 0 { hexGillham |= 0x0010 } // Bit 12 = C1
    if ID13Field & 0x0800 != 0 { hexGillham |= 0x1000 } // Bit 11 = A1
    if ID13Field & 0x0400 != 0 { hexGillham |= 0x0020 } // Bit 10 = C2
    if ID13Field & 0x0200 != 0 { hexGillham |= 0x2000 } // Bit  9 = A2
    if ID13Field & 0x0100 != 0 { hexGillham |= 0x0040 } // Bit  8 = C4
    if ID13Field & 0x0080 != 0 { hexGillham |= 0x4000 } // Bit  7 = A4
    // TODO: Find out why bit 6 was commented out in the C code
    // if (ID13Field & 0x0040) {hexGillham |= 0x0800;}  // Bit  6 = X  or M
    if ID13Field & 0x0020 != 0 { hexGillham |= 0x0100 } // Bit  5 = B1
    if ID13Field & 0x0010 != 0 { hexGillham |= 0x0001 } // Bit  4 = D1 or Q
    if ID13Field & 0x0008 != 0 { hexGillham |= 0x0200 } // Bit  3 = B2
    if ID13Field & 0x0004 != 0 { hexGillham |= 0x0002 } // Bit  2 = D2
    if ID13Field & 0x0002 != 0 { hexGillham |= 0x0400 } // Bit  1 = B4
    if ID13Field & 0x0001 != 0 { hexGillham |= 0x0004 } // Bit  0 = D4
    hexGillham
}

// Decode the 13 bit AC altitude field (in DF 20 and others).
// Returns the altitude, and set 'unit' to either MODES_UNIT_METERS or MDOES_UNIT_FEETS.
//
#[no_mangle]
pub unsafe extern "C" fn decodeAC13Field(AC13Field: c_int, unit: *mut c_int) -> c_int {
    let m_bit = (AC13Field & 0x40) != 0; // set = meters, clear = feet
    let q_bit = (AC13Field & 0x10) != 0; // set = 25 ft encoding, clear = Gillham Mode C encoding
    if !m_bit {
        *unit = MODES_UNIT_FEET;
        if q_bit {
            // N is the 11 bit integer resulting from the removal of bit Q and M
            let n: c_int = (AC13Field & 0x1f80) >> 2 | (AC13Field & 0x20) >> 1 | AC13Field & 0xf;
            // The final altitude is resulting number multiplied by 25, minus 1000.
            n * 25 - 1000
        } else {
            // N is an 11 bit Gillham coded altitude
            let mut n_0: c_int = ModeAToModeC(decodeID13Field(AC13Field) as c_uint);
            if n_0 < -12 {
                n_0 = 0
            }
            100 * n_0
        }
    } else {
        *unit = MODES_UNIT_METERS;
        // TODO(inherited): Implement altitude when meter unit is selected
        0
    }
}

// Decode the 12 bit AC altitude field (in DF 17 and others).
//
#[no_mangle]
pub unsafe extern "C" fn decodeAC12Field(AC12Field: c_int, unit: *mut c_int) -> c_int {
    let q_bit = (AC12Field & 0x10) != 0; // Bit 48 = Q
    *unit = MODES_UNIT_FEET;
    if q_bit {
        // / N is the 11 bit integer resulting from the removal of bit Q at bit 4
        let n: c_int = (AC12Field & 0xfe0) >> 1 | AC12Field & 0xf;
        // The final altitude is the resulting number multiplied by 25, minus 1000.
        n * 25 - 1000
    } else {
        // Make N a 13 bit Gillham coded altitude by inserting M=0 at bit 6
        let mut n_0: c_int = (AC12Field & 0xfc0) << 1 | AC12Field & 0x3f;
        n_0 = ModeAToModeC(decodeID13Field(n_0) as c_uint);
        if n_0 < -12 {
            n_0 = 0
        }
        100 * n_0
    }
}

// FIXME: this function has no test coverage
// Decode the 7 bit ground movement field PWL exponential style scale
//
#[no_mangle]
pub unsafe extern "C" fn decodeMovementField(movement: c_int) -> c_int {
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

// Decode a raw Mode S message demodulated as a stream of bytes by detectModeS(),
// and split it into fields populating a modesMessage structure.
//
#[no_mangle]
pub unsafe extern "C" fn decodeModesMessageImpl(
    mut mm: *mut modesMessage,
    msg: *const c_uchar,
    Modes: *mut modes,
    bit_errors_ptr: *const errorinfo,
    bit_errors_len: c_int,
) {
    let ais_charset: *mut c_char =
        b"?ABCDEFGHIJKLMNOPQRSTUVWXYZ????? ???????????????0123456789??????\x00" as *const u8
            as *const c_char as *mut c_char;

    // Work on our local copy
    ptr::copy_nonoverlapping(msg, (*mm).msg.as_mut_ptr(), MODES_LONG_MSG_BYTES as usize);
    let msg = (*mm).msg.as_mut_ptr();

    // Get the message type ASAP as other operations depend on this
    (*mm).msgtype = c_int::from(*msg.offset(0)) >> 3; // Downlink Format
    (*mm).msgbits = modesMessageLenByType((*mm).msgtype);
    (*mm).crc = modesChecksum(msg, (*mm).msgbits);

    if (*mm).crc != 0 && (*Modes).nfix_crc != 0 && ((*mm).msgtype == 17 || (*mm).msgtype == 18) {
        //  if ((mm->crc) && (Modes.nfix_crc) && ((mm->msgtype == 11) || (mm->msgtype == 17))) {
        //
        // Fixing single bit errors in DF-11 is a bit dodgy because we have no way to
        // know for sure if the crc is supposed to be 0 or not - it could be any value
        // less than 80. Therefore, attempting to fix DF-11 errors can result in a
        // multitude of possible crc solutions, only one of which is correct.
        //
        // We should probably perform some sanity checks on corrected DF-11's before
        // using the results. Perhaps check the ICAO against known aircraft, and check
        // IID against known good IID's. That's a TODO.
        //
        (*mm).correctedbits = fixBitErrorsImpl(
            msg,
            (*mm).msgbits,
            (*Modes).nfix_crc,
            (*mm).corrected.as_mut_ptr(),
            bit_errors_ptr,
            bit_errors_len,
        );

        // If we correct, validate ICAO addr to help filter birthday paradox solutions.
        if (*mm).correctedbits != 0 {
            let ulAddr: u32 = ((*msg.offset(1) as c_int) << 16 as c_int
                | (*msg.offset(2) as c_int) << 8 as c_int
                | *msg.offset(3) as c_int) as u32;
            if ICAOAddressWasRecentlySeenImpl(Modes, ulAddr) == 0 {
                (*mm).correctedbits = 0;
            }
        }
    }

    // Note that most of the other computation happens *after* we fix the
    // single/two bit errors, otherwise we would need to recompute the fields again.
    match (*mm).msgtype {
        11 => {
            // DF 11
            (*mm).iid = (*mm).crc as c_int; // Responder capabilities
            (*mm).addr = ((*msg.offset(1) as c_int) << 16 as c_int
                | (*msg.offset(2) as c_int) << 8 as c_int
                | *msg.offset(3) as c_int) as u32;
            (*mm).ca = *msg.offset(0) as c_int & 0x7 as c_int;
            (*mm).crcok = (0 as c_int as c_uint == (*mm).crc) as c_int;
            if (*mm).crcok != 0 {
                // DF 11 : if crc == 0 try to populate our ICAO addresses whitelist.
                addRecentlySeenICAOAddrImpl(Modes, (*mm).addr);
            } else if (*mm).crc < 80 as c_int as c_uint {
                (*mm).crcok = ICAOAddressWasRecentlySeenImpl(Modes, (*mm).addr);
                if (*mm).crcok != 0 {
                    addRecentlySeenICAOAddrImpl(Modes, (*mm).addr);
                }
            }
        }
        17 => {
            // DF 17
            (*mm).addr = ((*msg.offset(1) as c_int) << 16 as c_int
                | (*msg.offset(2) as c_int) << 8 as c_int
                | *msg.offset(3) as c_int) as u32; // Responder capabilities
            (*mm).ca = *msg.offset(0) as c_int & 0x7 as c_int;
            (*mm).crcok = (0 as c_int as c_uint == (*mm).crc) as c_int;
            if (*mm).crcok != 0 {
                // DF 17 : if crc == 0 try to populate our ICAO addresses whitelist.
                addRecentlySeenICAOAddrImpl(Modes, (*mm).addr); // All other DF's
            }
        }
        18 => {
            // DF 18
            (*mm).addr = ((*msg.offset(1) as c_int) << 16 as c_int
                | (*msg.offset(2) as c_int) << 8 as c_int
                | *msg.offset(3) as c_int) as u32; // Control Field
            (*mm).ca = *msg.offset(0) as c_int & 0x7 as c_int;
            (*mm).crcok = (0 as c_int as c_uint == (*mm).crc) as c_int;
            if (*mm).crcok != 0 {
                // DF 18 : if crc == 0 try to populate our ICAO addresses whitelist.
                addRecentlySeenICAOAddrImpl(Modes, (*mm).addr);
            }
        }
        _ => {
            // Compare the checksum with the whitelist of recently seen ICAO
            // addresses. If it matches one, then declare the message as valid
            (*mm).addr = (*mm).crc;
            (*mm).crcok = ICAOAddressWasRecentlySeenImpl(Modes, (*mm).addr)
        }
    }

    // If we're checking CRC and the CRC is invalid, then we can't trust any
    // of the data contents, so save time and give up now.
    if (*Modes).check_crc != 0 && (*mm).crcok == 0 && (*mm).correctedbits == 0 {
        return;
    }

    // Fields for DF0, DF16
    if (*mm).msgtype == 0 as c_int || (*mm).msgtype == 16 as c_int {
        if *msg.offset(0) as c_int & 0x4 as c_int != 0 {
            // VS Bit
            (*mm).bFlags |= (1 as c_int) << 12 as c_int | (1 as c_int) << 9 as c_int
        } else {
            (*mm).bFlags |= (1 as c_int) << 12 as c_int
        }
    }

    // Fields for DF11, DF17
    if (*mm).msgtype == 11 as c_int || (*mm).msgtype == 17 as c_int {
        if (*mm).ca == 4 as c_int {
            (*mm).bFlags |= (1 as c_int) << 12 as c_int | (1 as c_int) << 9 as c_int
        } else if (*mm).ca == 5 as c_int {
            (*mm).bFlags |= (1 as c_int) << 12 as c_int
        }
    }

    // Fields for DF5, DF21 = Gillham encoded Squawk
    if (*mm).msgtype == 5 as c_int || (*mm).msgtype == 21 as c_int {
        let ID13Field: c_int =
            ((*msg.offset(2) as c_int) << 8 as c_int | *msg.offset(3) as c_int) & 0x1fff as c_int;
        if ID13Field != 0 {
            (*mm).bFlags |= (1 as c_int) << 5 as c_int;
            (*mm).modeA = decodeID13Field(ID13Field)
        }
    }

    // Fields for DF0, DF4, DF16, DF20 13 bit altitude
    if (*mm).msgtype == 0 as c_int
        || (*mm).msgtype == 4 as c_int
        || (*mm).msgtype == 16 as c_int
        || (*mm).msgtype == 20 as c_int
    {
        let AC13Field: c_int =
            ((*msg.offset(2) as c_int) << 8 as c_int | *msg.offset(3) as c_int) & 0x1fff as c_int;
        if AC13Field != 0 {
            // Only attempt to decode if a valid (non zero) altitude is present
            (*mm).bFlags |= (1 as c_int) << 1 as c_int;
            (*mm).altitude = decodeAC13Field(AC13Field, &mut (*mm).unit)
        }
    }

    // Fields for DF4, DF5, DF20, DF21
    if (*mm).msgtype == 4 as c_int
        || (*mm).msgtype == 20 as c_int
        || (*mm).msgtype == 5 as c_int
        || (*mm).msgtype == 21 as c_int
    {
        (*mm).bFlags |= (1 as c_int) << 13 as c_int; // Flight status for DF4,5,20,21
        (*mm).fs = *msg.offset(0) as c_int & 7 as c_int;
        if (*mm).fs <= 3 as c_int {
            (*mm).bFlags |= (1 as c_int) << 12 as c_int;
            if (*mm).fs & 1 as c_int != 0 {
                (*mm).bFlags |= (1 as c_int) << 9 as c_int
            }
        }
    }

    // Fields for DF17, DF18_CF0, DF18_CF1, DF18_CF6 squitters
    if (*mm).msgtype == 17 as c_int
        || (*mm).msgtype == 18 as c_int
            && ((*mm).ca == 0 as c_int || (*mm).ca == 1 as c_int || (*mm).ca == 6 as c_int)
    {
        (*mm).metype = *msg.offset(4) as c_int >> 3 as c_int; // Extended squitter message type
        let metype: c_int = (*mm).metype; // Extended squitter message subtype
        (*mm).mesub = if metype == 29 as c_int {
            (*msg.offset(4) as c_int & 6 as c_int) >> 1 as c_int
        } else {
            (*msg.offset(4) as c_int) & 7 as c_int
        };
        let mesub: c_int = (*mm).mesub;
        // Decode the extended squitter message
        if metype >= 1 as c_int && metype <= 4 as c_int {
            // Aircraft Identification and Category
            let mut chars: u32;
            (*mm).bFlags |= (1 as c_int) << 6 as c_int;
            chars = ((*msg.offset(5) as c_int) << 16 as c_int
                | (*msg.offset(6) as c_int) << 8 as c_int
                | *msg.offset(7) as c_int) as u32;
            (*mm).flight[3 as c_int as usize] =
                *ais_charset.offset((chars & 0x3f as c_int as c_uint) as isize);
            chars = chars >> 6 as c_int;
            (*mm).flight[2 as c_int as usize] =
                *ais_charset.offset((chars & 0x3f as c_int as c_uint) as isize);
            chars = chars >> 6 as c_int;
            (*mm).flight[1 as c_int as usize] =
                *ais_charset.offset((chars & 0x3f as c_int as c_uint) as isize);
            chars = chars >> 6 as c_int;
            (*mm).flight[0 as c_int as usize] =
                *ais_charset.offset((chars & 0x3f as c_int as c_uint) as isize);
            chars = ((*msg.offset(8) as c_int) << 16 as c_int
                | (*msg.offset(9) as c_int) << 8 as c_int
                | *msg.offset(10) as c_int) as u32;
            (*mm).flight[7 as c_int as usize] =
                *ais_charset.offset((chars & 0x3f as c_int as c_uint) as isize);
            chars = chars >> 6 as c_int;
            (*mm).flight[6 as c_int as usize] =
                *ais_charset.offset((chars & 0x3f as c_int as c_uint) as isize);
            chars = chars >> 6 as c_int;
            (*mm).flight[5 as c_int as usize] =
                *ais_charset.offset((chars & 0x3f as c_int as c_uint) as isize);
            chars = chars >> 6 as c_int;
            (*mm).flight[4 as c_int as usize] =
                *ais_charset.offset((chars & 0x3f as c_int as c_uint) as isize);
            (*mm).flight[8 as c_int as usize] = '\u{0}' as i32 as c_char
        } else if metype == 19 as c_int {
            // Airborne Velocity Message
            // Presumably airborne if we get an Airborne Velocity Message
            (*mm).bFlags |= (1 as c_int) << 12 as c_int;
            if mesub >= 1 as c_int && mesub <= 4 as c_int {
                let mut vert_rate: c_int = (*msg.offset(8) as c_int & 0x7 as c_int) << 6 as c_int
                    | *msg.offset(9) as c_int >> 2 as c_int;
                if vert_rate != 0 {
                    vert_rate -= 1;
                    if *msg.offset(8) as c_int & 0x8 as c_int != 0 {
                        vert_rate = 0 as c_int - vert_rate
                    }
                    (*mm).vert_rate = vert_rate * 64 as c_int;
                    (*mm).bFlags |= (1 as c_int) << 4 as c_int
                }
            }
            if mesub == 1 as c_int || mesub == 2 as c_int {
                let ew_raw: c_int = (*msg.offset(5) as c_int & 0x3 as c_int) << 8 as c_int
                    | *msg.offset(6) as c_int;
                let mut ew_vel: c_int = ew_raw - 1 as c_int;
                let ns_raw: c_int = (*msg.offset(7) as c_int & 0x7f as c_int) << 3 as c_int
                    | *msg.offset(8) as c_int >> 5 as c_int;
                let mut ns_vel: c_int = ns_raw - 1 as c_int;
                if mesub == 2 as c_int {
                    // If (supersonic) unit is 4 kts
                    ns_vel = ns_vel << 2 as c_int;
                    ew_vel = ew_vel << 2 as c_int
                }
                if ew_raw != 0 {
                    // Do East/West
                    (*mm).bFlags |= (1 as c_int) << 7 as c_int;
                    if *msg.offset(5) as c_int & 0x4 as c_int != 0 {
                        ew_vel = 0 as c_int - ew_vel
                    }
                    (*mm).ew_velocity = ew_vel
                }
                if ns_raw != 0 {
                    // Do North/South
                    (*mm).bFlags |= (1 as c_int) << 8 as c_int;
                    if *msg.offset(7) as c_int & 0x80 as c_int != 0 {
                        ns_vel = 0 as c_int - ns_vel
                    }
                    (*mm).ns_velocity = ns_vel
                }
                if ew_raw != 0 && ns_raw != 0 {
                    // Compute velocity and angle from the two speed components
                    (*mm).bFlags |= (1 as c_int) << 3 as c_int
                        | (1 as c_int) << 2 as c_int
                        | (1 as c_int) << 14 as c_int;
                    (*mm).velocity = ((ns_vel * ns_vel + ew_vel * ew_vel) as f64).sqrt() as c_int;
                    if (*mm).velocity != 0 {
                        (*mm).heading = ((ew_vel as f64).atan2(ns_vel as f64) * 180.0f64
                            / 3.14159265358979323846f64)
                            as c_int;
                        // We don't want negative values but a 0-360 scale
                        if (*mm).heading < 0 as c_int {
                            (*mm).heading += 360 as c_int
                        }
                    }
                }
            } else if mesub == 3 as c_int || mesub == 4 as c_int {
                let mut airspeed: c_int = (*msg.offset(7) as c_int & 0x7f as c_int) << 3 as c_int
                    | *msg.offset(8) as c_int >> 5 as c_int;
                if airspeed != 0 {
                    (*mm).bFlags |= (1 as c_int) << 3 as c_int;
                    airspeed -= 1;
                    if mesub == 4 as c_int {
                        // If (supersonic) unit is 4 kts
                        airspeed = airspeed << 2 as c_int
                    }
                    (*mm).velocity = airspeed
                }
                if *msg.offset(5) as c_int & 0x4 as c_int != 0 {
                    (*mm).bFlags |= (1 as c_int) << 2 as c_int;
                    (*mm).heading = ((*msg.offset(5) as c_int & 0x3 as c_int) << 8 as c_int
                        | *msg.offset(6) as c_int)
                        * 45 as c_int
                        >> 7 as c_int
                }
            }
        } else if metype >= 5 as c_int && metype <= 22 as c_int {
            // Position Message
            (*mm).raw_latitude = (*msg.offset(6) as c_int & 3 as c_int) << 15 as c_int
                | (*msg.offset(7) as c_int) << 7 as c_int
                | *msg.offset(8) as c_int >> 1 as c_int; // Ground
            (*mm).raw_longitude = (*msg.offset(8) as c_int & 1 as c_int) << 16 as c_int
                | (*msg.offset(9) as c_int) << 8 as c_int
                | *msg.offset(10) as c_int;
            (*mm).bFlags |= if (*mm).msg[6 as c_int as usize] as c_int & 0x4 as c_int != 0 {
                (1 as c_int) << 11 as c_int
            } else {
                (1 as c_int) << 10 as c_int
            };
            if metype >= 9 as c_int {
                let AC12Field: c_int = ((*msg.offset(5) as c_int) << 4 as c_int
                    | *msg.offset(6) as c_int >> 4 as c_int)
                    & 0xfff as c_int;
                (*mm).bFlags |= (1 as c_int) << 12 as c_int;
                if AC12Field != 0 {
                    // Airborne
                    // Only attempt to decode if a valid (non zero) altitude is present
                    (*mm).bFlags |= (1 as c_int) << 1 as c_int;
                    (*mm).altitude = decodeAC12Field(AC12Field, &mut (*mm).unit)
                }
            } else {
                let movement: c_int = ((*msg.offset(4) as c_int) << 4 as c_int
                    | *msg.offset(5) as c_int >> 4 as c_int)
                    & 0x7f as c_int;
                (*mm).bFlags |= (1 as c_int) << 12 as c_int | (1 as c_int) << 9 as c_int;
                if movement != 0 && movement < 125 as c_int {
                    (*mm).bFlags |= (1 as c_int) << 3 as c_int;
                    (*mm).velocity = decodeMovementField(movement)
                }
                if *msg.offset(5) as c_int & 0x8 as c_int != 0 {
                    (*mm).bFlags |= (1 as c_int) << 2 as c_int;
                    (*mm).heading = (((*msg.offset(5) as c_int) << 4 as c_int
                        | *msg.offset(6) as c_int >> 4 as c_int)
                        & 0x7f as c_int)
                        * 45 as c_int
                        >> 4 as c_int
                }
            }
        } else if metype == 23 as c_int {
            // Test metype squawk field
            if mesub == 7 as c_int {
                // (see 1090-WP-15-20)
                let ID13Field_0: c_int = (((*msg.offset(5) as c_int) << 8 as c_int
                    | *msg.offset(6) as c_int)
                    & 0xfff1 as c_int)
                    >> 3 as c_int;
                if ID13Field_0 != 0 {
                    (*mm).bFlags |= (1 as c_int) << 5 as c_int;
                    (*mm).modeA = decodeID13Field(ID13Field_0)
                }
            }
        } else if !(metype == 24 as c_int) {
            if metype == 28 as c_int {
                // Extended Squitter Aircraft Status
                if mesub == 1 as c_int {
                    // Emergency status squawk field
                    let ID13Field_1: c_int = ((*msg.offset(5) as c_int) << 8 as c_int
                        | *msg.offset(6) as c_int)
                        & 0x1fff as c_int;
                    if ID13Field_1 != 0 {
                        (*mm).bFlags |= (1 as c_int) << 5 as c_int;
                        (*mm).modeA = decodeID13Field(ID13Field_1)
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
    if (*mm).msgtype == 20 as c_int || (*mm).msgtype == 21 as c_int {
        if *msg.offset(4) as c_int == 0x20 as c_int {
            // Aircraft Identification
            let mut chars_0: u32;
            (*mm).bFlags |= (1 as c_int) << 6 as c_int;
            chars_0 = ((*msg.offset(5) as c_int) << 16 as c_int
                | (*msg.offset(6) as c_int) << 8 as c_int
                | *msg.offset(7) as c_int) as u32;
            (*mm).flight[3 as c_int as usize] =
                *ais_charset.offset((chars_0 & 0x3f as c_int as c_uint) as isize);
            chars_0 = chars_0 >> 6 as c_int;
            (*mm).flight[2 as c_int as usize] =
                *ais_charset.offset((chars_0 & 0x3f as c_int as c_uint) as isize);
            chars_0 = chars_0 >> 6 as c_int;
            (*mm).flight[1 as c_int as usize] =
                *ais_charset.offset((chars_0 & 0x3f as c_int as c_uint) as isize);
            chars_0 = chars_0 >> 6 as c_int;
            (*mm).flight[0 as c_int as usize] =
                *ais_charset.offset((chars_0 & 0x3f as c_int as c_uint) as isize);
            chars_0 = ((*msg.offset(8) as c_int) << 16 as c_int
                | (*msg.offset(9) as c_int) << 8 as c_int
                | *msg.offset(10) as c_int) as u32;
            (*mm).flight[7 as c_int as usize] =
                *ais_charset.offset((chars_0 & 0x3f as c_int as c_uint) as isize);
            chars_0 = chars_0 >> 6 as c_int;
            (*mm).flight[6 as c_int as usize] =
                *ais_charset.offset((chars_0 & 0x3f as c_int as c_uint) as isize);
            chars_0 = chars_0 >> 6 as c_int;
            (*mm).flight[5 as c_int as usize] =
                *ais_charset.offset((chars_0 & 0x3f as c_int as c_uint) as isize);
            chars_0 = chars_0 >> 6 as c_int;
            (*mm).flight[4 as c_int as usize] =
                *ais_charset.offset((chars_0 & 0x3f as c_int as c_uint) as isize);
            (*mm).flight[8 as c_int as usize] = '\u{0}' as i32 as c_char
        }
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    const NERRORINFO: usize =
        (MODES_LONG_MSG_BITS + MODES_LONG_MSG_BITS * (MODES_LONG_MSG_BITS - 1) / 2) as usize;

    #[test]
    fn test_bit_error_table() {
        let mut bit_error_table = [errorinfo {
            syndrome: 0,
            bits: 0,
            pos: [0; 2],
        }; NERRORINFO];
        let nfix_crc_agressive = 2; // TODO: test with 1 and 2
        unsafe {
            modesInitErrorInfoImpl(
                bit_error_table.as_mut_ptr(),
                bit_error_table.len() as c_int,
                nfix_crc_agressive,
            )
        };

        // Test code: report if any syndrome appears at least twice. In this
        // case the correction cannot be done without ambiguity.
        // Tried it, does not happen for 1- and 2-bit errors.
        let errorinfo_zero = errorinfo {
            syndrome: 0,
            bits: 0,
            pos: [0; 2],
        };
        for i in 1..bit_error_table.len() {
            // The first 550 are zero in the C impl
            if i <= 549 {
                assert_eq!(bit_error_table[i - 1], errorinfo_zero);
                assert_eq!(bit_error_table[i], errorinfo_zero);
            } else {
                assert_ne!(
                    bit_error_table[i - 1].syndrome,
                    bit_error_table[i].syndrome,
                    "modesInitErrorInfo: Collision for syndrome {:0x}\n",
                    bit_error_table[i].syndrome
                )
            }
        }
    }
    /*
    for (i = 1;  i < NERRORINFO;  i++) {
        if (bitErrorTable[i-1].syndrome == bitErrorTable[i].syndrome) {
            fprintf(stderr, "modesInitErrorInfo: Collision for syndrome %06x\n",
                            (int)bitErrorTable[i].syndrome);
        }
    }

    for (i = 0;  i < NERRORINFO;  i++) {
        printf("syndrome %06x    bit0 %3d    bit1 %3d\n",
               bitErrorTable[i].syndrome,
               bitErrorTable[i].pos0, bitErrorTable[i].pos1);
    }
    */

    #[test]
    fn test_fix_bit_errors() {
        // TODO: Port commented out fixBitErrors test code from mode_s.c
    }
}
