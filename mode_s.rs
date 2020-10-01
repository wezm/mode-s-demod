// ===================== Mode S detection and decoding  ===================

use std::cmp::Ordering;
use std::os::raw::{c_char, c_int, c_uchar, c_uint};
use std::ptr;

const MODES_LONG_MSG_BYTES: c_int = 14;
const MODES_SHORT_MSG_BYTES: c_int = 7;
const MODES_LONG_MSG_BITS: c_int = MODES_LONG_MSG_BYTES * 8;
const MODES_SHORT_MSG_BITS: c_int = MODES_SHORT_MSG_BYTES * 8;

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
    table_ptr: *mut errorinfo,
    table_len: c_int,
) -> c_int {
    let bitErrorTable = &mut *ptr::slice_from_raw_parts_mut(table_ptr, table_len as usize);
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
