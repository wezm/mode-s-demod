// ===================== Mode S detection and decoding  ===================

use std::cmp::Ordering;
use std::convert::TryFrom;
use std::os::raw::{c_char, c_int, c_uchar, c_uint, c_ulong, c_void};
use std::time::SystemTime;
use std::{mem, ptr, time};

use crate::io::modesQueueOutput;
use crate::mode_ac::{ModeAToModeC, MODEAC_MSG_SAMPLES};
use crate::{aircraft, modes, modesMessage};

extern "C" {
    #[no_mangle]
    fn modesSendAllClients(service: c_int, msg: *mut c_void, len: c_int); // noport

    #[no_mangle]
    fn detectModeA(m: *mut u16, mm: *mut modesMessage) -> c_int;

    #[no_mangle]
    fn decodeModeAMessage(mm: *mut modesMessage, ModeA: c_int);

    #[no_mangle]
    fn dumpRawMessage(descr: *const c_char, msg: *mut c_uchar, m: *mut u16, offset: u32);

    #[no_mangle]
    fn interactiveReceiveData(mm: *mut modesMessage) -> *mut aircraft;

    #[no_mangle]
    fn displayModesMessage(mm: *mut modesMessage);
}

// const MODES_DEFAULT_PPM: c_int = 52;
// const MODES_DEFAULT_RATE: c_int = 2000000;
// const MODES_DEFAULT_FREQ: c_int = 1090000000;
// const MODES_DEFAULT_WIDTH: c_int = 1000;
// const MODES_DEFAULT_HEIGHT: c_int = 700;
// const MODES_ASYNC_BUF_NUMBER: usize = 16;
const MODES_ASYNC_BUF_SIZE: usize = 16 * 16384; // 256k
const MODES_ASYNC_BUF_SAMPLES: usize = MODES_ASYNC_BUF_SIZE / 2; // Each sample is 2 bytes
                                                                 // const MODES_AUTO_GAIN: c_int = -100; // Use automatic gain
                                                                 // const MODES_MAX_GAIN: c_int = 999999; // Use max available gain
const MODES_MSG_SQUELCH_LEVEL: c_int = 0x02FF; // Average signal strength limit
const MODES_MSG_ENCODER_ERRS: c_int = 3; // Maximum number of encoding errors

// When changing, change also fixBitErrors() and modesInitErrorTable() !!
const MODES_MAX_BITERRORS: c_int = 2; // Global max for fixable bit erros

const MODES_PREAMBLE_US: usize = 8; // microseconds = bits
const MODES_PREAMBLE_SAMPLES: usize = MODES_PREAMBLE_US * 2;
const MODES_PREAMBLE_SIZE: usize = MODES_PREAMBLE_SAMPLES * mem::size_of::<u16>();
const MODES_LONG_MSG_BYTES: usize = 14;
const MODES_SHORT_MSG_BYTES: usize = 7;
const MODES_LONG_MSG_BITS: c_int = MODES_LONG_MSG_BYTES as c_int * 8;
const MODES_SHORT_MSG_BITS: c_int = MODES_SHORT_MSG_BYTES as c_int * 8;
const MODES_LONG_MSG_SAMPLES: usize = MODES_LONG_MSG_BITS as usize * 2;
// const MODES_SHORT_MSG_SAMPLES: usize = MODES_SHORT_MSG_BITS as usize * 2;
const MODES_LONG_MSG_SIZE: usize = MODES_LONG_MSG_SAMPLES * mem::size_of::<u16>();
// const MODES_SHORT_MSG_SIZE: usize = MODES_SHORT_MSG_SAMPLES * mem::size_of::<u16>();

// const MODES_RAWOUT_BUF_SIZE: c_int = 1500;
// const MODES_RAWOUT_BUF_FLUSH: c_int = MODES_RAWOUT_BUF_SIZE - 200;
// const MODES_RAWOUT_BUF_RATE: c_int = 1000; // 1000 * 64mS = 1 Min approx

const MODES_ICAO_CACHE_LEN: u32 = 1024; // Value must be a power of two
const MODES_ICAO_CACHE_TTL: u64 = 60; // Time to live of cached addresses
const MODES_UNIT_FEET: c_int = 0;
const MODES_UNIT_METERS: c_int = 1;

const MODES_DEBUG_DEMOD: c_int = 1 << 0;
const MODES_DEBUG_DEMODERR: c_int = 1 << 1;
const MODES_DEBUG_BADCRC: c_int = 1 << 2;
const MODES_DEBUG_GOODCRC: c_int = 1 << 3;
const MODES_DEBUG_NOPREAMBLE: c_int = 1 << 4;
// const MODES_DEBUG_NET: c_int = 1<<5;
// const MODES_DEBUG_JS: c_int = 1<<6;

// When debug is set to MODES_DEBUG_NOPREAMBLE, the first sample must be
// at least greater than a given level for us to dump the signal.
const MODES_DEBUG_NOPREAMBLE_LEVEL: c_int = 25;

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
            (*mm).iid = (*mm).crc as c_int; // Responder capabilities, zero indicates no communications capability (surveillance only)
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
                addRecentlySeenICAOAddrImpl(Modes, (*mm).addr);
            }
        }
        18 => {
            // DF 18
            // This is currently identical to DF 17 as the ca field is being reused to store the
            // Control Field.
            (*mm).addr = ((*msg.offset(1) as c_int) << 16 as c_int
                | (*msg.offset(2) as c_int) << 8 as c_int
                | *msg.offset(3) as c_int) as u32; // Control Field, should always be 0 for ADS-B
            (*mm).ca = *msg.offset(0) as c_int & 0x7 as c_int;
            (*mm).crcok = (0 as c_int as c_uint == (*mm).crc) as c_int;
            if (*mm).crcok != 0 {
                // DF 18 : if crc == 0 try to populate our ICAO addresses whitelist.
                addRecentlySeenICAOAddrImpl(Modes, (*mm).addr);
            }
        }
        _ => {
            // All other DF's
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

// Turn I/Q samples pointed by Modes.data into the magnitude vector
// pointed by Modes.magnitude.
//
#[no_mangle]
pub unsafe extern "C" fn computeMagnitudeVectorImpl(mut p: *mut u16, Modes: *mut modes) {
    let mut m: *mut u16 = (*Modes)
        .magnitude
        .offset((MODES_PREAMBLE_SAMPLES + MODES_LONG_MSG_SAMPLES) as isize)
        as *mut u16;

    ptr::copy_nonoverlapping(
        (*Modes).magnitude.offset(MODES_ASYNC_BUF_SAMPLES as isize),
        (*Modes).magnitude,
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
        *fresh4 = *(*Modes).maglut.offset(*fresh3 as isize);
        j = j.wrapping_add(1)
    }
}

// Return -1 if the message is out of phase left-side
// Return  1 if the message is out of phase right-size
// Return  0 if the message is not particularly out of phase.
//
// Note: this function will access pPreamble[-1], so the caller should make sure to
// call it only if we are not at the start of the current buffer
//
#[no_mangle]
pub unsafe extern "C" fn detectOutOfPhase(pPreamble: *const u16) -> c_int {
    if *pPreamble.offset(3) > *pPreamble.offset(2) / 3 {
        return 1;
    }
    if *pPreamble.offset(10) > *pPreamble.offset(9) / 3 {
        return 1;
    }
    if *pPreamble.offset(6) > *pPreamble.offset(7) / 3 {
        return -1;
    }
    if *pPreamble.offset(-1) > *pPreamble.offset(1) / 3 {
        return -1;
    }

    0
}

#[no_mangle]
pub extern "C" fn clamped_scale(v: u16, scale: u16) -> u16 {
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
// pPayload[0] should be the start of the preamble,
// pPayload[-1 .. MODES_PREAMBLE_SAMPLES + MODES_LONG_MSG_SAMPLES - 1] should be accessible.
// pPayload[MODES_PREAMBLE_SAMPLES .. MODES_PREAMBLE_SAMPLES + MODES_LONG_MSG_SAMPLES - 1] will be updated.
#[no_mangle]
pub unsafe extern "C" fn applyPhaseCorrection(pPayload: *mut u16) {
    // we expect 1 bits at 0, 2, 7, 9
    // and 0 bits at -1, 1, 3, 4, 5, 6, 8, 10, 11, 12, 13, 14
    // use bits -1,6 for early detection (bit 0/7 arrived a little early, our sample period starts after the bit phase so we include some of the next bit)
    // use bits 3,10 for late detection (bit 2/9 arrived a little late, our sample period starts before the bit phase so we include some of the last bit)

    let onTime: u32 = (*pPayload.offset(0) as c_int
        + *pPayload.offset(2) as c_int
        + *pPayload.offset(7) as c_int
        + *pPayload.offset(9) as c_int) as u32;
    let early: u32 = ((*pPayload.offset(-(1 as c_int) as isize) as c_int
        + *pPayload.offset(6) as c_int)
        << 1 as c_int) as u32;
    let late: u32 =
        ((*pPayload.offset(3) as c_int + *pPayload.offset(10) as c_int) << 1 as c_int) as u32;

    if early > late {
        // Our sample period starts late and so includes some of the next bit.
        let scaleUp: u16 = (16384 as c_int as c_uint).wrapping_add(
            (16384 as c_int as c_uint)
                .wrapping_mul(early)
                .wrapping_div(early.wrapping_add(onTime)),
        ) as u16; // 1 + early / (early+onTime)
        let scaleDown: u16 = (16384 as c_int as c_uint).wrapping_sub(
            (16384 as c_int as c_uint)
                .wrapping_mul(early)
                .wrapping_div(early.wrapping_add(onTime)),
        ) as u16; // 1 - early / (early+onTime)

        // trailing bits are 0; final data sample will be a bit low.
        *pPayload.offset(
            (8 as c_int * 2 as c_int + 14 as c_int * 8 as c_int * 2 as c_int - 1 as c_int) as isize,
        ) = clamped_scale(
            *pPayload.offset(
                (8 as c_int * 2 as c_int + 14 as c_int * 8 as c_int * 2 as c_int - 1 as c_int)
                    as isize,
            ),
            scaleUp,
        );

        let mut j = MODES_PREAMBLE_SAMPLES + MODES_LONG_MSG_SAMPLES - 2;
        while j > MODES_PREAMBLE_SAMPLES {
            if *pPayload.offset(j as isize) as c_int > *pPayload.offset((j + 1) as isize) as c_int {
                // x [1 0] y
                // x overlapped with the "1" bit and is slightly high
                *pPayload.offset((j - 1) as isize) =
                    clamped_scale(*pPayload.offset(j as isize - 1), scaleDown)
            } else {
                // x [0 1] y
                // x overlapped with the "0" bit and is slightly low
                *pPayload.offset((j - 1) as isize) =
                    clamped_scale(*pPayload.offset(j as isize - 1), scaleUp)
            }
            j -= 2
        }
    } else {
        // Our sample period starts early and so includes some of the previous bit.
        let scaleUp_0: u16 = (16384 as c_int as c_uint).wrapping_add(
            (16384 as c_int as c_uint)
                .wrapping_mul(late)
                .wrapping_div(late.wrapping_add(onTime)),
        ) as u16; // 1 + late / (late+onTime)
        let scaleDown_0: u16 = (16384 as c_int as c_uint).wrapping_sub(
            (16384 as c_int as c_uint)
                .wrapping_mul(late)
                .wrapping_div(late.wrapping_add(onTime)),
        ) as u16; // 1 - late / (late+onTime)

        // leading bits are 0; first data sample will be a bit low.
        *pPayload.offset(MODES_PREAMBLE_SAMPLES as isize) =
            clamped_scale(*pPayload.offset(MODES_PREAMBLE_SAMPLES as isize), scaleUp_0);
        let mut j = MODES_PREAMBLE_SAMPLES;
        while j < MODES_PREAMBLE_SAMPLES + MODES_LONG_MSG_SAMPLES - 2 {
            if *pPayload.offset(j as isize) as c_int > *pPayload.offset(j as isize + 1) as c_int {
                // x [1 0] y
                // y overlapped with the "0" bit and is slightly low
                *pPayload.offset(j as isize + 2) =
                    clamped_scale(*pPayload.offset(j as isize + 2), scaleUp_0)
            } else {
                // x [0 1] y
                // y overlapped with the "1" bit and is slightly high
                *pPayload.offset(j as isize + 2) =
                    clamped_scale(*pPayload.offset(j as isize + 2), scaleDown_0)
            }
            j += 2
        }
    };
}

// Detect a Mode S messages inside the magnitude buffer pointed by 'm' and of
// size 'mlen' bytes. Every detected Mode S message is convert it into a
// stream of bits and passed to the function to display it.
//
#[no_mangle]
pub unsafe extern "C" fn detectModeSImpl(
    m: *mut u16,
    mlen: u32,
    Modes: *mut modes,
    bit_errors_ptr: *const errorinfo,
    bit_errors_len: c_int,
) {
    let mut mm: modesMessage = modesMessage::default();
    let mut msg = [0; MODES_LONG_MSG_BYTES];
    let mut pMsg: *mut c_uchar;
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
        let mut errorsTy: c_int;
        let mut pPtr: *mut u16;
        let mut theByte: u8;
        let mut theErrs: u8;
        let mut msglen: c_int;
        let mut scanlen: c_int;
        let mut sigStrength: c_int;

        let pPreamble = m.offset(j as isize);
        let mut pPayload = m.offset(j as isize + MODES_PREAMBLE_SAMPLES as isize);

        // Rather than clear the whole mm structure, just clear the parts which are required. The clear
        // is required for every bit of the input stream, and we don't want to be memset-ing the whole
        // modesMessage structure two million times per second if we don't have to..
        mm.correctedbits = 0;
        mm.crcok = mm.correctedbits;
        mm.bFlags = mm.crcok;

        if use_correction == 0 {
            // This is not a re-try with phase correction
            // so try to find a new preamble
            if (*Modes).mode_ac != 0 {
                let ModeA = detectModeA(pPreamble, &mut mm);
                if ModeA != 0 {
                    // We have found a valid ModeA/C in the data
                    mm.timestampMsg = (*Modes)
                        .timestampBlk
                        .wrapping_add(j.wrapping_add(1).wrapping_mul(6) as c_ulong);

                    // Decode the received message
                    decodeModeAMessage(&mut mm, ModeA);

                    // Pass data to the next layer
                    useModesMessage(Modes, &mut mm);
                    j += MODEAC_MSG_SAMPLES;
                    (*Modes).stat_ModeAC = (*Modes).stat_ModeAC.wrapping_add(1);
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
                    if !(*pPreamble.offset(0) > *pPreamble.offset(1)
                        && (*pPreamble.offset(1)) < *pPreamble.offset(2)
                        && *pPreamble.offset(2) > *pPreamble.offset(3)
                        && (*pPreamble.offset(3)) < *pPreamble.offset(0)
                        && (*pPreamble.offset(4)) < *pPreamble.offset(0)
                        && (*pPreamble.offset(5)) < *pPreamble.offset(0)
                        && (*pPreamble.offset(6)) < *pPreamble.offset(0)
                        && *pPreamble.offset(7) > *pPreamble.offset(8)
                        && (*pPreamble.offset(8)) < *pPreamble.offset(9)
                        && *pPreamble.offset(9) > *pPreamble.offset(6))
                    {
                        if (*Modes).debug & MODES_DEBUG_NOPREAMBLE != 0
                            && *pPreamble as c_int > MODES_DEBUG_NOPREAMBLE_LEVEL
                        {
                            dumpRawMessage(
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
                        high = (*pPreamble.offset(0) as c_int
                            + *pPreamble.offset(2) as c_int
                            + *pPreamble.offset(7) as c_int
                            + *pPreamble.offset(9) as c_int)
                            / 6 as c_int;
                        if *pPreamble.offset(4) as c_int >= high
                            || *pPreamble.offset(5) as c_int >= high
                        {
                            if (*Modes).debug & MODES_DEBUG_NOPREAMBLE != 0
                                && *pPreamble as c_int > MODES_DEBUG_NOPREAMBLE_LEVEL
                            {
                                dumpRawMessage(
                                    b"Too high level in samples between 3 and 6\x00".as_ptr()
                                        as *const c_char,
                                    msg.as_mut_ptr(),
                                    m,
                                    j,
                                );
                            }
                            current_block_183 = 735147466149431745;
                        } else if *pPreamble.offset(11) as c_int >= high
                            || *pPreamble.offset(12) as c_int >= high
                            || *pPreamble.offset(13) as c_int >= high
                            || *pPreamble.offset(14) as c_int >= high
                        {
                            // Similarly samples in the range 11-14 must be low, as it is the
                            // space between the preamble and real data. Again we don't test
                            // bits too near to high levels, see above
                            if (*Modes).debug & MODES_DEBUG_NOPREAMBLE != 0
                                && *pPreamble as c_int > MODES_DEBUG_NOPREAMBLE_LEVEL
                            {
                                dumpRawMessage(
                                    b"Too high level in samples between 10 and 15\x00".as_ptr()
                                        as *const c_char,
                                    msg.as_mut_ptr(),
                                    m,
                                    j,
                                );
                            }
                            current_block_183 = 735147466149431745;
                        } else {
                            (*Modes).stat_valid_preamble =
                                (*Modes).stat_valid_preamble.wrapping_add(1);
                            current_block_183 = 6450636197030046351;
                        }
                    }
                }
            }
        } else {
            // If the previous attempt with this message failed, retry using
            // magnitude correction
            // Make a copy of the Payload, and phase correct the copy
            ptr::copy_nonoverlapping(pPreamble.offset(-1), aux.as_mut_ptr(), aux.len());
            applyPhaseCorrection(&mut *aux.as_mut_ptr().offset(1));
            (*Modes).stat_out_of_phase = (*Modes).stat_out_of_phase.wrapping_add(1);
            pPayload = aux.as_mut_ptr().offset(1 + MODES_PREAMBLE_SAMPLES as isize) as *mut u16;
            current_block_183 = 6450636197030046351;
            // TODO ... apply other kind of corrections
        }

        match current_block_183 {
            6450636197030046351 => {
                // Decode all the next 112 bits, regardless of the actual message
                // size. We'll check the actual message type later
                pMsg = &mut *msg.as_mut_ptr().offset(0) as *mut c_uchar;
                pPtr = pPayload;
                theByte = 0;
                theErrs = 0;
                errorsTy = 0;
                errors = 0;
                errors56 = 0;

                // We should have 4 'bits' of 0/1 and 1/0 samples in the preamble,
                // so include these in the signal strength
                sigStrength = *pPreamble.offset(0) as c_int - *pPreamble.offset(1) as c_int
                    + (*pPreamble.offset(2) as c_int - *pPreamble.offset(3) as c_int)
                    + (*pPreamble.offset(7) as c_int - *pPreamble.offset(6) as c_int)
                    + (*pPreamble.offset(9) as c_int - *pPreamble.offset(8) as c_int);

                scanlen = MODES_LONG_MSG_BITS;
                msglen = scanlen;
                i = 0 as c_int;
                while i < scanlen {
                    let fresh5 = pPtr;
                    pPtr = pPtr.offset(1);
                    let a = *fresh5 as u32;
                    let fresh6 = pPtr;
                    pPtr = pPtr.offset(1);
                    let b = *fresh6 as u32;
                    if a > b {
                        theByte |= 1;
                        if i < 56 {
                            sigStrength =
                                (sigStrength as c_uint).wrapping_add(a.wrapping_sub(b)) as c_int
                        }
                    } else if a < b {
                        /*theByte |= 0;*/
                        if i < 56 {
                            sigStrength =
                                (sigStrength as c_uint).wrapping_add(b.wrapping_sub(a)) as c_int
                        }
                    } else if i >= MODES_SHORT_MSG_BITS {
                        //(a == b), and we're in the long part of a frame
                        errors += 1
                    /*theByte |= 0;*/
                    } else if i >= 5 {
                        //(a == b), and we're in the short part of a frame
                        scanlen = MODES_LONG_MSG_BITS;
                        errors += 1;
                        errors56 = errors
                    /*theByte |= 0;*/
                    } else if i != 0 {
                        //(a == b), and we're in the message type part of a frame
                        errors += 1;
                        errors56 = errors;
                        errorsTy = errors56;
                        theErrs |= 1;
                    /*theByte |= 0;*/
                    } else {
                        //(a == b), and we're in the first bit of the message type part of a frame
                        errors += 1;
                        errors56 = errors;
                        errorsTy = errors56;
                        theErrs |= 1;
                        theByte |= 1;
                    }

                    if i & 7 == 7 {
                        let fresh7 = pMsg;
                        pMsg = pMsg.offset(1);
                        *fresh7 = theByte
                    } else if i == 4 as c_int {
                        msglen = modesMessageLenByType(theByte as c_int);
                        if errors == 0 as c_int {
                            scanlen = msglen
                        }
                    }

                    theByte = ((theByte as c_int) << 1 as c_int) as u8;
                    if i < 7 {
                        theErrs = theErrs << 1;
                    }

                    // If we've exceeded the permissible number of encoding errors, abandon ship now
                    if errors > MODES_MSG_ENCODER_ERRS {
                        if i < MODES_SHORT_MSG_BITS {
                            msglen = 0;
                        } else if errorsTy == 1 && theErrs == 0x80 {
                            // If we only saw one error in the first bit of the byte of the frame, then it's possible
                            // we guessed wrongly about the value of the bit. We may be able to correct it by guessing
                            // the other way.
                            //
                            // We guessed a '1' at bit 7, which is the DF length bit == 112 Bits.
                            // Inverting bit 7 will change the message type from a long to a short.
                            // Invert the bit, cross your fingers and carry on.
                            msglen = MODES_SHORT_MSG_BITS; // revert to the number of errors prior to bit 56
                            msg[0] = (msg[0] as c_int ^ theErrs as c_int) as c_uchar;
                            errorsTy = 0;
                            errors = errors56;
                            (*Modes).stat_DF_Len_Corrected =
                                (*Modes).stat_DF_Len_Corrected.wrapping_add(1);
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
                i = modesMessageLenByType(msg[0] as c_int >> 3 as c_int);
                if msglen > i {
                    msglen = i
                } else if msglen < i {
                    msglen = 0 as c_int
                }

                // If we guessed at any of the bits in the DF type field, then look to see if our guess was sensible.
                // Do this by looking to see if the original guess results in the DF type being one of the ICAO defined
                // message types. If it isn't then toggle the guessed bit and see if this new value is ICAO defined.
                // if the new value is ICAO defined, then update it in our message.
                if msglen != 0 && errorsTy == 1 as c_int && theErrs as c_int & 0x78 as c_int != 0 {
                    // We guessed at one (and only one) of the message type bits. See if our guess is "likely"
                    // to be correct by comparing the DF against a list of known good DF's
                    theByte = msg[0 as c_int as usize]; // One bit per 32 possible DF's. Set bits 0,4,5,11,16.17.18.19,20,21,22,24
                    let mut thisDF = theByte as c_int >> 3 as c_int & 0x1f as c_int;
                    let validDFbits = 0x17f0831 as c_int as u32;
                    let mut thisDFbit = ((1 as c_int) << thisDF) as u32;
                    if 0 as c_int as c_uint == validDFbits & thisDFbit {
                        // The current DF is not ICAO defined, so is probably an errors.
                        // Toggle the bit we guessed at and see if the resultant DF is more likely
                        theByte = (theByte as c_int ^ theErrs as c_int) as u8;
                        thisDF = theByte as c_int >> 3 as c_int & 0x1f as c_int;
                        thisDFbit = ((1 as c_int) << thisDF) as u32;
                        // if this DF any more likely?
                        if validDFbits & thisDFbit != 0 {
                            // Yep, more likely, so update the main message
                            msg[0 as c_int as usize] = theByte;
                            (*Modes).stat_DF_Type_Corrected =
                                (*Modes).stat_DF_Type_Corrected.wrapping_add(1);
                            errors -= 1
                            // decrease the error count so we attempt to use the modified DF.
                        }
                    }
                }

                // We measured signal strength over the first 56 bits. Don't forget to add 4
                // for the preamble samples, so round up and divide by 60.
                sigStrength = (sigStrength + 29) / 60;

                // When we reach this point, if error is small, and the signal strength is large enough
                // we may have a Mode S message on our hands. It may still be broken and the CRC may not
                // be correct, but this can be handled by the next layer.
                if msglen != 0
                    && sigStrength > MODES_MSG_SQUELCH_LEVEL
                    && errors <= MODES_MSG_ENCODER_ERRS
                {
                    // Set initial mm structure details
                    mm.timestampMsg = (*Modes)
                        .timestampBlk
                        .wrapping_add(j.wrapping_mul(6) as c_ulong);
                    sigStrength = sigStrength + 0x7f >> 8;
                    mm.signalLevel = u8::try_from(sigStrength).unwrap_or(std::u8::MAX);
                    mm.phase_corrected = use_correction;

                    // Decode the received message
                    decodeModesMessageImpl(
                        &mut mm,
                        msg.as_mut_ptr(),
                        Modes,
                        bit_errors_ptr,
                        bit_errors_len,
                    );

                    // Update statistics
                    if (*Modes).stats != 0 {
                        if mm.crcok != 0 || use_correction != 0 || mm.correctedbits != 0 {
                            if use_correction != 0 {
                                match errors {
                                    0 => {
                                        (*Modes).stat_ph_demodulated0 =
                                            (*Modes).stat_ph_demodulated0.wrapping_add(1)
                                    }
                                    1 => {
                                        (*Modes).stat_ph_demodulated1 =
                                            (*Modes).stat_ph_demodulated1.wrapping_add(1)
                                    }
                                    2 => {
                                        (*Modes).stat_ph_demodulated2 =
                                            (*Modes).stat_ph_demodulated2.wrapping_add(1)
                                    }
                                    _ => {
                                        (*Modes).stat_ph_demodulated3 =
                                            (*Modes).stat_ph_demodulated3.wrapping_add(1)
                                    }
                                }
                            } else {
                                match errors {
                                    0 => {
                                        (*Modes).stat_demodulated0 =
                                            (*Modes).stat_demodulated0.wrapping_add(1)
                                    }
                                    1 => {
                                        (*Modes).stat_demodulated1 =
                                            (*Modes).stat_demodulated1.wrapping_add(1)
                                    }
                                    2 => {
                                        (*Modes).stat_demodulated2 =
                                            (*Modes).stat_demodulated2.wrapping_add(1)
                                    }
                                    _ => {
                                        (*Modes).stat_demodulated3 =
                                            (*Modes).stat_demodulated3.wrapping_add(1)
                                    }
                                }
                            }

                            if mm.correctedbits == 0 as c_int {
                                if use_correction != 0 {
                                    if mm.crcok != 0 {
                                        (*Modes).stat_ph_goodcrc =
                                            (*Modes).stat_ph_goodcrc.wrapping_add(1)
                                    } else {
                                        (*Modes).stat_ph_badcrc =
                                            (*Modes).stat_ph_badcrc.wrapping_add(1)
                                    }
                                } else if mm.crcok != 0 {
                                    (*Modes).stat_goodcrc = (*Modes).stat_goodcrc.wrapping_add(1)
                                } else {
                                    (*Modes).stat_badcrc = (*Modes).stat_badcrc.wrapping_add(1)
                                }
                            } else if use_correction != 0 {
                                (*Modes).stat_ph_badcrc = (*Modes).stat_ph_badcrc.wrapping_add(1);
                                (*Modes).stat_ph_fixed = (*Modes).stat_ph_fixed.wrapping_add(1);
                                if mm.correctedbits != 0 && mm.correctedbits <= MODES_MAX_BITERRORS
                                {
                                    (*Modes).stat_ph_bit_fix[(mm.correctedbits - 1) as usize] =
                                        (*Modes).stat_ph_bit_fix[(mm.correctedbits - 1) as usize]
                                            .wrapping_add(1)
                                }
                            } else {
                                (*Modes).stat_badcrc = (*Modes).stat_badcrc.wrapping_add(1);
                                (*Modes).stat_fixed = (*Modes).stat_fixed.wrapping_add(1);
                                if mm.correctedbits != 0 && mm.correctedbits <= MODES_MAX_BITERRORS
                                {
                                    (*Modes).stat_bit_fix[(mm.correctedbits - 1) as usize] =
                                        (*Modes).stat_bit_fix[(mm.correctedbits - 1) as usize]
                                            .wrapping_add(1)
                                }
                            }
                        }
                    }

                    // Output debug mode info if needed
                    if use_correction != 0 {
                        if (*Modes).debug & MODES_DEBUG_DEMOD != 0 {
                            dumpRawMessage(
                                b"Demodulated with 0 errors\x00".as_ptr() as *const c_char,
                                msg.as_mut_ptr(),
                                m,
                                j,
                            );
                        } else if (*Modes).debug & MODES_DEBUG_BADCRC != 0
                            && mm.msgtype == 17 as c_int
                            && (mm.crcok == 0 || mm.correctedbits != 0 as c_int)
                        {
                            dumpRawMessage(
                                b"Decoded with bad CRC\x00".as_ptr() as *const c_char,
                                msg.as_mut_ptr(),
                                m,
                                j,
                            );
                        } else if (*Modes).debug & MODES_DEBUG_GOODCRC != 0
                            && mm.crcok != 0
                            && mm.correctedbits == 0 as c_int
                        {
                            dumpRawMessage(
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
                    useModesMessage(Modes, &mut mm);
                } else if (*Modes).debug & MODES_DEBUG_DEMODERR != 0 && use_correction != 0 {
                    println!("The following message has {} demod errors", errors);
                    dumpRawMessage(
                        b"Demodulated with errors\x00".as_ptr() as *const c_char,
                        msg.as_mut_ptr(),
                        m,
                        j,
                    );
                }

                // Retry with phase correction if enabled, necessary and possible.
                if (*Modes).phase_enhance != 0
                    && mm.crcok == 0
                    && mm.correctedbits == 0
                    && use_correction == 0
                    && j != 0
                    && detectOutOfPhase(pPreamble) != 0
                {
                    use_correction = 1;
                    j = j.wrapping_sub(1)
                } else {
                    use_correction = 0;
                }
            }
            _ => {}
        }
        j = j.wrapping_add(1)
    }

    // Send any remaining partial raw buffers now
    if (*Modes).rawOutUsed != 0 || (*Modes).beastOutUsed != 0 {
        (*Modes).net_output_raw_rate_count += 1;
        if (*Modes).net_output_raw_rate_count > (*Modes).net_output_raw_rate {
            if (*Modes).rawOutUsed != 0 {
                modesSendAllClients(
                    (*Modes).ros,
                    (*Modes).rawOut as *mut c_void,
                    (*Modes).rawOutUsed,
                );
                (*Modes).rawOutUsed = 0 as c_int
            }

            if (*Modes).beastOutUsed != 0 {
                modesSendAllClients(
                    (*Modes).bos,
                    (*Modes).beastOut as *mut c_void,
                    (*Modes).beastOutUsed,
                );
                (*Modes).beastOutUsed = 0 as c_int
            }
            (*Modes).net_output_raw_rate_count = 0 as c_int
        }
    } else if (*Modes).net != 0 && (*Modes).net_heartbeat_rate != 0 && {
        (*Modes).net_heartbeat_count += 1;
        ((*Modes).net_heartbeat_count) > (*Modes).net_heartbeat_rate
    } {
        //
        // We haven't received any Mode A/C/S messages for some time. To try and keep any TCP
        // links alive, send a null frame. This will help stop any routers discarding our TCP
        // link which will cause an un-recoverable link error if/when a real frame arrives.
        //
        // Fudge up a null message
        // memset(&mut mm as *mut modesMessage as *mut c_void,
        //        0 as c_int,
        //        ::std::mem::size_of::<modesMessage>() as c_ulong);
        mm = modesMessage::default();
        mm.msgbits = 7 as c_int * 8 as c_int;
        mm.timestampMsg = (*Modes).timestampBlk;

        // Feed output clients
        modesQueueOutput(Modes, &mut mm);

        // Reset the heartbeat counter
        (*Modes).net_heartbeat_count = 0;
    };
}

// When a new message is available, because it was decoded from the RTL device,
// file, or received in the TCP input port, or any other way we can receive a
// decoded message, we call this function in order to use the message.
//
// Basically this function passes a raw message to the upper layers for further
// processing and visualization
//
#[no_mangle]
pub unsafe extern "C" fn useModesMessage(Modes: *mut modes, mm: *mut modesMessage) {
    if (*Modes).check_crc == 0 || (*mm).crcok != 0 || (*mm).correctedbits != 0 {
        // not checking, ok or fixed

        // Always track aircraft
        interactiveReceiveData(mm);

        // In non-interactive non-quiet mode, display messages on standard output
        if (*Modes).interactive == 0 && (*Modes).quiet == 0 {
            displayModesMessage(mm);
        }

        // Feed output clients
        if (*Modes).net != 0 {
            modesQueueOutput(Modes, mm);
        }

        // Heartbeat not required whilst we're seeing real messages
        (*Modes).net_heartbeat_count = 0;
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
