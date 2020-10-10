#![allow(non_snake_case, non_camel_case_types)]

use std::cmp::Ordering;
use std::convert::TryFrom;
use std::os::raw::{c_char, c_double, c_int, c_long, c_uchar, c_uint};
use std::time::SystemTime;
use std::{mem, ptr, time};

mod interactive;
mod mode_ac;
mod mode_s;

use mode_s::modesChecksum;
pub use mode_s::{computeMagnitudeVectorImpl, detectModeSImpl, errorinfo};

pub const MODES_NET_SNDBUF_MAX: c_int = 7;

const MODES_USER_LONGITUDE_DFLT: c_double = 0.0f64;
const MODES_USER_LATITUDE_DFLT: c_double = 0.0f64;
const MODES_INTERACTIVE_DISPLAY_TTL: c_int = 60 as c_int;
const MODES_NET_HEARTBEAT_RATE: c_int = 900 as c_int;
pub const MODES_USER_LATLON_VALID: c_int = (1 as c_int) << 0 as c_int;
pub const MODES_ASYNC_BUF_SIZE: usize = 16 * 16384; // 256k
pub const MODES_ASYNC_BUF_SAMPLES: usize = MODES_ASYNC_BUF_SIZE / 2; // Each sample is 2 bytes
const MODES_MSG_SQUELCH_LEVEL: c_int = 0x02FF; // Average signal strength limit
const MODES_MSG_ENCODER_ERRS: c_int = 3; // Maximum number of encoding errors

// When changing, change also fixBitErrors() and modesInitErrorTable() !!
pub const MODES_MAX_BITERRORS: usize = 2; // Global max for fixable bit errors

const MODES_PREAMBLE_US: usize = 8; // microseconds = bits
pub const MODES_PREAMBLE_SAMPLES: usize = MODES_PREAMBLE_US * 2;
const MODES_PREAMBLE_SIZE: usize = MODES_PREAMBLE_SAMPLES * mem::size_of::<u16>();
pub(crate) const MODES_LONG_MSG_BYTES: usize = 14;
const MODES_SHORT_MSG_BYTES: usize = 7;
const MODES_LONG_MSG_BITS: c_int = MODES_LONG_MSG_BYTES as c_int * 8;
const MODES_SHORT_MSG_BITS: c_int = MODES_SHORT_MSG_BYTES as c_int * 8;
pub const MODES_LONG_MSG_SAMPLES: usize = MODES_LONG_MSG_BITS as usize * 2;
const MODES_LONG_MSG_SIZE: usize = MODES_LONG_MSG_SAMPLES * mem::size_of::<u16>();

pub const MODES_RAWOUT_BUF_SIZE: usize = 1500;
pub const MODES_RAWOUT_BUF_FLUSH: usize = MODES_RAWOUT_BUF_SIZE - 200;
pub const MODES_RAWOUT_BUF_RATE: c_int = 1000; // 1000 * 64mS = 1 Min approx

pub const MODES_ICAO_CACHE_LEN: u32 = 1024; // Value must be a power of two
const MODES_ICAO_CACHE_TTL: u64 = 60; // Time to live of cached addresses
const MODES_UNIT_FEET: c_int = 0;
const MODES_UNIT_METERS: c_int = 1;

const MODES_ACFLAGS_LATLON_VALID: c_int = 1 << 0; // Aircraft Lat/Lon is decoded
const MODES_ACFLAGS_ALTITUDE_VALID: c_int = 1 << 1; // Aircraft altitude is known
const MODES_ACFLAGS_HEADING_VALID: c_int = 1 << 2; // Aircraft heading is known
const MODES_ACFLAGS_SPEED_VALID: c_int = 1 << 3; // Aircraft speed is known
const MODES_ACFLAGS_VERTRATE_VALID: c_int = 1 << 4; // Aircraft vertical rate is known
const MODES_ACFLAGS_SQUAWK_VALID: c_int = 1 << 5; // Aircraft Mode A Squawk is known
const MODES_ACFLAGS_CALLSIGN_VALID: c_int = 1 << 6; // Aircraft Callsign Identity
const MODES_ACFLAGS_EWSPEED_VALID: c_int = 1 << 7; // Aircraft East West Speed is known
const MODES_ACFLAGS_NSSPEED_VALID: c_int = 1 << 8; // Aircraft North South Speed is known
const MODES_ACFLAGS_AOG: c_int = 1 << 9; // Aircraft is On the Ground
const MODES_ACFLAGS_LLEVEN_VALID: c_int = 1 << 10; // Aircraft Even Lot/Lon is known
const MODES_ACFLAGS_LLODD_VALID: c_int = 1 << 11; // Aircraft Odd Lot/Lon is known
const MODES_ACFLAGS_AOG_VALID: c_int = 1 << 12; // MODES_ACFLAGS_AOG is valid
const MODES_ACFLAGS_LATLON_REL_OK: c_int = 1 << 15; // Indicates it's OK to do a relative CPR

const MODES_ACFLAGS_LLEITHER_VALID: c_int = MODES_ACFLAGS_LLEVEN_VALID | MODES_ACFLAGS_LLODD_VALID;
const MODES_ACFLAGS_LLBOTH_VALID: c_int = MODES_ACFLAGS_LLEVEN_VALID | MODES_ACFLAGS_LLODD_VALID;

const MODES_DEBUG_DEMOD: c_int = 1 << 0;
const MODES_DEBUG_DEMODERR: c_int = 1 << 1;
const MODES_DEBUG_BADCRC: c_int = 1 << 2;
const MODES_DEBUG_GOODCRC: c_int = 1 << 3;
const MODES_DEBUG_NOPREAMBLE: c_int = 1 << 4;

// When debug is set to MODES_DEBUG_NOPREAMBLE, the first sample must be
// at least greater than a given level for us to dump the signal.
const MODES_DEBUG_NOPREAMBLE_LEVEL: c_int = 25;

pub const NERRORINFO: usize =
    (MODES_LONG_MSG_BITS + MODES_LONG_MSG_BITS * (MODES_LONG_MSG_BITS - 1) / 2) as usize;

type time_t = c_long;

#[derive(Copy, Clone)]
#[repr(C)]
struct aircraft {
    addr: u32,                 // ICAO address
    flight: [c_char; 16],      // Flight number
    signalLevel: [c_uchar; 8], // Last 8 Signal Amplitudes
    altitude: c_int,           // Altitude
    speed: c_int,              // Velocity
    track: c_int,              // Angle of flight
    vert_rate: c_int,          // Vertical rate.
    seen: time_t,              // Time at which the last packet was received
    seenLatLon: time_t,        // Time at which the last lat long was calculated
    timestamp: u64,            // Timestamp at which the last packet was received
    timestampLatLon: u64,      // Timestamp at which the last lat long was calculated
    messages: c_long,          // Number of Mode S messages received
    modeA: c_int,              // Squawk
    modeC: c_int,              // Altitude
    modeAcount: c_long,        // Mode A Squawk hit Count
    modeCcount: c_long,        // Mode C Altitude hit Count
    modeACflags: c_int,        // Flags for mode A/C recognition

    // Encoded latitude and longitude as extracted by odd and even CPR encoded messages
    odd_cprlat: c_int,
    odd_cprlon: c_int,
    even_cprlat: c_int,
    even_cprlon: c_int,
    odd_cprtime: u64,
    even_cprtime: u64,
    lat: c_double,
    lon: c_double,       // Coordinated obtained from CPR encoded data
    bFlags: c_int,       // Flags related to valid fields in this structure
    next: *mut aircraft, // Next aircraft in our linked list
}

// Program global state
#[derive(Clone)]
#[repr(C)]
pub struct modes {
    pub magnitude: *mut u16, // Magnitude vector
    timestampBlk: u64,       // Timestamp of the start of the current block
    icao_cache: *mut u32,    // Recently seen ICAO addresses cache
    maglut: *mut u16,        // I/Q -> Magnitude lookup table

    // Networking
    rawOut: *mut c_char,   // Buffer for building raw output data
    rawOutUsed: c_int,     // How much of the buffer is currently used
    beastOut: *mut c_char, // Buffer for building beast output data
    beastOutUsed: c_int,   // How much if the buffer is currently used

    // Configuration
    phase_enhance: c_int,             // Enable phase enhancement if true
    nfix_crc: c_int,                  // Number of crc bit error(s) to correct
    check_crc: c_int,                 // Only display messages with good CRC
    raw: c_int,                       // Raw output format
    mode_ac: c_int,                   // Enable decoding of SSR Modes A & C
    debug: c_int,                     // Debugging mode
    net: c_int,                       // Enable networking
    net_heartbeat_count: c_int,       // TCP heartbeat counter
    net_heartbeat_rate: c_int,        // TCP heartbeat rate
    net_output_raw_size: c_int,       // Minimum Size of the output raw data
    net_output_raw_rate: c_int,       // Rate (in 64mS increments) of output raw data
    net_output_raw_rate_count: c_int, // Rate (in 64mS increments) of output raw data
    net_sndbuf_size: c_int,           // TCP output buffer size (64Kb * 2^n)
    quiet: c_int,                     // Suppress stdout
    interactive: c_int,               // Interactive mode
    interactive_display_ttl: c_int,   // Interactive mode: TTL display
    stats: c_int,                     // Print stats at exit in --ifile mode
    onlyaddr: c_int,                  // Print only ICAO addresses
    mlat: c_int, // Use Beast ascii format for raw data output, i.e. @...; iso *...;

    // User details
    fUserLat: c_double, // Users receiver/antenna lat/lon needed for initial surface location
    fUserLon: c_double, // Users receiver/antenna lat/lon needed for initial surface location
    bUserFlags: c_int,  // Flags relating to the user details

    // Interactive mode
    aircrafts: *mut aircraft,

    // DF List mode
    bEnableDFLogging: c_int, // Set to enable DF Logging

    // DF List mode
    stat_valid_preamble: c_uint,
    stat_demodulated0: c_uint,
    stat_demodulated1: c_uint,
    stat_demodulated2: c_uint,
    stat_demodulated3: c_uint,
    stat_goodcrc: c_uint,
    stat_badcrc: c_uint,
    stat_fixed: c_uint,

    // Histogram of fixed bit errors: index 0 for single bit errors,
    // index 1 for double bit errors etc.
    stat_bit_fix: [c_uint; MODES_MAX_BITERRORS],

    stat_out_of_phase: c_uint,
    stat_ph_demodulated0: c_uint,
    stat_ph_demodulated1: c_uint,
    stat_ph_demodulated2: c_uint,
    stat_ph_demodulated3: c_uint,
    stat_ph_goodcrc: c_uint,
    stat_ph_badcrc: c_uint,
    stat_ph_fixed: c_uint,
    // Histogram of fixed bit errors: index 0 for single bit errors,
    // index 1 for double bit errors etc.
    stat_ph_bit_fix: [c_uint; MODES_MAX_BITERRORS],

    stat_DF_Len_Corrected: c_uint,
    stat_DF_Type_Corrected: c_uint,
    stat_ModeAC: c_uint,
}

#[derive(Copy, Clone, Default)]
#[repr(C)]
struct modesMessage {
    msg: [c_uchar; MODES_LONG_MSG_BYTES],     // Binary message.
    msgbits: c_int,                           // Number of bits in message
    msgtype: c_int,                           // Downlink format #
    crcok: c_int,                             // True if CRC was valid
    crc: u32,                                 // Message CRC
    correctedbits: c_int,                     // No. of bits corrected
    corrected: [c_char; MODES_MAX_BITERRORS], // corrected bit positions
    addr: u32,                                // ICAO Address from bytes 1 2 and 3
    phase_corrected: c_int,                   // True if phase correction was applied
    timestampMsg: u64,                        // Timestamp of the message
    remote: c_int,                            // If set this message is from a remote station
    signalLevel: c_uchar,                     // Signal Amplitude

    // DF 11
    ca: c_int, // Responder capabilities
    iid: c_int,

    // DF 17, DF 18
    metype: c_int,        // Extended squitter message type.
    mesub: c_int,         // Extended squitter message subtype.
    heading: c_int,       // Reported by aircraft, or computed from from EW and NS velocity
    raw_latitude: c_int,  // Non decoded latitude.
    raw_longitude: c_int, // Non decoded longitude.
    fLat: c_double,       // Coordinates obtained from CPR encoded data if/when decoded
    fLon: c_double,       // Coordinates obtained from CPR encoded data if/when decoded
    flight: [c_char; 16], // 8 chars flight number.
    ew_velocity: c_int,   // E/W velocity.
    ns_velocity: c_int,   // N/S velocity.
    vert_rate: c_int,     // Vertical rate.
    velocity: c_int,      // Reported by aircraft, or computed from from EW and NS velocity

    // DF4, DF5, DF20, DF21
    fs: c_int,    // Flight status for DF4,5,20,21
    modeA: c_int, // 13 bits identity (Squawk).

    // Fields used by multiple message types.
    altitude: c_int,
    unit: c_int,
    bFlags: c_int, // Flags related to fields in this structure
}

impl Default for modes {
    fn default() -> Self {
        modes {
            check_crc: 1,
            raw: 0,
            mode_ac: 0,
            debug: 0,
            net: 0,
            net_heartbeat_count: 0,
            net_heartbeat_rate: MODES_NET_HEARTBEAT_RATE,
            net_output_raw_size: 0,
            net_output_raw_rate: 0,
            net_output_raw_rate_count: 0,
            net_sndbuf_size: 0,
            quiet: 0,
            interactive: 0,
            stats: 0,
            onlyaddr: 0,
            mlat: 0,
            interactive_display_ttl: MODES_INTERACTIVE_DISPLAY_TTL,
            fUserLat: MODES_USER_LATITUDE_DFLT,
            fUserLon: MODES_USER_LONGITUDE_DFLT,

            bUserFlags: 0,
            aircrafts: ptr::null_mut(),
            bEnableDFLogging: 0,
            stat_valid_preamble: 0,
            stat_demodulated0: 0,
            stat_demodulated1: 0,
            stat_demodulated2: 0,
            stat_demodulated3: 0,
            stat_goodcrc: 0,
            stat_badcrc: 0,
            stat_fixed: 0,
            stat_bit_fix: [0; MODES_MAX_BITERRORS],
            stat_out_of_phase: 0,
            stat_ph_demodulated0: 0,
            stat_ph_demodulated1: 0,
            stat_ph_demodulated2: 0,
            stat_ph_demodulated3: 0,
            stat_ph_goodcrc: 0,
            stat_ph_badcrc: 0,
            stat_ph_fixed: 0,
            stat_ph_bit_fix: [0; MODES_MAX_BITERRORS],
            stat_DF_Len_Corrected: 0,
            stat_DF_Type_Corrected: 0,
            stat_ModeAC: 0,
            icao_cache: ptr::null_mut(),
            magnitude: ptr::null_mut(),
            timestampBlk: 0,
            maglut: ptr::null_mut(),

            rawOut: ptr::null_mut(),
            rawOutUsed: 0,
            beastOut: ptr::null_mut(),
            beastOutUsed: 0,

            phase_enhance: 0,
            nfix_crc: 0,
        }
    }
}

fn cmp_errorinfo(e0: &errorinfo, e1: &errorinfo) -> Ordering {
    e0.syndrome.cmp(&e1.syndrome)
}

// TODO: Can this be made a const fn?
// Compute the table of all syndromes for 1-bit and 2-bit error vectors
pub unsafe fn modesInitErrorInfoImpl(bitErrorTable: &mut [errorinfo], nfix_crc: c_int) {
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

fn now() -> u64 {
    SystemTime::now()
        .duration_since(time::UNIX_EPOCH)
        .map(|duration| duration.as_secs())
        .expect("now doesn't fit in u64")
}

pub fn modes_init() -> (modes, [errorinfo; NERRORINFO]) {
    let mut state = modes {
        nfix_crc: MODES_MAX_BITERRORS as c_int, // --aggressive
        phase_enhance: 1,                       // --phase-enhance
        ..Default::default()
    };
    let mut bit_error_table = [errorinfo::default(); NERRORINFO];

    // Allocate the various buffers used by Modes
    let mut icao_cache = Box::new([0u32; MODES_ICAO_CACHE_LEN as usize * 2]);
    state.icao_cache = icao_cache.as_mut_ptr();
    Box::into_raw(icao_cache);

    let mut magnitude =
        Box::new([0u16; MODES_ASYNC_BUF_SAMPLES + MODES_PREAMBLE_SAMPLES + MODES_LONG_MSG_SAMPLES]);
    state.magnitude = magnitude.as_mut_ptr();
    Box::into_raw(magnitude);

    let mut beast_out = Box::new([0 as c_char; MODES_RAWOUT_BUF_SIZE]);
    state.beastOut = beast_out.as_mut_ptr();
    Box::into_raw(beast_out);

    let mut raw_out = Box::new([0 as c_char; MODES_RAWOUT_BUF_SIZE]);
    state.rawOut = raw_out.as_mut_ptr();
    Box::into_raw(raw_out);

    // Validate the users Lat/Lon home location inputs
    if state.fUserLat > 90.0f64
        || state.fUserLat < -90.0f64
        || state.fUserLon > 360.0f64
        || state.fUserLon < -180.0f64
    {
        state.fUserLon = 0.0f64;
        state.fUserLat = state.fUserLon
    } else if state.fUserLon > 180.0f64 {
        // If Longitude is +180 to +360, make it -180 to 0
        state.fUserLon -= 360.0f64
    }

    // If both Lat and Lon are 0.0 then the users location is either invalid/not-set, or (s)he's in the
    // Atlantic ocean off the west coast of Africa. This is unlikely to be correct.
    // Set the user LatLon valid flag only if either Lat or Lon are non zero. Note the Greenwich meridian
    // is at 0.0 Lon,so we must check for either fLat or fLon being non zero not both.
    // Testing the flag at runtime will be much quicker than ((fLon != 0.0) || (fLat != 0.0))
    state.bUserFlags &= !MODES_USER_LATLON_VALID;
    if state.fUserLat != 0.0f64 || state.fUserLon != 0.0f64 {
        state.bUserFlags |= MODES_USER_LATLON_VALID
    }

    // Limit the maximum requested raw output size to less than one Ethernet Block
    if state.net_output_raw_size > MODES_RAWOUT_BUF_SIZE as c_int - 200 {
        state.net_output_raw_size = MODES_RAWOUT_BUF_FLUSH as c_int
    }
    if state.net_output_raw_rate > 1000 {
        state.net_output_raw_rate = MODES_RAWOUT_BUF_RATE
    }
    if state.net_sndbuf_size > 7 {
        state.net_sndbuf_size = MODES_NET_SNDBUF_MAX
    }

    // Each I and Q value varies from 0 to 255, which represents a range from -1 to +1. To get from the
    // unsigned (0-255) range you therefore subtract 127 (or 128 or 127.5) from each I and Q, giving you
    // a range from -127 to +128 (or -128 to +127, or -127.5 to +127.5)..
    //
    // To decode the AM signal, you need the magnitude of the waveform, which is given by sqrt((I^2)+(Q^2))
    // The most this could be is if I&Q are both 128 (or 127 or 127.5), so you could end up with a magnitude
    // of 181.019 (or 179.605, or 180.312)
    //
    // However, in reality the magnitude of the signal should never exceed the range -1 to +1, because the
    // values are I = rCos(w) and Q = rSin(w). Therefore the integer computed magnitude should (can?) never
    // exceed 128 (or 127, or 127.5 or whatever)
    //
    // If we scale up the results so that they range from 0 to 65535 (16 bits) then we need to multiply
    // by 511.99, (or 516.02 or 514). antirez's original code multiplies by 360, presumably because he's
    // assuming the maximim calculated amplitude is 181.019, and (181.019 * 360) = 65166.
    //
    // So lets see if we can improve things by subtracting 127.5, Well in integer arithmatic we can't
    // subtract half, so, we'll double everything up and subtract one, and then compensate for the doubling
    // in the multiplier at the end.
    //
    // If we do this we can never have I or Q equal to 0 - they can only be as small as +/- 1.
    // This gives us a minimum magnitude of root 2 (0.707), so the dynamic range becomes (1.414-255). This
    // also affects our scaling value, which is now 65535/(255 - 1.414), or 258.433254
    //
    // The sums then become mag = 258.433254 * (sqrt((I*2-255)^2 + (Q*2-255)^2) - 1.414)
    //                   or mag = (258.433254 * sqrt((I*2-255)^2 + (Q*2-255)^2)) - 365.4798
    //
    // We also need to clip mag just incaes any rogue I/Q values somehow do have a magnitude greater than 255.
    //
    // i = 0 as c_int;
    // while i <= 255 as c_int {
    //     q = 0 as c_int;
    //     while q <= 255 as c_int {
    //         let mut mag: c_int = 0;
    //         let mut mag_i: c_int = 0;
    //         let mut mag_q: c_int = 0;
    //         mag_i = i * 2 as c_int - 255 as c_int;
    //         mag_q = q * 2 as c_int - 255 as c_int;
    //         mag =
    //             round(sqrt((mag_i * mag_i + mag_q * mag_q) as c_double)
    //                 * 258.433254f64 - 365.4798f64) as c_int;
    //         *state.maglut.offset((i * 256 as c_int + q) as isize) =
    //             if mag < 65535 as c_int {
    //                 mag
    //             } else { 65535 as c_int } as u16;
    //         q += 1
    //     }
    //     i += 1
    // }
    // for (i = 0; i <= 255; i++) {
    //     for (q = 0; q <= 255; q++) {
    //         int mag, mag_i, mag_q;
    //
    //         mag_i = (i * 2) - 255;
    //         mag_q = (q * 2) - 255;
    //
    //         mag = (int) round((sqrt((mag_i*mag_i)+(mag_q*mag_q)) * 258.433254) - 365.4798);
    //
    //         Modes.maglut[(i*256)+q] = (uint16_t) ((mag < 65535) ? mag : 65535);
    //     }
    // }

    let mut maglut = Box::new([0u16; 256 * 256]);
    for i in 0..=255 {
        for q in 0..=255 {
            let mag_i = (i as c_int * 2) - 255;
            let mag_q = (q as c_int * 2) - 255;
            let mag = ((((mag_i * mag_i + mag_q * mag_q) as c_double).sqrt() * 258.433254)
                - 365.4798)
                .round() as c_int;

            maglut[(i * 256) + q] = u16::try_from(mag).unwrap_or(std::u16::MAX);
        }
    }
    state.maglut = maglut.as_mut_ptr();
    Box::into_raw(maglut);

    // Prepare error correction tables
    unsafe { modesInitErrorInfoImpl(&mut bit_error_table, state.nfix_crc) };

    (state, bit_error_table)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bit_error_table() {
        let mut bit_error_table = [errorinfo {
            syndrome: 0,
            bits: 0,
            pos: [0; 2],
        }; NERRORINFO];
        let nfix_crc_agressive = 2; // TODO: test with 1 and 2
        unsafe { modesInitErrorInfoImpl(&mut bit_error_table, nfix_crc_agressive) };

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
