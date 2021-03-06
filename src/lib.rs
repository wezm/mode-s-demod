use std::cmp::Ordering;
use std::convert::TryFrom;
use std::os::raw::{c_char, c_double, c_int, c_long, c_uchar, c_uint};
use std::time::{Instant, SystemTime};
use std::{fmt, mem, ptr, time};

mod interactive;
mod mode_ac;
mod mode_s;

use mode_s::mode_s_checksum;
pub use mode_s::{compute_magnitude_vector, detect_mode_s, ErrorInfo};

pub const MODES_NET_SNDBUF_MAX: c_int = 7;

const MODES_USER_LONGITUDE_DFLT: c_double = 0.0f64;
const MODES_USER_LATITUDE_DFLT: c_double = 0.0f64;
const MODES_INTERACTIVE_DISPLAY_TTL: c_int = 60 as c_int;
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

#[derive(Copy, Clone, Eq, PartialEq)]
enum Altitude {
    Feet,
    Metres,
}

#[derive(Clone)]
#[repr(C)]
struct Aircraft {
    addr: u32,                  // ICAO address
    flight: [u8; 8],            // Flight number
    signal_level: [c_uchar; 8], // Last 8 Signal Amplitudes
    altitude: c_int,            // Altitude
    speed: c_int,               // Velocity
    track: c_int,               // Angle of flight
    vert_rate: c_int,           // Vertical rate.
    seen: time_t,               // Time at which the last packet was received
    seen_lat_lon: time_t,       // Time at which the last lat long was calculated
    timestamp: u64,             // Timestamp at which the last packet was received
    timestamp_lat_lon: u64,     // Timestamp at which the last lat long was calculated
    messages: c_long,           // Number of Mode S messages received
    mode_a: c_int,              // Squawk
    mode_c: c_int,              // Altitude
    mode_a_count: c_long,       // Mode A Squawk hit Count
    mode_c_count: c_long,       // Mode C Altitude hit Count
    mode_ac_flags: c_int,       // Flags for mode A/C recognition

    // Encoded latitude and longitude as extracted by odd and even CPR encoded messages
    odd_cprlat: c_int,
    odd_cprlon: c_int,
    even_cprlat: c_int,
    even_cprlon: c_int,
    odd_cprtime: u64,
    even_cprtime: u64,
    lat: c_double,
    lon: c_double,  // Coordinated obtained from CPR encoded data
    b_flags: c_int, // Flags related to valid fields in this structure
}

// Program global state
#[repr(C)]
pub struct ModeS {
    pub magnitude: Vec<u16>, // Magnitude vector
    timestamp_blk: u64,      // Timestamp of the start of the current block
    icao_cache: ICAOCache,   // Recently seen ICAO addresses cache
    maglut: *mut u16,        // I/Q -> Magnitude lookup table

    // Networking
    raw_out: *mut c_char,   // Buffer for building raw output data
    raw_out_used: c_int,    // How much of the buffer is currently used
    beast_out: *mut c_char, // Buffer for building beast output data
    beast_out_used: c_int,  // How much if the buffer is currently used

    // Configuration
    phase_enhance: bool,            // Enable phase enhancement if true
    nfix_crc: c_int,                // Number of crc bit error(s) to correct
    check_crc: bool,                // Only display messages with good CRC
    raw: bool,                      // Raw output format
    mode_ac: bool,                  // Enable decoding of SSR Modes A & C
    debug: c_int,                   // Debugging mode
    quiet: bool,                    // Suppress stdout
    interactive: bool,              // Interactive mode
    interactive_display_ttl: c_int, // Interactive mode: TTL display
    enable_stats: bool,             // Print stats at exit in --ifile mode
    onlyaddr: bool,                 // Print only ICAO addresses
    mlat: bool, // Use Beast ascii format for raw data output, i.e. @...; iso *...;

    // User details
    f_user_lat: Option<c_double>, // Users receiver/antenna lat/lon needed for initial surface location
    f_user_lon: Option<c_double>, // Users receiver/antenna lat/lon needed for initial surface location
    b_user_flags: c_int,          // Flags relating to the user details

    // Interactive mode
    aircrafts: Vec<Aircraft>,

    // DF List mode
    b_enable_dflogging: bool, // Set to enable DF Logging

    stats: Stats,
}

struct ICAOCache {
    cache: Box<[(u32, Instant); MODES_ICAO_CACHE_LEN as usize]>,
}

/// Various stats and counters
struct Stats {
    // DF List mode
    valid_preamble: c_uint,
    demodulated0: c_uint,
    demodulated1: c_uint,
    demodulated2: c_uint,
    demodulated3: c_uint,
    goodcrc: c_uint,
    badcrc: c_uint,
    fixed: c_uint,

    // Histogram of fixed bit errors: index 0 for single bit errors,
    // index 1 for double bit errors etc.
    bit_fix: [c_uint; MODES_MAX_BITERRORS],

    out_of_phase: c_uint,
    ph_demodulated0: c_uint,
    ph_demodulated1: c_uint,
    ph_demodulated2: c_uint,
    ph_demodulated3: c_uint,
    ph_goodcrc: c_uint,
    ph_badcrc: c_uint,
    ph_fixed: c_uint,
    // Histogram of fixed bit errors: index 0 for single bit errors,
    // index 1 for double bit errors etc.
    ph_bit_fix: [c_uint; MODES_MAX_BITERRORS],

    df_len_corrected: c_uint,
    df_type_corrected: c_uint,
    mode_ac: c_uint,
}

#[derive(Clone, Default)]
#[repr(C)]
struct ModesMessage {
    msg: [c_uchar; MODES_LONG_MSG_BYTES],     // Binary message.
    msgbits: c_int,                           // Number of bits in message
    msgtype: c_int,                           // Downlink format #
    crcok: c_int,                             // True if CRC was valid
    crc: u32,                                 // Message CRC
    correctedbits: c_int,                     // No. of bits corrected
    corrected: [c_char; MODES_MAX_BITERRORS], // corrected bit positions
    addr: u32,                                // ICAO Address from bytes 1 2 and 3
    phase_corrected: c_int,                   // True if phase correction was applied
    timestamp_msg: u64,                       // Timestamp of the message
    remote: c_int,                            // If set this message is from a remote station
    signal_level: c_uchar,                    // Signal Amplitude

    // DF 11
    ca: c_int, // Responder capabilities
    iid: c_int,

    // DF 17, DF 18
    metype: c_int,        // Extended squitter message type.
    mesub: c_int,         // Extended squitter message subtype.
    heading: c_int,       // Reported by Aircraft, or computed from from EW and NS velocity
    raw_latitude: c_int,  // Non decoded latitude.
    raw_longitude: c_int, // Non decoded longitude.
    f_lat: c_double,      // Coordinates obtained from CPR encoded data if/when decoded
    f_lon: c_double,      // Coordinates obtained from CPR encoded data if/when decoded
    flight: [u8; 8],      // 8 chars flight number.
    ew_velocity: c_int,   // E/W velocity.
    ns_velocity: c_int,   // N/S velocity.
    vert_rate: c_int,     // Vertical rate.
    velocity: c_int,      // Reported by Aircraft, or computed from from EW and NS velocity

    // DF4, DF5, DF20, DF21
    fs: c_int,     // Flight status for DF4,5,20,21
    mode_a: c_int, // 13 bits identity (Squawk).

    // Fields used by multiple message types.
    altitude: c_int, // TODO: Combine altitude and unit
    unit: Altitude,
    b_flags: c_int, // Flags related to fields in this structure
}

impl Default for Altitude {
    fn default() -> Self {
        Altitude::Feet
    }
}

impl fmt::Display for Altitude {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Altitude::Feet => f.write_str("feet"),
            Altitude::Metres => f.write_str("metres"),
        }
    }
}

impl ICAOCache {
    fn new() -> Self {
        let now = Instant::now();
        ICAOCache {
            cache: Box::new([(0u32, now); MODES_ICAO_CACHE_LEN as usize]),
        }
    }

    // Hash the ICAO address to index our cache of MODES_ICAO_CACHE_LEN
    // elements, that is assumed to be a power of two
    fn icao_cache_hash_address(mut a: u32) -> u32 {
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
    fn add_recently_seen_addr(&mut self, addr: u32) {
        let h: u32 = Self::icao_cache_hash_address(addr);
        self.cache[h as usize] = (addr, Instant::now());
    }

    // Returns 1 if the specified ICAO address was seen in a DF format with
    // proper checksum (not xored with address) no more than * MODES_ICAO_CACHE_TTL
    // seconds ago. Otherwise returns 0.
    //
    fn address_was_recently_seen(&self, addr: u32) -> c_int {
        let h: u32 = Self::icao_cache_hash_address(addr);
        let (a, t) = self.cache[h as usize];
        let tn = Instant::now();
        let age = tn.duration_since(t);
        (a != 0 && a == addr && age.as_secs() <= MODES_ICAO_CACHE_TTL) as c_int
    }
}

impl Default for ModeS {
    fn default() -> Self {
        ModeS {
            check_crc: true,
            raw: false,
            mode_ac: false,
            debug: 0,
            quiet: false,
            interactive: false,
            enable_stats: false,
            onlyaddr: false,
            mlat: false,
            interactive_display_ttl: MODES_INTERACTIVE_DISPLAY_TTL,
            f_user_lat: None,
            f_user_lon: None,

            b_user_flags: 0,
            aircrafts: Vec::new(),
            b_enable_dflogging: false,
            stats: Stats::default(),
            icao_cache: ICAOCache::new(),
            magnitude: vec![
                0;
                MODES_ASYNC_BUF_SAMPLES
                    + MODES_PREAMBLE_SAMPLES
                    + MODES_LONG_MSG_SAMPLES
            ],
            timestamp_blk: 0,
            maglut: ptr::null_mut(),

            raw_out: ptr::null_mut(),
            raw_out_used: 0,
            beast_out: ptr::null_mut(),
            beast_out_used: 0,

            phase_enhance: false,
            nfix_crc: 0,
        }
    }
}

impl Default for Stats {
    fn default() -> Self {
        Stats {
            valid_preamble: 0,
            demodulated0: 0,
            demodulated1: 0,
            demodulated2: 0,
            demodulated3: 0,
            goodcrc: 0,
            badcrc: 0,
            fixed: 0,
            bit_fix: [0; MODES_MAX_BITERRORS],
            out_of_phase: 0,
            ph_demodulated0: 0,
            ph_demodulated1: 0,
            ph_demodulated2: 0,
            ph_demodulated3: 0,
            ph_goodcrc: 0,
            ph_badcrc: 0,
            ph_fixed: 0,
            ph_bit_fix: [0; MODES_MAX_BITERRORS],
            df_len_corrected: 0,
            df_type_corrected: 0,
            mode_ac: 0,
        }
    }
}

fn cmp_errorinfo(e0: &ErrorInfo, e1: &ErrorInfo) -> Ordering {
    e0.syndrome.cmp(&e1.syndrome)
}

// TODO: This can be a const fn if sort_by becomes const
// Compute the table of all syndromes for 1-bit and 2-bit error vectors
pub fn modes_init_error_info(nfix_crc: c_int) -> [ErrorInfo; NERRORINFO] {
    let mut bit_error_table = [ErrorInfo::default(); NERRORINFO];

    let mut msg: [c_uchar; 14] = [0; MODES_LONG_MSG_BYTES as usize];
    let mut j: u8;
    let mut n: c_int = 0;
    let mut crc: u32;

    // Add all possible single and double bit errors
    // don't include errors in first 5 bits (DF type)
    let mut i = 5u8;
    while c_int::from(i) < MODES_LONG_MSG_BITS {
        let bytepos0 = i >> 3;
        let mask0: u8 = 1 << 7 - (i & 7);
        // revert error0
        msg[usize::from(bytepos0)] ^= mask0; // create error0
        crc = mode_s_checksum(msg, MODES_LONG_MSG_BITS); // single bit error case
        bit_error_table[n as usize].syndrome = crc;
        bit_error_table[n as usize].bits = 1;
        bit_error_table[n as usize].pos[0] = i8::try_from(i).unwrap();
        bit_error_table[n as usize].pos[1] = -1;
        n += 1;
        if nfix_crc > 1 {
            j = i + 1;
            while c_int::from(j) < MODES_LONG_MSG_BITS {
                let bytepos1 = j >> 3;
                let mask1: u8 = 1 << 7 - (j & 7);
                // revert error1
                msg[bytepos1 as usize] ^= mask1; // create error1
                crc = mode_s_checksum(msg, MODES_LONG_MSG_BITS); // two bit error case
                if n >= bit_error_table.len() as c_int {
                    break;
                }
                bit_error_table[n as usize].syndrome = crc;
                bit_error_table[n as usize].bits = 2;
                bit_error_table[n as usize].pos[0] = i8::try_from(i).unwrap();
                bit_error_table[n as usize].pos[1] = i8::try_from(j).unwrap();
                n += 1;
                msg[usize::from(bytepos1)] ^= mask1;
                j += 1
            }
        }
        msg[usize::from(bytepos0)] ^= mask0;
        i += 1
    }

    bit_error_table.sort_by(cmp_errorinfo);
    bit_error_table
}

fn now() -> u64 {
    SystemTime::now()
        .duration_since(time::UNIX_EPOCH)
        .map(|duration| duration.as_secs())
        .expect("now doesn't fit in u64")
}

pub fn modes_init() -> (ModeS, [ErrorInfo; NERRORINFO]) {
    let mut state = ModeS {
        nfix_crc: MODES_MAX_BITERRORS as c_int, // --aggressive
        phase_enhance: true,                    // --phase-enhance
        ..Default::default()
    };

    // Allocate the various buffers used by Modes
    let mut beast_out = Box::new([0 as c_char; MODES_RAWOUT_BUF_SIZE]);
    state.beast_out = beast_out.as_mut_ptr();
    Box::into_raw(beast_out);

    let mut raw_out = Box::new([0 as c_char; MODES_RAWOUT_BUF_SIZE]);
    state.raw_out = raw_out.as_mut_ptr();
    Box::into_raw(raw_out);

    // Validate the users Lat/Lon home location inputs
    match (state.f_user_lat, state.f_user_lon) {
        (Some(lat), Some(lon))
            if lat > 90.0f64 || lat < -90.0f64 || lon > 360.0f64 || lon < -180.0f64 =>
        {
            state.f_user_lat = None;
            state.f_user_lon = None;
        }
        (_, Some(lon)) if lon > 180.0f64 => {
            // If Longitude is +180 to +360, make it -180 to 0
            state.f_user_lon = Some(lon - 360.0f64)
        }
        (_, _) => (),
    }

    // If both Lat and Lon are 0.0 then the users location is either invalid/not-set, or (s)he's in the
    // Atlantic ocean off the west coast of Africa. This is unlikely to be correct.
    // Set the user LatLon valid flag only if either Lat or Lon are non zero. Note the Greenwich meridian
    // is at 0.0 Lon,so we must check for either f_lat or f_lon being non zero not both.
    // Testing the flag at runtime will be much quicker than ((f_lon != 0.0) || (f_lat != 0.0))
    state.b_user_flags &= !MODES_USER_LATLON_VALID;
    if state.f_user_lat.is_some() && state.f_user_lon.is_some() {
        state.b_user_flags |= MODES_USER_LATLON_VALID
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

    // Prepare error correction table
    let bit_error_table = modes_init_error_info(state.nfix_crc);

    (state, bit_error_table)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bit_error_table() {
        let nfix_crc_agressive = 2; // TODO: test with 1 and 2
        let bit_error_table = modes_init_error_info(nfix_crc_agressive);

        // Test code: report if any syndrome appears at least twice. In this
        // case the correction cannot be done without ambiguity.
        // Tried it, does not happen for 1- and 2-bit errors.
        let errorinfo_zero = ErrorInfo {
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

    #[test]
    fn test_size_of_error_info() {
        eprintln!("{}", mem::size_of::<ErrorInfo>() * NERRORINFO);
    }
}
