#![allow(non_snake_case, non_camel_case_types)]

use std::os::raw::{
    c_char, c_double, c_int, c_long, c_longlong, c_short, c_uchar, c_uint, c_ulong, c_ulonglong,
    c_ushort, c_void,
};
use std::{mem, ptr, time};

mod io;
mod mode_ac;
mod mode_s;

const ANET_ERR_LEN: usize = 256;

const MODES_NET_SERVICES_NUM: c_int = 6;
const MODES_NET_INPUT_RAW_PORT: c_int = 30001;
const MODES_NET_OUTPUT_RAW_PORT: c_int = 30002;
const MODES_NET_OUTPUT_SBS_PORT: c_int = 30003;
const MODES_NET_INPUT_BEAST_PORT: c_int = 30004;
const MODES_NET_OUTPUT_BEAST_PORT: c_int = 30005;
const MODES_NET_HTTP_PORT: c_int = 8080;
const MODES_CLIENT_BUF_SIZE: c_int = 1024;
const MODES_NET_SNDBUF_SIZE: c_int = (1024 * 64);
pub const MODES_NET_SNDBUF_MAX: c_int = (7);

const MODES_USER_LONGITUDE_DFLT: c_double = 0.0f64;
const MODES_USER_LATITUDE_DFLT: c_double = 0.0f64;
const MODES_INTERACTIVE_DISPLAY_TTL: c_int = 60 as c_int;
const MODES_INTERACTIVE_DELETE_TTL: c_int = 300 as c_int;
const MODES_NET_HEARTBEAT_RATE: c_int = 900 as c_int;
const MODES_DEFAULT_PPM: c_int = 52 as c_int;
const MODES_DEFAULT_FREQ: c_int = 1090000000 as c_int;
const MODES_MAX_GAIN: c_int = 999999 as c_int;
pub const MODES_USER_LATLON_VALID: c_int = (1 as c_int) << 0 as c_int;
// const MODES_PREAMBLE_US: c_int = 8 as c_int;
// const MODES_PREAMBLE_SAMPLES: c_int =
//     MODES_PREAMBLE_US * 2 as c_int;
// const MODES_ASYNC_BUF_SIZE: c_int =
//     16 as c_int * 16384 as c_int;
// const MODES_ICAO_CACHE_LEN: c_int = 1024 as c_int;

// const MODES_DEFAULT_PPM: c_int = 52;
// const MODES_DEFAULT_RATE: c_int = 2000000;
// const MODES_DEFAULT_FREQ: c_int = 1090000000;
// const MODES_DEFAULT_WIDTH: c_int = 1000;
// const MODES_DEFAULT_HEIGHT: c_int = 700;
pub(crate) const MODES_ASYNC_BUF_NUMBER: usize = 16;
pub const MODES_ASYNC_BUF_SIZE: usize = 16 * 16384; // 256k
pub const MODES_ASYNC_BUF_SAMPLES: usize = MODES_ASYNC_BUF_SIZE / 2; // Each sample is 2 bytes
                                                                     // const MODES_AUTO_GAIN: c_int = -100; // Use automatic gain
                                                                     // const MODES_MAX_GAIN: c_int = 999999; // Use max available gain
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
// const MODES_SHORT_MSG_SAMPLES: usize = MODES_SHORT_MSG_BITS as usize * 2;
const MODES_LONG_MSG_SIZE: usize = MODES_LONG_MSG_SAMPLES * mem::size_of::<u16>();
// const MODES_SHORT_MSG_SIZE: usize = MODES_SHORT_MSG_SAMPLES * mem::size_of::<u16>();

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
                                                // const MODES_ACFLAGS_FS_VALID: c_int = 1 << 13; // Aircraft Flight Status is known
                                                // const MODES_ACFLAGS_NSEWSPD_VALID: c_int = 1 << 14; // Aircraft EW and NS Speed is known
const MODES_ACFLAGS_LATLON_REL_OK: c_int = 1 << 15; // Indicates it's OK to do a relative CPR

const MODES_ACFLAGS_LLEITHER_VALID: c_int = MODES_ACFLAGS_LLEVEN_VALID | MODES_ACFLAGS_LLODD_VALID;
const MODES_ACFLAGS_LLBOTH_VALID: c_int = MODES_ACFLAGS_LLEVEN_VALID | MODES_ACFLAGS_LLODD_VALID;
// const MODES_ACFLAGS_AOG_GROUND: c_int = MODES_ACFLAGS_AOG_VALID | MODES_ACFLAGS_AOG;

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

pub const NERRORINFO: usize =
    (MODES_LONG_MSG_BITS + MODES_LONG_MSG_BITS * (MODES_LONG_MSG_BITS - 1) / 2) as usize;

pub type time_t = c_long;

#[derive(Copy, Clone)]
#[repr(C)]
pub struct __pthread_internal_list {
    pub __prev: *mut __pthread_internal_list,
    pub __next: *mut __pthread_internal_list,
}

pub type __pthread_list_t = __pthread_internal_list;

#[derive(Copy, Clone)]
#[repr(C)]
pub struct __pthread_mutex_s {
    pub __lock: c_int,
    pub __count: c_uint,
    pub __owner: c_int,
    pub __nusers: c_uint,
    pub __kind: c_int,
    pub __spins: c_short,
    pub __elision: c_short,
    pub __list: __pthread_list_t,
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct __pthread_cond_s {
    pub c2rust_unnamed: C2RustUnnamed_1,
    pub c2rust_unnamed_0: C2RustUnnamed,
    pub __g_refs: [c_uint; 2],
    pub __g_size: [c_uint; 2],
    pub __g1_orig_size: c_uint,
    pub __wrefs: c_uint,
    pub __g_signals: [c_uint; 2],
}

#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed {
    pub __g1_start: c_ulonglong,
    pub __g1_start32: C2RustUnnamed_0,
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct C2RustUnnamed_0 {
    pub __low: c_uint,
    pub __high: c_uint,
}

#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_1 {
    pub __wseq: c_ulonglong,
    pub __wseq32: C2RustUnnamed_2,
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct C2RustUnnamed_2 {
    pub __low: c_uint,
    pub __high: c_uint,
}
pub type pthread_t = c_ulong;

#[derive(Copy, Clone)]
#[repr(C)]
pub union pthread_mutex_t {
    pub __data: __pthread_mutex_s,
    pub __size: [c_char; 40],
    pub __align: c_long,
}

#[derive(Copy, Clone)]
#[repr(C)]
pub union pthread_cond_t {
    pub __data: __pthread_cond_s,
    pub __size: [c_char; 48],
    pub __align: c_longlong,
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct timeb {
    pub time: time_t,
    pub millitm: c_ushort,
    pub timezone: c_short,
    pub dstflag: c_short,
}

type rtlsdr_dev_t = *mut c_void;

#[derive(Copy, Clone)]
#[repr(C)]
pub struct client {
    pub next: *mut client,
    pub fd: c_int,
    pub service: c_int,
    pub buflen: c_int,
    pub buf: [c_char; MODES_CLIENT_BUF_SIZE as usize + 1],
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct aircraft {
    pub addr: u32,                 // ICAO address
    pub flight: [c_char; 16],      // Flight number
    pub signalLevel: [c_uchar; 8], // Last 8 Signal Amplitudes
    pub altitude: c_int,           // Altitude
    pub speed: c_int,              // Velocity
    pub track: c_int,              // Angle of flight
    pub vert_rate: c_int,          // Vertical rate.
    pub seen: time_t,              // Time at which the last packet was received
    pub seenLatLon: time_t,        // Time at which the last lat long was calculated
    pub timestamp: u64,            // Timestamp at which the last packet was received
    pub timestampLatLon: u64,      // Timestamp at which the last lat long was calculated
    pub messages: c_long,          // Number of Mode S messages received
    pub modeA: c_int,              // Squawk
    pub modeC: c_int,              // Altitude
    pub modeAcount: c_long,        // Mode A Squawk hit Count
    pub modeCcount: c_long,        // Mode C Altitude hit Count
    pub modeACflags: c_int,        // Flags for mode A/C recognition

    // Encoded latitude and longitude as extracted by odd and even CPR encoded messages
    pub odd_cprlat: c_int,
    pub odd_cprlon: c_int,
    pub even_cprlat: c_int,
    pub even_cprlon: c_int,
    pub odd_cprtime: u64,
    pub even_cprtime: u64,
    pub lat: c_double,
    pub lon: c_double,       // Coordinated obtained from CPR encoded data
    pub bFlags: c_int,       // Flags related to valid fields in this structure
    pub next: *mut aircraft, // Next aircraft in our linked list
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct stDF {
    pub pNext: *mut stDF,         // Pointer to next item in the linked list
    pub pPrev: *mut stDF,         // Pointer to previous item in the linked list
    pub pAircraft: *mut aircraft, // Pointer to the Aircraft structure for this DF
    pub seen: time_t,             // Dos/UNIX Time at which the this packet was received
    pub llTimestamp: u64,         // Timestamp at which the this packet was received
    pub addr: u32,                // Timestamp at which the this packet was received
    pub msg: [c_uchar; MODES_LONG_MSG_BYTES], // the binary
}

// Program global state
#[derive(Clone)]
#[repr(C)]
pub struct modes {
    pub reader_thread: pthread_t,

    pub data_mutex: pthread_mutex_t, // Mutex to synchronize buffer access
    pub data_cond: pthread_cond_t,   // Conditional variable associated
    pub pData: [*mut u16; MODES_ASYNC_BUF_NUMBER], // Raw IQ sample buffers from RTL
    pub stSystemTimeRTL: [timeb; MODES_ASYNC_BUF_NUMBER], // System time when RTL passed us this block
    pub iDataIn: c_int,                                   // Fifo input pointer
    pub iDataOut: c_int,                                  // Fifo output pointer
    pub iDataReady: c_int,                                // Fifo content count
    pub iDataLost: c_int,                                 // Count of missed buffers

    pub pFileData: *mut u16,    // Raw IQ samples buffer (from a File)
    pub magnitude: *mut u16,    // Magnitude vector
    pub timestampBlk: u64,      // Timestamp of the start of the current block
    pub stSystemTimeBlk: timeb, // System time when RTL passed us currently processing this block
    pub fd: c_int,              // --ifile option file descriptor
    pub icao_cache: *mut u32,   // Recently seen ICAO addresses cache
    pub maglut: *mut u16,       // I/Q -> Magnitude lookup table
    pub exit: c_int,            // Exit from the main loop when true

    // RTLSDR
    pub dev_index: c_int,
    pub gain: c_int,
    pub enable_agc: c_int,
    pub dev: *mut rtlsdr_dev_t,
    pub freq: c_int,
    pub ppm_error: c_int,

    // Networking
    pub aneterr: [c_char; ANET_ERR_LEN],
    pub clients: *mut client,  // Our clients
    pub sbsos: c_int,          // SBS output listening socket
    pub ros: c_int,            // Raw output listening socket
    pub ris: c_int,            // Raw input listening socket
    pub bos: c_int,            // Beast output listening socket
    pub bis: c_int,            // Beast input listening socket
    pub https: c_int,          // HTTP listening socket
    pub rawOut: *mut c_char,   // Buffer for building raw output data
    pub rawOutUsed: c_int,     // How much of the buffer is currently used
    pub beastOut: *mut c_char, // Buffer for building beast output data
    pub beastOutUsed: c_int,   // How much if the buffer is currently used

    // Configuration
    pub filename: *mut c_char,            // Input form file, --ifile option
    pub phase_enhance: c_int,             // Enable phase enhancement if true
    pub nfix_crc: c_int,                  // Number of crc bit error(s) to correct
    pub check_crc: c_int,                 // Only display messages with good CRC
    pub raw: c_int,                       // Raw output format
    pub beast: c_int,                     // Beast binary format output
    pub mode_ac: c_int,                   // Enable decoding of SSR Modes A & C
    pub debug: c_int,                     // Debugging mode
    pub net: c_int,                       // Enable networking
    pub net_only: c_int,                  // Enable just networking
    pub net_heartbeat_count: c_int,       // TCP heartbeat counter
    pub net_heartbeat_rate: c_int,        // TCP heartbeat rate
    pub net_output_sbs_port: c_int,       // SBS output TCP port
    pub net_output_raw_size: c_int,       // Minimum Size of the output raw data
    pub net_output_raw_rate: c_int,       // Rate (in 64mS increments) of output raw data
    pub net_output_raw_rate_count: c_int, // Rate (in 64mS increments) of output raw data
    pub net_output_raw_port: c_int,       // Raw output TCP port
    pub net_input_raw_port: c_int,        // Raw input TCP port
    pub net_output_beast_port: c_int,     // Beast output TCP port
    pub net_input_beast_port: c_int,      // Beast input TCP port
    pub net_bind_address: *mut c_char,    // Bind address
    pub net_http_port: c_int,             // HTTP port
    pub net_sndbuf_size: c_int,           // TCP output buffer size (64Kb * 2^n)
    pub quiet: c_int,                     // Suppress stdout
    pub interactive: c_int,               // Interactive mode
    pub interactive_rows: c_int,          // Interactive mode: max number of rows
    pub interactive_display_ttl: c_int,   // Interactive mode: TTL display
    pub interactive_delete_ttl: c_int,    // Interactive mode: TTL before deletion
    pub stats: c_int,                     // Print stats at exit in --ifile mode
    pub onlyaddr: c_int,                  // Print only ICAO addresses
    pub metric: c_int,                    // Use metric units
    pub mlat: c_int, // Use Beast ascii format for raw data output, i.e. @...; iso *...;
    pub interactive_rtl1090: c_int, // flight table in interactive mode is formatted like RTL1090

    // User details
    pub fUserLat: c_double, // Users receiver/antenna lat/lon needed for initial surface location
    pub fUserLon: c_double, // Users receiver/antenna lat/lon needed for initial surface location
    pub bUserFlags: c_int,  // Flags relating to the user details

    // Interactive mode
    pub aircrafts: *mut aircraft,
    pub interactive_last_update: u64, // Last screen update in milliseconds
    pub last_cleanup_time: time_t,    // Last cleanup time in seconds

    // DF List mode
    pub bEnableDFLogging: c_int,    // Set to enable DF Logging
    pub pDF_mutex: pthread_mutex_t, // Mutex to synchronize pDF access
    pub pDF: *mut stDF,             // Pointer to DF list

    // DF List mode
    pub stat_valid_preamble: c_uint,
    pub stat_demodulated0: c_uint,
    pub stat_demodulated1: c_uint,
    pub stat_demodulated2: c_uint,
    pub stat_demodulated3: c_uint,
    pub stat_goodcrc: c_uint,
    pub stat_badcrc: c_uint,
    pub stat_fixed: c_uint,

    // Histogram of fixed bit errors: index 0 for single bit errors,
    // index 1 for double bit errors etc.
    pub stat_bit_fix: [c_uint; MODES_MAX_BITERRORS],

    pub stat_http_requests: c_uint,
    pub stat_sbs_connections: c_uint,
    pub stat_raw_connections: c_uint,
    pub stat_beast_connections: c_uint,
    pub stat_out_of_phase: c_uint,
    pub stat_ph_demodulated0: c_uint,
    pub stat_ph_demodulated1: c_uint,
    pub stat_ph_demodulated2: c_uint,
    pub stat_ph_demodulated3: c_uint,
    pub stat_ph_goodcrc: c_uint,
    pub stat_ph_badcrc: c_uint,
    pub stat_ph_fixed: c_uint,
    // Histogram of fixed bit errors: index 0 for single bit errors,
    // index 1 for double bit errors etc.
    pub stat_ph_bit_fix: [c_uint; MODES_MAX_BITERRORS],

    pub stat_DF_Len_Corrected: c_uint,
    pub stat_DF_Type_Corrected: c_uint,
    pub stat_ModeAC: c_uint,

    pub stat_blocks_processed: c_uint,
    pub stat_blocks_dropped: c_uint,
}

#[derive(Copy, Clone, Default)]
#[repr(C)]
pub struct modesMessage {
    pub msg: [c_uchar; MODES_LONG_MSG_BYTES], // Binary message.
    pub msgbits: c_int,                       // Number of bits in message
    pub msgtype: c_int,                       // Downlink format #
    pub crcok: c_int,                         // True if CRC was valid
    pub crc: u32,                             // Message CRC
    pub correctedbits: c_int,                 // No. of bits corrected
    pub corrected: [c_char; MODES_MAX_BITERRORS], // corrected bit positions
    pub addr: u32,                            // ICAO Address from bytes 1 2 and 3
    pub phase_corrected: c_int,               // True if phase correction was applied
    pub timestampMsg: u64,                    // Timestamp of the message
    pub remote: c_int,                        // If set this message is from a remote station
    pub signalLevel: c_uchar,                 // Signal Amplitude

    // DF 11
    pub ca: c_int, // Responder capabilities
    pub iid: c_int,

    // DF 17, DF 18
    pub metype: c_int,        // Extended squitter message type.
    pub mesub: c_int,         // Extended squitter message subtype.
    pub heading: c_int,       // Reported by aircraft, or computed from from EW and NS velocity
    pub raw_latitude: c_int,  // Non decoded latitude.
    pub raw_longitude: c_int, // Non decoded longitude.
    pub fLat: c_double,       // Coordinates obtained from CPR encoded data if/when decoded
    pub fLon: c_double,       // Coordinates obtained from CPR encoded data if/when decoded
    pub flight: [c_char; 16], // 8 chars flight number.
    pub ew_velocity: c_int,   // E/W velocity.
    pub ns_velocity: c_int,   // N/S velocity.
    pub vert_rate: c_int,     // Vertical rate.
    pub velocity: c_int,      // Reported by aircraft, or computed from from EW and NS velocity

    // DF4, DF5, DF20, DF21
    pub fs: c_int,    // Flight status for DF4,5,20,21
    pub modeA: c_int, // 13 bits identity (Squawk).

    // Fields used by multiple message types.
    pub altitude: c_int,
    pub unit: c_int,
    pub bFlags: c_int, // Flags related to fields in this structure
}
