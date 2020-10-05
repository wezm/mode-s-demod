#![allow(non_snake_case, non_camel_case_types)]

use std::os::raw::{
    c_char, c_double, c_int, c_long, c_longlong, c_short, c_uchar, c_uint, c_ulong, c_ulonglong,
    c_ushort, c_void,
};

mod io;
mod mode_ac;
mod mode_s;

use mode_s::{MODES_ASYNC_BUF_NUMBER, MODES_LONG_MSG_BYTES, MODES_MAX_BITERRORS};

const ANET_ERR_LEN: usize = 256;
const MODES_CLIENT_BUF_SIZE: usize = 1024;

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
    pub buf: [c_char; MODES_CLIENT_BUF_SIZE + 1],
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
