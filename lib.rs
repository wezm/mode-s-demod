#![allow(non_snake_case, non_camel_case_types)]

use std::os::raw::{
    c_char, c_double, c_int, c_long, c_longlong, c_short, c_uchar, c_uint, c_ulong, c_ulonglong,
    c_ushort, c_void,
};

mod mode_s;

extern "C" {
    #[no_mangle]
    static mut Modes: modes;
// pub type _IO_wide_data;
// pub type _IO_codecvt;
// pub type _IO_marker;
// pub type rtlsdr_dev;
}

pub type size_t = c_ulong;
pub type __uint8_t = c_uchar;
pub type __uint16_t = c_ushort;
pub type __uint32_t = c_uint;
pub type __uint64_t = c_ulong;
pub type __off_t = c_long;
pub type __off64_t = c_long;
pub type __time_t = c_long;
// #[derive(Copy, Clone)]
// #[repr(C)]
// pub struct _IO_FILE {
//     pub _flags: c_int,
//     pub _IO_read_ptr: *mut c_char,
//     pub _IO_read_end: *mut c_char,
//     pub _IO_read_base: *mut c_char,
//     pub _IO_write_base: *mut c_char,
//     pub _IO_write_ptr: *mut c_char,
//     pub _IO_write_end: *mut c_char,
//     pub _IO_buf_base: *mut c_char,
//     pub _IO_buf_end: *mut c_char,
//     pub _IO_save_base: *mut c_char,
//     pub _IO_backup_base: *mut c_char,
//     pub _IO_save_end: *mut c_char,
//     pub _markers: *mut _IO_marker,
//     pub _chain: *mut _IO_FILE,
//     pub _fileno: c_int,
//     pub _flags2: c_int,
//     pub _old_offset: __off_t,
//     pub _cur_column: c_ushort,
//     pub _vtable_offset: c_schar,
//     pub _shortbuf: [c_char; 1],
//     pub _lock: *mut c_void,
//     pub _offset: __off64_t,
//     pub _codecvt: *mut _IO_codecvt,
//     pub _wide_data: *mut _IO_wide_data,
//     pub _freeres_list: *mut _IO_FILE,
//     pub _freeres_buf: *mut c_void,
//     pub __pad5: size_t,
//     pub _mode: c_int,
//     pub _unused2: [c_char; 20],
// }
pub type _IO_lock_t = ();
// pub type FILE = _IO_FILE;
pub type time_t = __time_t;
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
pub type __compar_fn_t = Option<unsafe extern "C" fn(_: *const c_void, _: *const c_void) -> c_int>;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type uint64_t = __uint64_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct timeb {
    pub time: time_t,
    pub millitm: c_ushort,
    pub timezone: c_short,
    pub dstflag: c_short,
}
// pub type rtlsdr_dev_t = rtlsdr_dev;
type rtlsdr_dev_t = *mut c_void;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct client {
    pub next: *mut client,
    pub fd: c_int,
    pub service: c_int,
    pub buflen: c_int,
    pub buf: [c_char; 1025],
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct aircraft {
    pub addr: uint32_t,
    pub flight: [c_char; 16],
    pub signalLevel: [c_uchar; 8],
    pub altitude: c_int,
    pub speed: c_int,
    pub track: c_int,
    pub vert_rate: c_int,
    pub seen: time_t,
    pub seenLatLon: time_t,
    pub timestamp: uint64_t,
    pub timestampLatLon: uint64_t,
    pub messages: c_long,
    pub modeA: c_int,
    pub modeC: c_int,
    pub modeAcount: c_long,
    pub modeCcount: c_long,
    pub modeACflags: c_int,
    pub odd_cprlat: c_int,
    pub odd_cprlon: c_int,
    pub even_cprlat: c_int,
    pub even_cprlon: c_int,
    pub odd_cprtime: uint64_t,
    pub even_cprtime: uint64_t,
    pub lat: c_double,
    pub lon: c_double,
    pub bFlags: c_int,
    pub next: *mut aircraft,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct stDF {
    pub pNext: *mut stDF,
    pub pPrev: *mut stDF,
    pub pAircraft: *mut aircraft,
    pub seen: time_t,
    pub llTimestamp: uint64_t,
    pub addr: uint32_t,
    pub msg: [c_uchar; 14],
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct modes {
    pub reader_thread: pthread_t,
    pub data_mutex: pthread_mutex_t,
    pub data_cond: pthread_cond_t,
    pub pData: [*mut uint16_t; 16],
    pub stSystemTimeRTL: [timeb; 16],
    pub iDataIn: c_int,
    pub iDataOut: c_int,
    pub iDataReady: c_int,
    pub iDataLost: c_int,
    pub pFileData: *mut uint16_t,
    pub magnitude: *mut uint16_t,
    pub timestampBlk: uint64_t,
    pub stSystemTimeBlk: timeb,
    pub fd: c_int,
    pub icao_cache: *mut uint32_t,
    pub maglut: *mut uint16_t,
    pub exit: c_int,
    pub dev_index: c_int,
    pub gain: c_int,
    pub enable_agc: c_int,
    pub dev: *mut rtlsdr_dev_t,
    pub freq: c_int,
    pub ppm_error: c_int,
    pub aneterr: [c_char; 256],
    pub clients: *mut client,
    pub sbsos: c_int,
    pub ros: c_int,
    pub ris: c_int,
    pub bos: c_int,
    pub bis: c_int,
    pub https: c_int,
    pub rawOut: *mut c_char,
    pub rawOutUsed: c_int,
    pub beastOut: *mut c_char,
    pub beastOutUsed: c_int,
    pub filename: *mut c_char,
    pub phase_enhance: c_int,
    pub nfix_crc: c_int,
    pub check_crc: c_int,
    pub raw: c_int,
    pub beast: c_int,
    pub mode_ac: c_int,
    pub debug: c_int,
    pub net: c_int,
    pub net_only: c_int,
    pub net_heartbeat_count: c_int,
    pub net_heartbeat_rate: c_int,
    pub net_output_sbs_port: c_int,
    pub net_output_raw_size: c_int,
    pub net_output_raw_rate: c_int,
    pub net_output_raw_rate_count: c_int,
    pub net_output_raw_port: c_int,
    pub net_input_raw_port: c_int,
    pub net_output_beast_port: c_int,
    pub net_input_beast_port: c_int,
    pub net_bind_address: *mut c_char,
    pub net_http_port: c_int,
    pub net_sndbuf_size: c_int,
    pub quiet: c_int,
    pub interactive: c_int,
    pub interactive_rows: c_int,
    pub interactive_display_ttl: c_int,
    pub interactive_delete_ttl: c_int,
    pub stats: c_int,
    pub onlyaddr: c_int,
    pub metric: c_int,
    pub mlat: c_int,
    pub interactive_rtl1090: c_int,
    pub fUserLat: c_double,
    pub fUserLon: c_double,
    pub bUserFlags: c_int,
    pub aircrafts: *mut aircraft,
    pub interactive_last_update: uint64_t,
    pub last_cleanup_time: time_t,
    pub bEnableDFLogging: c_int,
    pub pDF_mutex: pthread_mutex_t,
    pub pDF: *mut stDF,
    pub stat_valid_preamble: c_uint,
    pub stat_demodulated0: c_uint,
    pub stat_demodulated1: c_uint,
    pub stat_demodulated2: c_uint,
    pub stat_demodulated3: c_uint,
    pub stat_goodcrc: c_uint,
    pub stat_badcrc: c_uint,
    pub stat_fixed: c_uint,
    pub stat_bit_fix: [c_uint; 2],
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
    pub stat_ph_bit_fix: [c_uint; 2],
    pub stat_DF_Len_Corrected: c_uint,
    pub stat_DF_Type_Corrected: c_uint,
    pub stat_ModeAC: c_uint,
    pub stat_blocks_processed: c_uint,
    pub stat_blocks_dropped: c_uint,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct modesMessage {
    pub msg: [c_uchar; 14],
    pub msgbits: c_int,
    pub msgtype: c_int,
    pub crcok: c_int,
    pub crc: uint32_t,
    pub correctedbits: c_int,
    pub corrected: [c_char; 2],
    pub addr: uint32_t,
    pub phase_corrected: c_int,
    pub timestampMsg: uint64_t,
    pub remote: c_int,
    pub signalLevel: c_uchar,
    pub ca: c_int,
    pub iid: c_int,
    pub metype: c_int,
    pub mesub: c_int,
    pub heading: c_int,
    pub raw_latitude: c_int,
    pub raw_longitude: c_int,
    pub fLat: c_double,
    pub fLon: c_double,
    pub flight: [c_char; 16],
    pub ew_velocity: c_int,
    pub ns_velocity: c_int,
    pub vert_rate: c_int,
    pub velocity: c_int,
    pub fs: c_int,
    pub modeA: c_int,
    pub altitude: c_int,
    pub unit: c_int,
    pub bFlags: c_int,
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
