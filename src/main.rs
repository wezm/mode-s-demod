// Very basic re-implementation of the dump1090 binary.
// Just enough to be able to pass the tests.

use io::BufReader;
use std::convert::TryFrom;
use std::fs::File;
use std::io::Read;
use std::os::raw::{c_char, c_double, c_int};
use std::{env, io, process};

use ten_ninety::{
    computeMagnitudeVectorImpl, detectModeSImpl, errorinfo, modes, modesInitErrorInfoImpl,
    MODES_ASYNC_BUF_SAMPLES, MODES_ASYNC_BUF_SIZE, MODES_ICAO_CACHE_LEN, MODES_LONG_MSG_SAMPLES,
    MODES_MAX_BITERRORS, MODES_NET_SNDBUF_MAX, MODES_PREAMBLE_SAMPLES, MODES_RAWOUT_BUF_FLUSH,
    MODES_RAWOUT_BUF_RATE, MODES_RAWOUT_BUF_SIZE, MODES_USER_LATLON_VALID, NERRORINFO,
};

fn main() -> Result<(), io::Error> {
    let (mut state, bit_error_table) = modes_init();

    let ifile = match env::args().skip(1).find(|arg| !arg.starts_with("--")) {
        Some(path) => path,
        None => usage(),
    };

    // Read and decode the data from the file
    let mut file = BufReader::new(File::open(ifile)?);
    let mut buf = [0u8; MODES_ASYNC_BUF_SIZE];
    let mut toread = buf.len();
    let mut exit = false;
    while !exit {
        while toread > 0 {
            match file.read(&mut buf) {
                Ok(0) => {
                    exit = true;
                    break;
                } // EOF
                Ok(nread) => toread -= nread,
                Err(err) => return Err(err),
            }
        }

        if toread > 0 {
            // Not enough data in file to fill the buffer? Pad with no signal.
            for i in (buf.len() - toread)..buf.len() {
                buf[i] = 127;
            }
        }
        toread = buf.len();

        // Translate the next lot of I/Q samples into Modes.magnitude
        unsafe {
            computeMagnitudeVectorImpl(buf.as_mut_ptr() as *mut u16, &mut state as *mut _);
        }

        // Process this buffer
        unsafe {
            detectModeSImpl(
                state.magnitude,
                MODES_ASYNC_BUF_SAMPLES as u32,
                &mut state as *mut _,
                bit_error_table.as_ptr(),
                bit_error_table.len() as c_int,
            );
        }
    }

    Ok(())
}

fn usage() -> ! {
    eprintln!("Usage dump1090 path/to/samples.bin");
    process::exit(2)
}

fn modes_init() -> (modes, [errorinfo; NERRORINFO]) {
    let mut state = modes {
        // filename: "", // --ifile
        nfix_crc: MODES_MAX_BITERRORS as c_int, // --aggressive
        phase_enhance: 1,                       // --phase-enhance
        ..Default::default()
    };
    // struct errorinfo bitErrorTable[NERRORINFO];
    let mut bit_error_table = [errorinfo::default(); NERRORINFO];

    // pthread_mutex_init(&mut Modes.pDF_mutex,
    //                    NULL_0 as *const pthread_mutexattr_t);
    // pthread_mutex_init(&mut Modes.data_mutex,
    //                    NULL_0 as *const pthread_mutexattr_t);
    // pthread_cond_init(&mut Modes.data_cond,
    //                   NULL_0 as *const pthread_condattr_t);

    // Allocate the various buffers used by Modes
    let mut icao_cache = Box::new([0u32; MODES_ICAO_CACHE_LEN as usize * 2]);
    state.icao_cache = icao_cache.as_mut_ptr();
    Box::into_raw(icao_cache);

    // Modes.icao_cache =
    //     malloc((::std::mem::size_of::<u32>() as
    //         c_ulong).wrapping_mul(MODES_ICAO_CACHE_LEN as
    //         c_ulong).wrapping_mul(2
    //         as
    //         c_int
    //         as
    //         c_ulong))
    //         as *mut u32;
    // if Modes.icao_cache.is_null() ||
    //     {
    //         Modes.pFileData =
    //             malloc(MODES_ASYNC_BUF_SIZE as c_ulong) as
    //                 *mut u16;
    //         Modes.pFileData.is_null()
    //     } ||
    //     {
    //         Modes.magnitude =
    //             malloc((MODES_ASYNC_BUF_SIZE as
    //                 c_ulong).wrapping_add((MODES_PREAMBLE_SAMPLES
    //                 as
    //                 c_ulong).wrapping_mul(::std::mem::size_of::<u16>()
    //                 as
    //                 c_ulong)).wrapping_add((MODES_LONG_MSG_SAMPLES
    //                 as
    //                 c_ulong).wrapping_mul(::std::mem::size_of::<u16>()
    //                 as
    //                 c_ulong)))
    //                 as *mut u16;
    //         Modes.magnitude.is_null()
    let mut magnitude =
        Box::new([0u16; MODES_ASYNC_BUF_SAMPLES + MODES_PREAMBLE_SAMPLES + MODES_LONG_MSG_SAMPLES]);
    state.magnitude = magnitude.as_mut_ptr();
    Box::into_raw(magnitude);
    //     } ||
    //     {
    //         Modes.maglut =
    //             malloc((::std::mem::size_of::<u16>() as
    //                 c_ulong).wrapping_mul(256 as c_int
    //                 as
    //                 c_ulong).wrapping_mul(256
    //                 as
    //                 c_int
    //                 as
    //                 c_ulong))
    //                 as *mut u16;
    //         Modes.maglut.is_null()
    //     } ||
    //     {
    //         Modes.beastOut =
    //             malloc(MODES_RAWOUT_BUF_SIZE as c_ulong) as
    //                 *mut c_char;
    //         Modes.beastOut.is_null()
    let mut beast_out = Box::new([0 as c_char; MODES_RAWOUT_BUF_SIZE]);
    state.beastOut = beast_out.as_mut_ptr();
    Box::into_raw(beast_out);
    //     } ||
    //     {
    //         Modes.rawOut =
    //             malloc(MODES_RAWOUT_BUF_SIZE as c_ulong) as
    //                 *mut c_char;
    //         Modes.rawOut.is_null()
    let mut raw_out = Box::new([0 as c_char; MODES_RAWOUT_BUF_SIZE]);
    state.rawOut = raw_out.as_mut_ptr();
    Box::into_raw(raw_out);
    //     } {
    //     fprintf(stderr,
    //             b"Out of memory allocating data buffer.\n\x00" as *const u8 as
    //                 *const c_char);
    //     exit(1 as c_int);
    // }
    //
    // // Clear the buffers that have just been allocated, just in-case
    // memset(Modes.icao_cache as *mut c_void, 0 as c_int,
    //        (::std::mem::size_of::<u32>() as
    //            c_ulong).wrapping_mul(MODES_ICAO_CACHE_LEN as
    //            c_ulong).wrapping_mul(2
    //            as
    //            c_int
    //            as
    //            c_ulong));
    // memset(Modes.pFileData as *mut c_void, 127 as c_int,
    //        MODES_ASYNC_BUF_SIZE as c_ulong);
    // memset(Modes.magnitude as *mut c_void, 0 as c_int,
    //        (MODES_ASYNC_BUF_SIZE as
    //            c_ulong).wrapping_add((MODES_PREAMBLE_SAMPLES as
    //            c_ulong).wrapping_mul(::std::mem::size_of::<u16>()
    //            as
    //            c_ulong)).wrapping_add((MODES_LONG_MSG_SAMPLES
    //            as
    //            c_ulong).wrapping_mul(::std::mem::size_of::<u16>()
    //            as
    //            c_ulong)));

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

    // // Initialise the Block Timers to something half sensible
    // ftime(&mut Modes.stSystemTimeBlk);
    // i = 0 as c_int;
    // while i < MODES_ASYNC_BUF_NUMBER {
    //     Modes.stSystemTimeRTL[i as usize] = Modes.stSystemTimeBlk;
    //     i += 1
    // }
    //
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
