// Very basic re-implementation of the dump1090 binary.
// Just enough to be able to pass the tests.

use io::BufReader;

use std::fs::File;
use std::io::Read;
use std::os::raw::c_int;
use std::{env, io, process};

use ten_ninety::{
    computeMagnitudeVectorImpl, detectModeSImpl, MODES_ASYNC_BUF_SAMPLES, MODES_ASYNC_BUF_SIZE,
};

fn main() -> Result<(), io::Error> {
    let (mut state, bit_error_table) = ten_ninety::modes_init();

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
