// Very basic re-implementation of the dump1090 binary.
// Just enough to be able to pass the tests.

use io::BufReader;

use std::fs::File;
use std::io::Read;
use std::{env, io, mem, process};

use ten_ninety::{
    compute_magnitude_vector_impl, detect_mode_s, MODES_ASYNC_BUF_SAMPLES, MODES_ASYNC_BUF_SIZE,
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
            compute_magnitude_vector_impl(buf.as_mut_ptr() as *mut u16, &mut state as *mut _);
        }

        // Process this buffer
        let mut magnitude = Vec::new();
        mem::swap(&mut state.magnitude, &mut magnitude);
        unsafe {
            detect_mode_s(
                &mut magnitude[0..MODES_ASYNC_BUF_SAMPLES],
                &mut state as *mut _,
                &bit_error_table,
            );
        }
        mem::swap(&mut state.magnitude, &mut magnitude);
    }

    Ok(())
}

fn usage() -> ! {
    eprintln!("Usage dump1090 path/to/samples.bin");
    process::exit(2)
}
