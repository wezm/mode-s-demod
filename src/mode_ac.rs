use std::os::raw::{c_int, c_uchar, c_uint};

use crate::ModesMessage;

// ===================== Mode A/C detection and decoding  ===================

pub const MODEAC_MSG_SAMPLES: u32 = 25 * 2; // include up to the SPI bit
                                            // const MODEAC_MSG_BYTES: c_int = 2;
const MODES_ACFLAGS_SQUAWK_VALID: c_int = 1 << 5;
const MODEAC_MSG_SQUELCH_LEVEL: c_int = 0x07FF; // Average signal strength limit
pub(crate) const MODEAC_MSG_FLAG: c_int = 1 << 0;
// const MODEAC_MSG_MODES_HIT: c_int = 1 << 1;
// const MODEAC_MSG_MODEA_HIT: c_int = 1 << 2;
// const MODEAC_MSG_MODEC_HIT: c_int = 1 << 3;
pub const MODEAC_MSG_MODEA_ONLY: c_int = 1 << 4;
pub const MODEAC_MSG_MODEC_OLD: c_int = 1 << 5;

// This table is used to build the Mode A/C variable called ModeABits.Each
// bit period is inspected, and if it's value exceeds the threshold limit,
// then the value in this table is or-ed into ModeABits.
//
// At the end of message processing, ModeABits will be the decoded ModeA value.
//
// We can also flag noise in bits that should be zeros - the xx bits. Noise in
// these bits cause bits (31-16) in ModeABits to be set. Then at the end of message
// processing we can test for errors by looking at these bits.
//
pub static mut MODE_ABIT_TABLE: [u32; 24] = [
    0x00000000, // F1 = 1
    0x00000010, // C1
    0x00001000, // A1
    0x00000020, // C2
    0x00002000, // A2
    0x00000040, // C4
    0x00004000, // A4
    0x40000000, // xx = 0  Set bit 30 if we see this high
    0x00000100, // B1
    0x00000001, // D1
    0x00000200, // B2
    0x00000002, // D2
    0x00000400, // B4
    0x00000004, // D4
    0x00000000, // F2 = 1
    0x08000000, // xx = 0  Set bit 27 if we see this high
    0x04000000, // xx = 0  Set bit 26 if we see this high
    0x00000080, // SPI
    0x02000000, // xx = 0  Set bit 25 if we see this high
    0x01000000, // xx = 0  Set bit 24 if we see this high
    0x00800000, // xx = 0  Set bit 23 if we see this high
    0x00400000, // xx = 0  Set bit 22 if we see this high
    0x00200000, // xx = 0  Set bit 21 if we see this high
    0x00100000, // xx = 0  Set bit 20 if we see this high
];

// This table is used to produce an error variable called ModeAErrs.Each
// inter-bit period is inspected, and if it's value falls outside of the
// expected range, then the value in this table is or-ed into ModeAErrs.
//
// At the end of message processing, ModeAErrs will indicate if we saw
// any inter-bit anomalies, and the bits that are set will show which
// bits had them.
//
pub static mut MODE_AMID_TABLE: [u32; 24] = [
    0x80000000, // F1 = 1  Set bit 31 if we see F1_C1  error
    0x00000010, // C1      Set bit  4 if we see C1_A1  error
    0x00001000, // A1      Set bit 12 if we see A1_C2  error
    0x00000020, // C2      Set bit  5 if we see C2_A2  error
    0x00002000, // A2      Set bit 13 if we see A2_C4  error
    0x00000040, // C4      Set bit  6 if we see C3_A4  error
    0x00004000, // A4      Set bit 14 if we see A4_xx  error
    0x40000000, // xx = 0  Set bit 30 if we see xx_B1  error
    0x00000100, // B1      Set bit  8 if we see B1_D1  error
    0x00000001, // D1      Set bit  0 if we see D1_B2  error
    0x00000200, // B2      Set bit  9 if we see B2_D2  error
    0x00000002, // D2      Set bit  1 if we see D2_B4  error
    0x00000400, // B4      Set bit 10 if we see B4_D4  error
    0x00000004, // D4      Set bit  2 if we see D4_F2  error
    0x20000000, // F2 = 1  Set bit 29 if we see F2_xx  error
    0x08000000, // xx = 0  Set bit 27 if we see xx_xx  error
    0x04000000, // xx = 0  Set bit 26 if we see xx_SPI error
    0x00000080, // SPI     Set bit 15 if we see SPI_xx error
    0x02000000, // xx = 0  Set bit 25 if we see xx_xx  error
    0x01000000, // xx = 0  Set bit 24 if we see xx_xx  error
    0x00800000, // xx = 0  Set bit 23 if we see xx_xx  error
    0x00400000, // xx = 0  Set bit 22 if we see xx_xx  error
    0x00200000, // xx = 0  Set bit 21 if we see xx_xx  error
    0x00100000, // xx = 0  Set bit 20 if we see xx_xx  error
];

// The "off air" format is,,
// _F1_C1_A1_C2_A2_C4_A4_xx_B1_D1_B2_D2_B4_D4_F2_xx_xx_SPI_
//
// Bit spacing is 1.45uS, with 0.45uS high, and 1.00us low. This is a problem
// because we ase sampling at 2Mhz (500nS) so we are below Nyquist.
//
// The bit spacings are..
// F1 :  0.00,
//       1.45,  2.90,  4.35,  5.80,  7.25,  8.70,
// X  : 10.15,
//    : 11.60, 13.05, 14.50, 15.95, 17.40, 18.85,
// F2 : 20.30,
// X  : 21.75, 23.20, 24.65
//
// This equates to the following sample point centers at 2Mhz.
// [ 0.0],
// [ 2.9], [ 5.8], [ 8.7], [11.6], [14.5], [17.4],
// [20.3],
// [23.2], [26.1], [29.0], [31.9], [34.8], [37.7]
// [40.6]
// [43.5], [46.4], [49.3]
//
// We know that this is a supposed to be a binary stream, so the signal
// should either be a 1 or a 0. Therefore, any energy above the noise level
// in two adjacent samples must be from the same pulse, so we can simply
// add the values together..
//
#[rustfmt::skip]
pub(crate) unsafe fn detect_mode_a(m: *mut u16, mm: *mut ModesMessage) -> c_int {
    let mut mode_a_bits = 0;
    let mut mode_a_errs = 0;
    let mut bit: c_int;
    let mut last_space = 0;
    let n3: c_int;

    // m[0] contains the energy from    0 ->  499 nS
    // m[1] contains the energy from  500 ->  999 nS
    // m[2] contains the energy from 1000 -> 1499 nS
    // m[3] contains the energy from 1500 -> 1999 nS
    //
    // We are looking for a Frame bit (F1) whose width is 450nS, followed by
    // 1000nS of quiet.
    //
    // The width of the frame bit is 450nS, which is 90% of our sample rate.
    // Therefore, in an ideal world, all the energy for the frame bit will be
    // in a single sample, preceded by (at least) one zero, and followed by
    // two zeros, Best case we can look for ...
    //
    // 0 - 1 - 0 - 0
    //
    // However, our samples are not phase aligned, so some of the energy from
    // each bit could be spread over two consecutive samples. Worst case is
    // that we sample half in one bit, and half in the next. In that case,
    // we're looking for
    //
    // 0 - 0.5 - 0.5 - 0.

    let m0 = *m.offset(0) as c_int;
    let m1 = *m.offset(1) as c_int;

    if m0 >= m1 {
        // m1 *must* be bigger than m0 for this to be F1
        return 0;
    }

    let mut m2 = *m.offset(2) as c_int;
    let mut m3 = *m.offset(3) as c_int;

    // if (m2 <= m0), then assume the sample bob on (Phase == 0), so don't look at m3
    if m2 <= m0 || m2 < m3 {
        m3 = m2;
        m2 = m0
    }

    // #[rustfmt::skip]
    if m3 >= m1     // m1 must be bigger than m3
        || m0 > m2 // m2 can be equal to m0 if ( 0,1,0,0 )
        || m3 > m2 // m2 can be equal to m3 if ( 0,1,0,0 )
    {
        return 0;
    }

    // m0 = noise
    // m1 = noise + (signal *    X))
    // m2 = noise + (signal * (1-X))
    // m3 = noise
    //
    // Hence, assuming all 4 samples have similar amounts of noise in them
    //      signal = (m1 + m2) - ((m0 + m3) * 2)
    //      noise  = (m0 + m3) / 2
    //
    let f1_sig = m1 + m2 - (m0 + m3 << 1);
    let f1_noise = m0 + m3 >> 1;

    // #[rustfmt::skip]
    if f1_sig < MODEAC_MSG_SQUELCH_LEVEL // minimum required  F1 signal amplitude
        || f1_sig < f1_noise << 2        // minimum allowable Sig/Noise ratio 4:1
    {
        return 0;
    }

    // If we get here then we have a potential F1, so look for an equally valid F2 20.3uS later
    //
    // Our F1 is centered somewhere between samples m[1] and m[2]. We can guesstimate where F2 is
    // by comparing the ratio of m1 and m2, and adding on 20.3 uS (40.6 samples)
    //
    let mut m_phase = m2 * 20 / (m1 + m2);
    let mut byte = (m_phase + 812) / 20;
    let fresh0 = byte;
    byte = byte + 1;
    let n0 = *m.offset(fresh0 as isize) as c_int;
    let fresh1 = byte;
    byte = byte + 1;
    let n1 = *m.offset(fresh1 as isize) as c_int;

    if n0 >= n1 {
        // n1 *must* be bigger than n0 for this to be F2
        return 0;
    }

    let fresh2 = byte;
    byte = byte + 1;
    let mut n2 = *m.offset(fresh2 as isize) as c_int;
    //
    // if the sample bob on (Phase == 0), don't look at n3
    //
    if (m_phase + 812) % 20 != 0 {
        let fresh3 = byte;
        byte = byte + 1;
        n3 = *m.offset(fresh3 as isize) as c_int
    } else {
        n3 = n2;
        n2 = n0
    }

    // #[rustfmt::skip]
    if n3 >= n1    // n1 must be bigger than n3
        || n0 > n2 // n2 can be equal to n0 ( 0,1,0,0 )
        || n3 > n2 // n2 can be equal to n3 ( 0,1,0,0 )
    {
        return 0;
    }

    let f2_sig = n1 + n2 - (n0 + n3 << 1);
    let f2_noise = n0 + n3 >> 1;

    // #[rustfmt::skip]
    if f2_sig < MODEAC_MSG_SQUELCH_LEVEL // minimum required  F2 signal amplitude
        || f2_sig < f2_noise << 2 // maximum allowable Sig/Noise ratio 4:1
    {
        return 0;
    }

    let mut f_sig = f1_sig + f2_sig >> 1;
    let f_noise = f1_noise + f2_noise >> 1;
    let f_lo_lo = f_noise + (f_sig >> 2); // 1/2
    let f_level = f_noise + (f_sig >> 1);
    let mut last_bit_was_one = 1;
    let mut last_bit = f1_sig;
    //
    // Now step by a half ModeA bit, 0.725nS, which is 1.45 samples, which is 29/20
    // No need to do bit 0 because we've already selected it as a valid F1
    // Do several bits past the SPI to increase error rejection
    //
    let mut j = 1; //    add in the second sample's energy
    m_phase += 29;
    while j < 48 as c_int {
        byte = 1 + m_phase / 20;

        let mut this_sample = *m.offset(byte as isize) as c_int - f_noise;
        // If the bit is split over two samples...
        if m_phase % 20 != 0 {
            // add in the second sample's energy
            this_sample += *m.offset((byte + 1) as isize) as c_int - f_noise
        }

        // If we're calculating a space value
        if j & 1 != 0 {
            last_space = this_sample
        } else {
            // We're calculating a new bit value
            bit = j >> 1;
            if this_sample >= f_level {
                // We're calculating a new bit value, and its a one
                let fresh4 = bit; // or in the correct bit
                bit = bit - 1;
                mode_a_bits = (mode_a_bits as c_uint | MODE_ABIT_TABLE[fresh4 as usize]) as c_int;

                if last_bit_was_one != 0 {
                    // This bit is one, last bit was one, so check the last space is somewhere less than one
                    if last_space >= this_sample >> 1 || last_space >= last_bit {
                        mode_a_errs = (mode_a_errs as c_uint | MODE_AMID_TABLE[bit as usize]) as c_int
                    }
                } else if last_space >= this_sample >> 1 {
                    // This bit,is one, last bit was zero, so check the last space is somewhere less than one
                    mode_a_errs = (mode_a_errs as c_uint | MODE_AMID_TABLE[bit as usize]) as c_int
                }

                last_bit_was_one = 1;
            } else {
                // We're calculating a new bit value, and its a zero
                if last_bit_was_one != 0 {
                    // This bit is zero, last bit was one, so check the last space is somewhere in between
                    if last_space >= last_bit {
                        mode_a_errs = (mode_a_errs as c_uint | MODE_AMID_TABLE[bit as usize]) as c_int
                    }
                } else if last_space >= f_lo_lo {
                    // This bit,is zero, last bit was zero, so check the last space is zero too
                    mode_a_errs = (mode_a_errs as c_uint | MODE_AMID_TABLE[bit as usize]) as c_int
                }

                last_bit_was_one = 0;
            }

            last_bit = this_sample >> 1;
        }

        m_phase += 29;
        j += 1
    }

    //
    // Output format is : 00:A4:A2:A1:00:B4:B2:B1:00:C4:C2:C1:00:D4:D2:D1
    //
    if mode_a_bits < 3 || mode_a_bits as c_uint & 0xffff8808 as c_uint != 0 || mode_a_errs != 0 {
        mode_a_bits = 0;
        return mode_a_bits;
    }

    f_sig = f_sig + 0x7f >> 8;
    (*mm).signal_level = if f_sig < 255 { f_sig } else { 255 } as c_uchar;

    return mode_a_bits;
}

// Input format is : 00:A4:A2:A1:00:B4:B2:B1:00:C4:C2:C1:00:D4:D2:D1
//
#[rustfmt::skip]
pub(crate) fn mode_a_to_mode_c(mode_a: c_uint) -> c_int {
    let mut five_hundreds: c_uint = 0;
    let mut one_hundreds: c_uint = 0;

    if mode_a & 0xffff888b != 0 // D1 set is illegal. D2 set is > 62700ft which is unlikely
       || mode_a & 0xf0 == 0 // C1,,C4 cannot be Zero
    {
        // TODO: Change return type to Option to capture this case and the one below
        return -9999;
    }

    if mode_a & 0x10 != 0 { one_hundreds ^= 0x7 } // C1
    if mode_a & 0x20 != 0 { one_hundreds ^= 0x3 } // C2
    if mode_a & 0x40 != 0 { one_hundreds ^= 0x1 } // C4

    // Remove 7s from one_hundreds (Make 7->5, and 5->7).
    if one_hundreds & 5 == 5 { one_hundreds ^= 2 }

    // Check for invalid codes, only 1 to 5 are valid
    if one_hundreds > 5 { return -9999; }

    // if (mode_a & 0x0001) {five_hundreds ^= 0x1FF;} // D1 never used for altitude
    if mode_a & 0x2 != 0 { five_hundreds ^= 0xff }    // D2
    if mode_a & 0x4 != 0 { five_hundreds ^= 0x7f }    // D4
    if mode_a & 0x1000 != 0 { five_hundreds ^= 0x3f } // A1
    if mode_a & 0x2000 != 0 { five_hundreds ^= 0x1f } // A2
    if mode_a & 0x4000 != 0 { five_hundreds ^= 0xf }  // A4
    if mode_a & 0x100 != 0 { five_hundreds ^= 0x7 }   // B1
    if mode_a & 0x200 != 0 { five_hundreds ^= 0x3 }   // B2
    if mode_a & 0x400 != 0 { five_hundreds ^= 0x1 }   // B4

    // Correct order of one_hundreds.
    if five_hundreds & 1 != 0 {
        one_hundreds = (6 as c_uint).wrapping_sub(one_hundreds)
    }

    return five_hundreds
        .wrapping_mul(5)
        .wrapping_add(one_hundreds)
        .wrapping_sub(13) as c_int;
}

pub(crate) unsafe fn decode_mode_a_message(mm: *mut ModesMessage, mode_a: c_int) {
    (*mm).msgtype = 32; // Valid Mode S DF's are DF-00 to DF-31.
                        // so use 32 to indicate Mode A/C

    (*mm).msgbits = 16; // Fudge up a Mode S style data stream
    (*mm).msg[0] = (mode_a >> 8) as c_uchar;
    (*mm).msg[1] = mode_a as c_uchar;

    // Fudge an ICAO address based on Mode A (remove the Ident bit)
    // Use an upper address byte of FF, since this is ICAO unallocated
    (*mm).addr = (0xff0000 | mode_a & 0xff7f) as u32;

    // Set the Identity field to mode_a
    (*mm).mode_a = mode_a & 0x7777;
    (*mm).b_flags |= MODES_ACFLAGS_SQUAWK_VALID;

    // Flag ident in flight status
    (*mm).fs = mode_a & 0x80;

    // Not much else we can tell from a Mode A/C reply.
    // Just fudge up a few bits to keep other code happy
    (*mm).crcok = 1;
    (*mm).correctedbits = 0;
}
