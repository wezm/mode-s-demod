use std::os::raw::{c_int, c_uchar, c_uint};

use crate::modesMessage;

// ===================== Mode A/C detection and decoding  ===================

pub const MODEAC_MSG_SAMPLES: u32 = 25 * 2; // include up to the SPI bit
                                            // const MODEAC_MSG_BYTES: c_int = 2;
const MODEAC_MSG_SQUELCH_LEVEL: c_int = 0x07FF; // Average signal strength limit
                                                // const MODEAC_MSG_FLAG: c_int = 1 << 0;
                                                // const MODEAC_MSG_MODES_HIT: c_int = 1 << 1;
                                                // const MODEAC_MSG_MODEA_HIT: c_int = 1 << 2;
                                                // const MODEAC_MSG_MODEC_HIT: c_int = 1 << 3;
                                                // const MODEAC_MSG_MODEA_ONLY: c_int = 1 << 4;
                                                // const MODEAC_MSG_MODEC_OLD: c_int = 1 << 5;

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
#[no_mangle]
pub static mut ModeABitTable: [u32; 24] = [
    0, 0x10, 0x1000, 0x20, 0x2000, 0x40, 0x4000, 0x40000000, 0x100, 0x1, 0x200, 0x2, 0x400, 0x4, 0,
    0x8000000, 0x4000000, 0x80, 0x2000000, 0x1000000, 0x800000, 0x400000, 0x200000, 0x100000,
];

// This table is used to produce an error variable called ModeAErrs.Each
// inter-bit period is inspected, and if it's value falls outside of the
// expected range, then the value in this table is or-ed into ModeAErrs.
//
// At the end of message processing, ModeAErrs will indicate if we saw
// any inter-bit anomolies, and the bits that are set will show which
// bits had them.
//
#[no_mangle]
pub static mut ModeAMidTable: [u32; 24] = [
    0x80000000, 0x10, 0x1000, 0x20, 0x2000, 0x40, 0x4000, 0x40000000, 0x100, 0x1, 0x200, 0x2,
    0x400, 0x4, 0x20000000, 0x8000000, 0x4000000, 0x80, 0x2000000, 0x1000000, 0x800000, 0x400000,
    0x200000, 0x100000,
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
#[no_mangle]
pub unsafe extern "C" fn detectModeA(m: *mut u16, mm: *mut modesMessage) -> c_int {
    let mut ModeABits = 0 as c_int;
    let mut ModeAErrs = 0 as c_int;
    let mut bit: c_int;
    let mut lastSpace = 0 as c_int;
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
    let m0 = *m.offset(0 as c_int as isize) as c_int;
    let m1 = *m.offset(1 as c_int as isize) as c_int;
    if m0 >= m1 {
        // m1 *must* be bigger than m0 for this to be F1
        return 0 as c_int;
    }
    let mut m2 = *m.offset(2 as c_int as isize) as c_int;
    let mut m3 = *m.offset(3 as c_int as isize) as c_int;
    //
    // if (m2 <= m0), then assume the sample bob on (Phase == 0), so don't look at m3
    if m2 <= m0 || m2 < m3 {
        m3 = m2;
        m2 = m0
    }
    if m3 >= m1 || m0 > m2 || m3 > m2 {
        // m2 can be equal to m3 if ( 0,1,0,0 )
        return 0 as c_int;
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
    let F1_sig = m1 + m2 - (m0 + m3 << 1 as c_int);
    let F1_noise = m0 + m3 >> 1 as c_int;
    if F1_sig < MODEAC_MSG_SQUELCH_LEVEL || F1_sig < F1_noise << 2 as c_int {
        // minimum allowable Sig/Noise ratio 4:1
        return 0 as c_int;
    }
    // If we get here then we have a potential F1, so look for an equally valid F2 20.3uS later
    //
    // Our F1 is centered somewhere between samples m[1] and m[2]. We can guestimate where F2 is
    // by comparing the ratio of m1 and m2, and adding on 20.3 uS (40.6 samples)
    //
    let mut mPhase = m2 * 20 as c_int / (m1 + m2);
    let mut byte = (mPhase + 812 as c_int) / 20 as c_int;
    let fresh0 = byte;
    byte = byte + 1;
    let n0 = *m.offset(fresh0 as isize) as c_int;
    let fresh1 = byte;
    byte = byte + 1;
    let n1 = *m.offset(fresh1 as isize) as c_int;
    if n0 >= n1 {
        // n1 *must* be bigger than n0 for this to be F2
        return 0 as c_int;
    }
    let fresh2 = byte;
    byte = byte + 1;
    let mut n2 = *m.offset(fresh2 as isize) as c_int;
    //
    // if the sample bob on (Phase == 0), don't look at n3
    //
    if (mPhase + 812 as c_int) % 20 as c_int != 0 {
        let fresh3 = byte;
        byte = byte + 1;
        n3 = *m.offset(fresh3 as isize) as c_int
    } else {
        n3 = n2;
        n2 = n0
    }
    if n3 >= n1 || n0 > n2 || n3 > n2 {
        // n2 can be equal to n3 ( 0,1,0,0 )
        return 0 as c_int;
    }
    let F2_sig = n1 + n2 - (n0 + n3 << 1 as c_int);
    let F2_noise = n0 + n3 >> 1 as c_int;
    if F2_sig < MODEAC_MSG_SQUELCH_LEVEL || F2_sig < F2_noise << 2 as c_int {
        // maximum allowable Sig/Noise ratio 4:1
        return 0 as c_int;
    } // 1/2
    let mut fSig = F1_sig + F2_sig >> 1 as c_int;
    let fNoise = F1_noise + F2_noise >> 1 as c_int;
    let fLoLo = fNoise + (fSig >> 2 as c_int);
    let fLevel = fNoise + (fSig >> 1 as c_int);
    let mut lastBitWasOne = 1 as c_int;
    let mut lastBit = F1_sig;
    //
    // Now step by a half ModeA bit, 0.725nS, which is 1.45 samples, which is 29/20
    // No need to do bit 0 because we've already selected it as a valid F1
    // Do several bits past the SPI to increase error rejection
    //
    let mut j = 1 as c_int; //    add in the second sample's energy
    mPhase += 29 as c_int;
    while j < 48 as c_int {
        byte = 1 as c_int + mPhase / 20 as c_int;
        let mut thisSample = *m.offset(byte as isize) as c_int - fNoise;
        if mPhase % 20 as c_int != 0 {
            // If the bit is split over two samples...
            thisSample += *m.offset((byte + 1 as c_int) as isize) as c_int - fNoise
        }
        // If we're calculating a space value
        if j & 1 as c_int != 0 {
            lastSpace = thisSample
        } else {
            // We're calculating a new bit value
            bit = j >> 1 as c_int;
            if thisSample >= fLevel {
                // We're calculating a new bit value, and its a one
                let fresh4 = bit; // or in the correct bit
                bit = bit - 1;
                ModeABits = (ModeABits as c_uint | ModeABitTable[fresh4 as usize]) as c_int;
                if lastBitWasOne != 0 {
                    // This bit is one, last bit was one, so check the last space is somewhere less than one
                    if lastSpace >= thisSample >> 1 as c_int || lastSpace >= lastBit {
                        ModeAErrs = (ModeAErrs as c_uint | ModeAMidTable[bit as usize]) as c_int
                    }
                } else if lastSpace >= thisSample >> 1 as c_int {
                    ModeAErrs = (ModeAErrs as c_uint | ModeAMidTable[bit as usize]) as c_int
                }
                lastBitWasOne = 1 as c_int
            } else {
                // This bit,is one, last bit was zero, so check the last space is somewhere less than one
                // We're calculating a new bit value, and its a zero
                if lastBitWasOne != 0 {
                    // This bit is zero, last bit was one, so check the last space is somewhere in between
                    if lastSpace >= lastBit {
                        ModeAErrs = (ModeAErrs as c_uint | ModeAMidTable[bit as usize]) as c_int
                    }
                } else if lastSpace >= fLoLo {
                    ModeAErrs = (ModeAErrs as c_uint | ModeAMidTable[bit as usize]) as c_int
                }
                lastBitWasOne = 0 as c_int
            }
            lastBit = thisSample >> 1 as c_int
        }
        mPhase += 29 as c_int;
        j += 1
    }
    // This bit,is zero, last bit was zero, so check the last space is zero too
    //
    // Output format is : 00:A4:A2:A1:00:B4:B2:B1:00:C4:C2:C1:00:D4:D2:D1
    //
    if ModeABits < 3 as c_int || ModeABits as c_uint & 0xffff8808 as c_uint != 0 || ModeAErrs != 0 {
        ModeABits = 0 as c_int;
        return ModeABits;
    }
    fSig = fSig + 0x7f as c_int >> 8 as c_int;
    (*mm).signalLevel = if fSig < 255 as c_int {
        fSig
    } else {
        255 as c_int
    } as c_uchar;
    return ModeABits;
}

// Input format is : 00:A4:A2:A1:00:B4:B2:B1:00:C4:C2:C1:00:D4:D2:D1
//
#[rustfmt::skip]
#[no_mangle]
pub extern "C" fn ModeAToModeC(ModeA: c_uint) -> c_int {
    let mut FiveHundreds: c_uint = 0;
    let mut OneHundreds: c_uint = 0;

    if ModeA & 0xffff888b != 0 // D1 set is illegal. D2 set is > 62700ft which is unlikely
       || ModeA & 0xf0 == 0 // C1,,C4 cannot be Zero
    {
        // TODO: Change return type to Option to capture this case and the one below
        return -9999;
    }

    if ModeA & 0x10 != 0 { OneHundreds ^= 0x7 } // C1
    if ModeA & 0x20 != 0 { OneHundreds ^= 0x3 } // C2
    if ModeA & 0x40 != 0 { OneHundreds ^= 0x1 } // C4

    // Remove 7s from OneHundreds (Make 7->5, and 5->7).
    if OneHundreds & 5 == 5 { OneHundreds ^= 2 }

    // Check for invalid codes, only 1 to 5 are valid
    if OneHundreds > 5 { return -9999; }

    // if (ModeA & 0x0001) {FiveHundreds ^= 0x1FF;} // D1 never used for altitude
    if ModeA & 0x2 != 0 { FiveHundreds ^= 0xff }    // D2
    if ModeA & 0x4 != 0 { FiveHundreds ^= 0x7f }    // D4
    if ModeA & 0x1000 != 0 { FiveHundreds ^= 0x3f } // A1
    if ModeA & 0x2000 != 0 { FiveHundreds ^= 0x1f } // A2
    if ModeA & 0x4000 != 0 { FiveHundreds ^= 0xf }  // A4
    if ModeA & 0x100 != 0 { FiveHundreds ^= 0x7 }   // B1
    if ModeA & 0x200 != 0 { FiveHundreds ^= 0x3 }   // B2
    if ModeA & 0x400 != 0 { FiveHundreds ^= 0x1 }   // B4

    // Correct order of OneHundreds.
    if FiveHundreds & 1 != 0 {
        OneHundreds = (6 as c_uint).wrapping_sub(OneHundreds)
    }

    return FiveHundreds
        .wrapping_mul(5)
        .wrapping_add(OneHundreds)
        .wrapping_sub(13) as c_int;
}
