use std::os::raw::{c_int, c_uint};

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
