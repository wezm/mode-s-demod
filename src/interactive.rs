use std::convert::TryInto;
use std::os::raw::{c_char, c_int, c_long, c_uint, c_ulong, c_void};
use std::time::SystemTime;
use std::{mem, ptr, time};

use crate::mode_ac::{ModeAToModeC, MODEAC_MSG_FLAG, MODEAC_MSG_MODEA_ONLY, MODEAC_MSG_MODEC_OLD};
use crate::mode_s::{decodeCPR, decodeCPRrelative};
use crate::{
    aircraft, modes, modesMessage, time_t, MODES_ACFLAGS_ALTITUDE_VALID, MODES_ACFLAGS_AOG,
    MODES_ACFLAGS_AOG_VALID, MODES_ACFLAGS_CALLSIGN_VALID, MODES_ACFLAGS_HEADING_VALID,
    MODES_ACFLAGS_LATLON_VALID, MODES_ACFLAGS_LLBOTH_VALID, MODES_ACFLAGS_LLEITHER_VALID,
    MODES_ACFLAGS_LLODD_VALID, MODES_ACFLAGS_SPEED_VALID, MODES_ACFLAGS_SQUAWK_VALID,
    MODES_ACFLAGS_VERTRATE_VALID,
};

const MODEAC_MSG_MODEA_HIT: c_int = (1 as c_int) << 2 as c_int;
const MODEAC_MSG_MODEC_HIT: c_int = (1 as c_int) << 3 as c_int;

// Receive new messages and populate the interactive mode with more info
//
pub(crate) unsafe fn interactiveReceiveData(Modes: &mut modes, mm: *mut modesMessage) -> *mut aircraft {
    let mut a = 0 as *mut aircraft;
    let mut aux = 0 as *mut aircraft;

    // Return if (checking crc) AND (not crcok) AND (not fixed)
    if Modes.check_crc != 0 && (*mm).crcok == 0 as c_int && (*mm).correctedbits == 0 as c_int {
        return ptr::null_mut();
    }

    // Lookup our aircraft or create a new one
    a = interactiveFindAircraft(Modes, (*mm).addr);
    if a.is_null() {
        // If it's a currently unknown aircraft....
        a = interactiveCreateAircraft(mm); // ., create a new record for it,
        (*a).next = Modes.aircrafts; // .. and put it at the head of the list
        Modes.aircrafts = a
    } else if 0 as c_int != 0 && Modes.aircrafts != a &&
        // FIXME: This was disabled (via if 0 in the C code)
        crate::now() as i64 - (*a).seen >= 1
    {
        aux = Modes.aircrafts;
        while (*aux).next != a {
            aux = (*aux).next
        }
        /* If it is an already known aircraft, move it on head
         * so we keep aircrafts ordered by received message time.
         *
         * However move it on head only if at least one second elapsed
         * since the aircraft that is currently on head sent a message,
         * otherwise with multiple aircrafts at the same time we have an
         * useless shuffle of positions on the screen. */
        /* Now we are a node before the aircraft to remove. */
        (*aux).next = (*(*aux).next).next; /* removed. */
        /* Add on head */
        (*a).next = Modes.aircrafts;
        Modes.aircrafts = a
    }

    (*a).signalLevel[((*a).messages & 7 as c_int as c_long) as usize] = (*mm).signalLevel; // replace the 8th oldest signal strength
    (*a).seen = crate::now() as i64;
    (*a).timestamp = (*mm).timestampMsg;
    (*a).messages += 1;

    // If a (new) CALLSIGN has been received, copy it to the aircraft structure
    if (*mm).bFlags & MODES_ACFLAGS_CALLSIGN_VALID != 0 {
        ptr::copy_nonoverlapping(
            (*mm).flight.as_mut_ptr() as *const c_void,
            (*a).flight.as_mut_ptr() as *mut c_void,
            mem::size_of::<[c_char; 16]>(),
        );
    }

    // If a (new) ALTITUDE has been received, copy it to the aircraft structure
    if (*mm).bFlags & MODES_ACFLAGS_ALTITUDE_VALID != 0 {
        if (*a).modeCcount != 0 && (*a).altitude != (*mm).altitude {
            // and Altitude has changed
            //        && (a->modeC     != mm->modeC + 1)   // and Altitude not changed by +100 feet
            //        && (a->modeC + 1 != mm->modeC    ) ) // and Altitude not changes by -100 feet
            (*a).modeCcount = 0 as c_int as c_long; //....zero the hit count
            (*a).modeACflags &= !MODEAC_MSG_MODEC_HIT
        }
        (*a).altitude = (*mm).altitude;
        (*a).modeC = ((*mm).altitude + 49) / 100
    }

    // If a (new) SQUAWK has been received, copy it to the aircraft structure
    if (*mm).bFlags & MODES_ACFLAGS_SQUAWK_VALID != 0 {
        if (*a).modeA != (*mm).modeA {
            (*a).modeAcount = 0; // Squawk has changed, so zero the hit count
            (*a).modeACflags &= !MODEAC_MSG_MODEA_HIT
        }
        (*a).modeA = (*mm).modeA
    }

    // If a (new) HEADING has been received, copy it to the aircraft structure
    if (*mm).bFlags & MODES_ACFLAGS_HEADING_VALID != 0 {
        (*a).track = (*mm).heading
    }

    // If a (new) SPEED has been received, copy it to the aircraft structure
    if (*mm).bFlags & MODES_ACFLAGS_SPEED_VALID != 0 {
        (*a).speed = (*mm).velocity
    }

    // If a (new) Vertical Descent rate has been received, copy it to the aircraft structure
    if (*mm).bFlags & MODES_ACFLAGS_VERTRATE_VALID != 0 {
        (*a).vert_rate = (*mm).vert_rate
    }

    // if the Aircraft has landed or taken off since the last message, clear the even/odd CPR flags
    if (*mm).bFlags & MODES_ACFLAGS_AOG_VALID != 0
        && ((*a).bFlags ^ (*mm).bFlags) & MODES_ACFLAGS_AOG != 0
    {
        (*a).bFlags &= !(MODES_ACFLAGS_LLBOTH_VALID | MODES_ACFLAGS_AOG)
    }

    // If we've got a new cprlat or cprlon
    if (*mm).bFlags & MODES_ACFLAGS_LLEITHER_VALID != 0 {
        let mut location_ok = 0;

        if (*mm).bFlags & MODES_ACFLAGS_LLODD_VALID != 0 {
            (*a).odd_cprlat = (*mm).raw_latitude;
            (*a).odd_cprlon = (*mm).raw_longitude;
            (*a).odd_cprtime = mstime()
        } else {
            (*a).even_cprlat = (*mm).raw_latitude;
            (*a).even_cprlon = (*mm).raw_longitude;
            (*a).even_cprtime = mstime()
        }

        // If we have enough recent data, try global CPR
        if ((*mm).bFlags | (*a).bFlags) & MODES_ACFLAGS_LLEITHER_VALID == MODES_ACFLAGS_LLBOTH_VALID
            && ((*a).even_cprtime.wrapping_sub((*a).odd_cprtime) as c_int).abs() <= 10000
        {
            if decodeCPR(
                &Modes,
                a,
                (*mm).bFlags & MODES_ACFLAGS_LLODD_VALID,
                (*mm).bFlags & MODES_ACFLAGS_AOG,
            ) == 0
            {
                location_ok = 1
            }
        }

        // Otherwise try relative CPR.
        if location_ok == 0
            && decodeCPRrelative(
                &Modes,
                a,
                (*mm).bFlags & MODES_ACFLAGS_LLODD_VALID,
                (*mm).bFlags & MODES_ACFLAGS_AOG,
            ) == 0
        {
            location_ok = 1
        }

        // If we sucessfully decoded, back copy the results to mm so that we can print them in list output
        if location_ok != 0 {
            (*mm).bFlags |= MODES_ACFLAGS_LATLON_VALID;
            (*mm).fLat = (*a).lat;
            (*mm).fLon = (*a).lon
        }
    }

    // Update the aircrafts a->bFlags to reflect the newly received mm->bFlags;
    (*a).bFlags |= (*mm).bFlags;
    if (*mm).msgtype == 32 {
        let mut flags = (*a).modeACflags;
        if flags & (MODEAC_MSG_MODEC_HIT | MODEAC_MSG_MODEC_OLD) == MODEAC_MSG_MODEC_OLD {
            //
            // This Mode-C doesn't currently hit any known Mode-S, but it used to because MODEAC_MSG_MODEC_OLD is
            // set  So the aircraft it used to match has either changed altitude, or gone out of our receiver range
            //
            // We've now received this Mode-A/C again, so it must be a new aircraft. It could be another aircraft
            // at the same Mode-C altitude, or it could be a new airctraft with a new Mods-A squawk.
            //
            // To avoid masking this aircraft from the interactive display, clear the MODEAC_MSG_MODES_OLD flag
            // and set messages to 1;
            //
            (*a).modeACflags = flags & !MODEAC_MSG_MODEC_OLD;
            (*a).messages = 1
        }
    }

    // If we are Logging DF's, and it's not a Mode A/C
    if Modes.bEnableDFLogging != 0 && (*mm).msgtype < 32 {
        // FIXME; port this if needed
        // interactiveCreateDF(a, mm);
    }
    return a;
}

// Return a new aircraft structure for the interactive mode linked list
// of aircraft
//
unsafe fn interactiveCreateAircraft(mut mm: *mut modesMessage) -> *mut aircraft {
    let mut a = Box::new(aircraft {
        addr: (*mm).addr,
        flight: [0; 16],
        signalLevel: [(*mm).signalLevel; 8], // First time, initialise everything to the first signal strength
        altitude: 0,
        speed: 0,
        track: 0,
        vert_rate: 0,
        seen: 0,
        seenLatLon: 0,
        timestamp: 0,
        timestampLatLon: 0,
        messages: 0,
        modeA: 0,
        modeC: 0,
        modeAcount: 0,
        modeCcount: 0,
        modeACflags: 0,
        odd_cprlat: 0,
        odd_cprlon: 0,
        even_cprlat: 0,
        even_cprlon: 0,
        odd_cprtime: 0,
        even_cprtime: 0,
        lat: 0.0,
        lon: 0.0,
        bFlags: 0,
        next: ptr::null_mut(),
    });

    // mm->msgtype 32 is used to represent Mode A/C. These values can never change, so
    // set them once here during initialisation, and don't bother to set them every
    // time this ModeA/C is received again in the future
    if (*mm).msgtype == 32 as c_int {
        let mut modeC = ModeAToModeC(((*mm).modeA | (*mm).fs) as c_uint);
        a.modeACflags = MODEAC_MSG_FLAG;
        if modeC < -(12 as c_int) {
            a.modeACflags |= MODEAC_MSG_MODEA_ONLY
        } else {
            (*mm).altitude = modeC * 100 as c_int;
            (*mm).bFlags |= MODES_ACFLAGS_ALTITUDE_VALID
        }
    }

    Box::into_raw(a)
}

// Return the aircraft with the specified address, or NULL if no aircraft
// exists with this address.
//
unsafe fn interactiveFindAircraft(Modes: *const modes, addr: u32) -> *mut aircraft {
    let mut a = (*Modes).aircrafts;
    while !a.is_null() {
        if (*a).addr == addr {
            return a;
        }
        a = (*a).next
    }
    ptr::null_mut()
}

fn mstime() -> u64 {
    SystemTime::now()
        .duration_since(time::UNIX_EPOCH)
        .ok()
        .and_then(|duration| duration.as_millis().try_into().ok())
        .expect("mstime doesn't fit in u64")
}
