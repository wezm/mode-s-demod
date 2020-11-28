use std::convert::TryInto;
use std::os::raw::{c_int, c_long, c_uint};
use std::time::SystemTime;
use std::{mem, time};

use crate::mode_ac::{
    mode_a_to_mode_c, MODEAC_MSG_FLAG, MODEAC_MSG_MODEA_ONLY, MODEAC_MSG_MODEC_OLD,
};
use crate::mode_s::{decode_cpr, decode_cpr_relative};
use crate::{
    Aircraft, ModeS, ModesMessage, MODES_ACFLAGS_ALTITUDE_VALID, MODES_ACFLAGS_AOG,
    MODES_ACFLAGS_AOG_VALID, MODES_ACFLAGS_CALLSIGN_VALID, MODES_ACFLAGS_HEADING_VALID,
    MODES_ACFLAGS_LATLON_VALID, MODES_ACFLAGS_LLBOTH_VALID, MODES_ACFLAGS_LLEITHER_VALID,
    MODES_ACFLAGS_LLODD_VALID, MODES_ACFLAGS_SPEED_VALID, MODES_ACFLAGS_SQUAWK_VALID,
    MODES_ACFLAGS_VERTRATE_VALID,
};

const MODEAC_MSG_MODEA_HIT: c_int = (1 as c_int) << 2 as c_int;
const MODEAC_MSG_MODEC_HIT: c_int = (1 as c_int) << 3 as c_int;

// Receive new messages and populate the interactive mode with more info
//
pub(crate) fn interactive_receive_data(mode_s: &mut ModeS, mm: &mut ModesMessage) {
    // Return if (checking crc) AND (not crcok) AND (not fixed)
    if mode_s.check_crc && mm.crcok == 0 && mm.correctedbits == 0 {
        return;
    }

    let mut aircrafts = Vec::new();
    mem::swap(&mut mode_s.aircrafts, &mut aircrafts);

    // Lookup our Aircraft or create a new one
    let a = match interactive_find_aircraft(&mut aircrafts, mm.addr) {
        Some(aircraft) => aircraft,
        None => {
            // If it's a currently unknown Aircraft... create a new record for it,.
            let a = interactive_create_aircraft(mm);
            // .. and put it at the head of the list
            aircrafts.push(a);
            aircrafts.last_mut().unwrap()
        }
    };

    // // FIXME: This was disabled (via if 0 in the C code)
    // if 0 as c_int != 0 && mode_s.aircrafts != a && crate::now() as i64 - (*a).seen >= 1
    // {
    //     aux = mode_s.aircrafts;
    //     while (*aux).next != a {
    //         aux = (*aux).next
    //     }
    //     /* If it is an already known Aircraft, move it on head
    //      * so we keep aircrafts ordered by received message time.
    //      *
    //      * However move it on head only if at least one second elapsed
    //      * since the Aircraft that is currently on head sent a message,
    //      * otherwise with multiple aircrafts at the same time we have an
    //      * useless shuffle of positions on the screen. */
    //     /* Now we are a node before the Aircraft to remove. */
    //     (*aux).next = (*(*aux).next).next; /* removed. */
    //     /* Add on head */
    //     (*a).next = mode_s.aircrafts;
    //     mode_s.aircrafts = a
    // }

    a.signal_level[(a.messages & 7 as c_int as c_long) as usize] = mm.signal_level; // replace the 8th oldest signal strength
    a.seen = crate::now() as i64;
    a.timestamp = mm.timestamp_msg;
    a.messages += 1;

    // If a (new) CALLSIGN has been received, copy it to the Aircraft structure
    if mm.b_flags & MODES_ACFLAGS_CALLSIGN_VALID != 0 {
        a.flight.copy_from_slice(&mm.flight);
    }

    // If a (new) ALTITUDE has been received, copy it to the Aircraft structure
    if mm.b_flags & MODES_ACFLAGS_ALTITUDE_VALID != 0 {
        if a.mode_c_count != 0 && a.altitude != mm.altitude {
            // and Altitude has changed
            //        && (a->mode_c     != mm->mode_c + 1)   // and Altitude not changed by +100 feet
            //        && (a->mode_c + 1 != mm->mode_c    ) ) // and Altitude not changes by -100 feet
            a.mode_c_count = 0 as c_int as c_long; //....zero the hit count
            a.mode_ac_flags &= !MODEAC_MSG_MODEC_HIT
        }
        a.altitude = mm.altitude;
        a.mode_c = (mm.altitude + 49) / 100
    }

    // If a (new) SQUAWK has been received, copy it to the Aircraft structure
    if mm.b_flags & MODES_ACFLAGS_SQUAWK_VALID != 0 {
        if a.mode_a != mm.mode_a {
            a.mode_a_count = 0; // Squawk has changed, so zero the hit count
            a.mode_ac_flags &= !MODEAC_MSG_MODEA_HIT
        }
        a.mode_a = mm.mode_a
    }

    // If a (new) HEADING has been received, copy it to the Aircraft structure
    if mm.b_flags & MODES_ACFLAGS_HEADING_VALID != 0 {
        a.track = mm.heading
    }

    // If a (new) SPEED has been received, copy it to the Aircraft structure
    if mm.b_flags & MODES_ACFLAGS_SPEED_VALID != 0 {
        a.speed = mm.velocity
    }

    // If a (new) Vertical Descent rate has been received, copy it to the Aircraft structure
    if mm.b_flags & MODES_ACFLAGS_VERTRATE_VALID != 0 {
        a.vert_rate = mm.vert_rate
    }

    // if the Aircraft has landed or taken off since the last message, clear the even/odd CPR flags
    if mm.b_flags & MODES_ACFLAGS_AOG_VALID != 0
        && (a.b_flags ^ mm.b_flags) & MODES_ACFLAGS_AOG != 0
    {
        a.b_flags &= !(MODES_ACFLAGS_LLBOTH_VALID | MODES_ACFLAGS_AOG)
    }

    // If we've got a new cprlat or cprlon
    if mm.b_flags & MODES_ACFLAGS_LLEITHER_VALID != 0 {
        let mut location_ok = 0;

        if mm.b_flags & MODES_ACFLAGS_LLODD_VALID != 0 {
            a.odd_cprlat = mm.raw_latitude;
            a.odd_cprlon = mm.raw_longitude;
            a.odd_cprtime = mstime()
        } else {
            a.even_cprlat = mm.raw_latitude;
            a.even_cprlon = mm.raw_longitude;
            a.even_cprtime = mstime()
        }

        // If we have enough recent data, try global CPR
        if (mm.b_flags | a.b_flags) & MODES_ACFLAGS_LLEITHER_VALID == MODES_ACFLAGS_LLBOTH_VALID
            && (a.even_cprtime.wrapping_sub(a.odd_cprtime) as c_int).abs() <= 10000
        {
            if decode_cpr(
                &mode_s,
                a,
                mm.b_flags & MODES_ACFLAGS_LLODD_VALID,
                mm.b_flags & MODES_ACFLAGS_AOG,
            ) == 0
            {
                location_ok = 1
            }
        }

        // Otherwise try relative CPR.
        if location_ok == 0
            && decode_cpr_relative(
                &mode_s,
                a,
                mm.b_flags & MODES_ACFLAGS_LLODD_VALID,
                mm.b_flags & MODES_ACFLAGS_AOG,
            ) == 0
        {
            location_ok = 1
        }

        // If we successfully decoded, back copy the results to mm so that we can print them in list output
        if location_ok != 0 {
            mm.b_flags |= MODES_ACFLAGS_LATLON_VALID;
            mm.f_lat = a.lat;
            mm.f_lon = a.lon
        }
    }

    // Update the aircrafts a->b_flags to reflect the newly received mm->b_flags;
    a.b_flags |= (*mm).b_flags;
    if (*mm).msgtype == 32 {
        let flags = a.mode_ac_flags;
        if flags & (MODEAC_MSG_MODEC_HIT | MODEAC_MSG_MODEC_OLD) == MODEAC_MSG_MODEC_OLD {
            //
            // This Mode-C doesn't currently hit any known Mode-S, but it used to because MODEAC_MSG_MODEC_OLD is
            // set  So the Aircraft it used to match has either changed altitude, or gone out of our receiver range
            //
            // We've now received this Mode-A/C again, so it must be a new Aircraft. It could be another Aircraft
            // at the same Mode-C altitude, or it could be a new airctraft with a new Mods-A squawk.
            //
            // To avoid masking this Aircraft from the interactive display, clear the MODEAC_MSG_MODES_OLD flag
            // and set messages to 1;
            //
            a.mode_ac_flags = flags & !MODEAC_MSG_MODEC_OLD;
            a.messages = 1
        }
    }

    // If we are Logging DF's, and it's not a Mode A/C
    if mode_s.b_enable_dflogging && (*mm).msgtype < 32 {
        // FIXME; port this if needed
        // interactiveCreateDF(a, mm);
    }

    // Put it back
    mem::swap(&mut mode_s.aircrafts, &mut aircrafts);
}

// Return a new Aircraft structure for the interactive mode linked list
// of Aircraft
//
fn interactive_create_aircraft(mm: &mut ModesMessage) -> Aircraft {
    let mut a = Aircraft {
        addr: mm.addr,
        flight: ['?' as u8; 8],
        signal_level: [mm.signal_level; 8], // First time, initialise everything to the first signal strength
        altitude: 0,
        speed: 0,
        track: 0,
        vert_rate: 0,
        seen: 0,
        seen_lat_lon: 0,
        timestamp: 0,
        timestamp_lat_lon: 0,
        messages: 0,
        mode_a: 0,
        mode_c: 0,
        mode_a_count: 0,
        mode_c_count: 0,
        mode_ac_flags: 0,
        odd_cprlat: 0,
        odd_cprlon: 0,
        even_cprlat: 0,
        even_cprlon: 0,
        odd_cprtime: 0,
        even_cprtime: 0,
        lat: 0.0,
        lon: 0.0,
        b_flags: 0,
    };

    // mm->msgtype 32 is used to represent Mode A/C. These values can never change, so
    // set them once here during initialisation, and don't bother to set them every
    // time this ModeA/C is received again in the future
    if mm.msgtype == 32 {
        let mode_c = mode_a_to_mode_c((mm.mode_a | mm.fs) as c_uint);
        a.mode_ac_flags = MODEAC_MSG_FLAG;
        if mode_c < -12 {
            a.mode_ac_flags |= MODEAC_MSG_MODEA_ONLY
        } else {
            mm.altitude = mode_c * 100;
            mm.b_flags |= MODES_ACFLAGS_ALTITUDE_VALID
        }
    }

    a
}

// Return the Aircraft with the specified address, or NULL if no Aircraft
// exists with this address.
//
fn interactive_find_aircraft(aircraft: &mut [Aircraft], addr: u32) -> Option<&mut Aircraft> {
    aircraft.iter_mut().find(|aircraft| aircraft.addr == addr)
}

fn mstime() -> u64 {
    SystemTime::now()
        .duration_since(time::UNIX_EPOCH)
        .ok()
        .and_then(|duration| duration.as_millis().try_into().ok())
        .expect("mstime doesn't fit in u64")
}
