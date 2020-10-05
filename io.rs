use crate::{modes, modesMessage};

extern "C" {
    fn modesSendSBSOutput(mm: *mut modesMessage);
    fn modesSendBeastOutput(mm: *mut modesMessage);
    fn modesSendRawOutput(mm: *mut modesMessage);
}

#[no_mangle]
pub unsafe extern "C" fn modesQueueOutput(Modes: *mut modes, mm: *mut modesMessage) {
    if (*Modes).stat_sbs_connections != 0 {
        modesSendSBSOutput(mm);
    }
    if (*Modes).stat_beast_connections != 0 {
        modesSendBeastOutput(mm);
    }
    if (*Modes).stat_raw_connections != 0 {
        modesSendRawOutput(mm);
    };
}
