use std::os::raw::c_int;

const MODES_LONG_MSG_BYTES: c_int = 14;
const MODES_SHORT_MSG_BYTES: c_int = 7;
const MODES_LONG_MSG_BITS: c_int = MODES_LONG_MSG_BYTES * 8;
const MODES_SHORT_MSG_BITS: c_int = MODES_SHORT_MSG_BYTES * 8;

#[no_mangle]
pub extern "C" fn modesMessageLenByType(type_: c_int) -> c_int {
    // (type_ & 0x10) ? MODES_LONG_MSG_BITS : MODES_SHORT_MSG_BITS ;
    if type_ & 0x10 == 0x10 {
        MODES_LONG_MSG_BITS
    } else {
        MODES_SHORT_MSG_BITS
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
