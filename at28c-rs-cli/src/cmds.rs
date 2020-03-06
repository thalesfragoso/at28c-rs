#[derive(Copy, Clone, PartialEq)]
#[repr(u8)]
pub enum Commands {
    #[allow(dead_code)]
    Connect = 0x00,
    #[allow(dead_code)]
    ReadByte = 0x01,
    #[allow(dead_code)]
    WriteByte = 0x02,
    WritePage = 0x03,
    #[allow(dead_code)]
    Disconnect = 0x04,
    #[allow(dead_code)]
    QueryState = 0x05,
    ReadPage = 0x06,
    #[allow(dead_code)]
    DisableProctetion256 = 0x07,
    #[allow(dead_code)]
    DisableProctetion64 = 0x08,
    #[allow(dead_code)]
    Invalid = 0x09,
}

#[derive(Copy, Clone, PartialEq)]
#[repr(u8)]
pub enum Response {
    Connected = 0x00,
    WriteDone = 0x01,
    #[allow(dead_code)]
    Idle = 0x02,
    #[allow(dead_code)]
    Busy = 0x03,
    #[allow(dead_code)]
    Disconnected = 0x04,
    #[allow(dead_code)]
    NotValid = 0x05,
    #[allow(dead_code)]
    NoResponse = 0x06,
    #[allow(dead_code)]
    SendPage = 0x07,
    #[allow(dead_code)]
    Error = 0x08,
    Invalid = 0x09,
}

impl Into<u8> for Commands {
    fn into(self) -> u8 {
        // NOTE(unsafe), Commands is repr(u8)
        unsafe { core::mem::transmute(self) }
    }
}

impl From<u8> for Response {
    fn from(resp: u8) -> Self {
        if resp <= 0x08 {
            // NOTE(unsafe), Response is repr(u8)
            unsafe { core::mem::transmute(resp) }
        } else {
            Response::Invalid
        }
    }
}
