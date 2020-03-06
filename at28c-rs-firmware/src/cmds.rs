use core::{
    convert::{From, Into},
    default::Default,
    mem,
};

#[derive(Copy, Clone, PartialEq)]
#[repr(u8)]
pub enum Commands {
    #[allow(dead_code)]
    Connect = 0x00,
    #[allow(dead_code)]
    ReadByte = 0x01,
    WriteByte = 0x02,
    WritePage = 0x03,
    #[allow(dead_code)]
    Disconnect = 0x04,
    #[allow(dead_code)]
    QueryState = 0x05,
    ReadPage = 0x06,
    DisableProctetion256 = 0x07,
    DisableProctetion64 = 0x08,
    Invalid = 0x09,
}

#[derive(Copy, Clone, PartialEq)]
#[repr(u8)]
pub enum Response {
    Connected = 0x00,
    WriteDone = 0x01,
    Idle = 0x02,
    Busy = 0x03,
    Disconnected = 0x04,
    NotValid = 0x05,
    NoResponse = 0x06,
    SendPage = 0x07,
    Error = 0x08,
}

#[derive(Copy, Clone, PartialEq)]
pub enum State {
    Idle = 0x00,
    Busy = 0x01,
    WaitingPage = 0x02,
    Disconnected = 0x03,
    Sending = 0x04,
}

impl Default for State {
    fn default() -> Self {
        Self::Disconnected
    }
}

impl Into<u8> for Response {
    fn into(self) -> u8 {
        // NOTE(unsafe), Response is repr(u8)
        unsafe { mem::transmute(self) }
    }
}

impl From<u8> for Commands {
    fn from(cmd: u8) -> Self {
        if cmd <= 0x08 {
            // NOTE(unsafe), Commands is repr(u8)
            unsafe { mem::transmute(cmd) }
        } else {
            Commands::Invalid
        }
    }
}

impl Commands {
    pub fn process(cmd: Self, state: &mut State) -> Response {
        match cmd {
            Commands::Connect => {
                if *state != State::Disconnected {
                    Response::NotValid
                } else {
                    *state = State::Idle;
                    Response::Connected
                }
            }
            Commands::ReadByte
            | Commands::WriteByte
            | Commands::DisableProctetion256
            | Commands::DisableProctetion64
            | Commands::ReadPage => {
                if *state != State::Idle {
                    Response::NotValid
                } else {
                    *state = State::Busy;
                    Response::NoResponse
                }
            }
            Commands::WritePage => {
                if *state != State::Idle {
                    Response::NotValid
                } else {
                    *state = State::WaitingPage;
                    Response::SendPage
                }
            }
            Commands::Disconnect => {
                if *state != State::Idle {
                    Response::NotValid
                } else {
                    *state = State::Disconnected;
                    Response::Disconnected
                }
            }
            Commands::QueryState => match *state {
                State::Idle => Response::Idle,
                State::Busy | State::Sending => Response::Busy,
                State::Disconnected => Response::Disconnected,
                State::WaitingPage => Response::SendPage,
            },
            Commands::Invalid => Response::NoResponse,
        }
    }
}
