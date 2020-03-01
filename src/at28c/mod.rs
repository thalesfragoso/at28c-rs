mod blue_at;
pub use blue_at::BlueIO;

#[derive(Copy, Clone, PartialEq)]
pub enum AtError {
    WriteTimeout,
    InvalidAddress,
    PageOverflow,
}

pub trait At28cIO {
    const MAX_ADDR: u16;
    const WRITE_TIMEOUT_MS: u8;
    fn read_byte(&mut self, addr: u16) -> u8;
    fn write_byte(&mut self, byte: u8, addr: u16) -> Result<(), AtError>;
    fn write_page(&mut self, buf: &[u8], start_addr: u16) -> Result<(), AtError>;
}
