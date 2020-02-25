use as_slice::AsSlice;
mod blue_at;
pub use blue_at::BlueIO;

#[derive(Copy, Clone, PartialEq)]
pub enum AtError {
    VerificationFail,
    InvalidAddress,
}

pub trait At28cIO {
    const MAX_ADDR: u16 = 0x7FFF;
    fn read_byte(&mut self, addr: u16) -> Result<u8, AtError>;
    fn write_byte(&mut self, byte: u8, addr: u16) -> Result<(), AtError>;
    fn write_page<B: AsSlice<Element = u8>>(
        &mut self,
        buff: B,
        start_addr: u16,
    ) -> Result<(), AtError>;
}
