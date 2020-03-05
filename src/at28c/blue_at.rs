use core::convert::TryFrom;

use super::{At28cIO, AtError};
use cortex_m::asm::delay;
use stm32f1xx_hal::{
    afio::MAPR,
    bb,
    pac::{self, AFIO, GPIOB, RCC},
    time::Hertz,
};

#[derive(Copy, Clone, PartialEq)]
pub enum IOState {
    Reading,
    Writing,
}

/// B8 to B15 -> IO lines
/// A0 to A10 -> Lower address byte
/// B3 to B6 -> Higher address byte
/// B0 -> Output Enable
/// B1 -> Write Enable
pub struct BlueIO {
    portb: GPIOB,
    state: IOState,
    cycles_us: u32,
}

impl BlueIO {
    #[inline(always)]
    pub fn new(portb: GPIOB, _mapr: &mut MAPR, sysclk: impl Into<Hertz>) -> Self {
        // Enable and reset the GPIOB peripheral
        // NOTE(unsafe) atomic writes with no side-effects
        unsafe {
            // PORTA
            bb::set(&(*RCC::ptr()).apb2enr, 2);
            bb::set(&(*RCC::ptr()).apb2rstr, 2);
            bb::clear(&(*RCC::ptr()).apb2rstr, 2);

            // PORTB
            bb::set(&(*RCC::ptr()).apb2enr, 3);
            bb::set(&(*RCC::ptr()).apb2rstr, 3);
            bb::clear(&(*RCC::ptr()).apb2rstr, 3);
        }
        // NOTE(unsafe) Only access BlueIO exclusive registers
        let porta = unsafe { &(*pac::GPIOA::ptr()) };
        porta.crl.write(|w| {
            w.mode0()
                .output50()
                .mode1()
                .output50()
                .mode2()
                .output50()
                .mode3()
                .output50()
                .mode4()
                .output50()
                .mode5()
                .output50()
                .mode6()
                .output50()
                .mode7()
                .output50()
                .cnf0()
                .push_pull()
                .cnf1()
                .push_pull()
                .cnf2()
                .push_pull()
                .cnf3()
                .push_pull()
                .cnf4()
                .push_pull()
                .cnf5()
                .push_pull()
                .cnf6()
                .push_pull()
                .cnf7()
                .push_pull()
        });
        porta.crh.modify(|_, w| {
            w.mode8()
                .output50()
                .mode9()
                .output50()
                .mode10()
                .output50()
                .cnf8()
                .push_pull()
                .cnf9()
                .push_pull()
                .cnf10()
                .push_pull()
        });
        // Disable JTAG to use B3 and B4
        // NOTE(unsafe) We have exclusive access to mapr register and `0b010` is safe to write to
        // swj_cfg
        unsafe {
            (*AFIO::ptr()).mapr.modify(|_, w| w.swj_cfg().bits(0b010));
        }
        // Output enable and write enable are active low, so set then high before configuring them
        portb.bsrr.write(|w| w.bs0().set_bit().bs1().set_bit());
        portb.crl.modify(|_, w| {
            w.mode0()
                .output50()
                .mode1()
                .output50()
                .mode3()
                .output50()
                .mode4()
                .output50()
                .mode5()
                .output50()
                .mode6()
                .output50()
                .cnf0()
                .push_pull()
                .cnf1()
                .push_pull()
                .cnf3()
                .push_pull()
                .cnf4()
                .push_pull()
                .cnf5()
                .push_pull()
                .cnf6()
                .push_pull()
        });
        Self {
            portb,
            state: IOState::Reading,
            cycles_us: sysclk.into().0 / 1_000_000,
        }
    }

    /// Returns the higher 8bits of PORTB
    fn read(&mut self) -> u8 {
        if self.state != IOState::Reading {
            self.portb.crh.reset();
            self.state = IOState::Reading;
        }
        let input = self.portb.idr.read().bits();
        ((input >> 8) & 0xFF) as u8
    }

    /// Writes to the higher 8bits of PORTB
    fn write(&mut self, byte: u8) {
        if self.state != IOState::Writing {
            self.portb.crh.write(|w| {
                w.mode8()
                    .output50()
                    .mode9()
                    .output50()
                    .mode10()
                    .output50()
                    .mode11()
                    .output50()
                    .mode12()
                    .output50()
                    .mode13()
                    .output50()
                    .mode14()
                    .output50()
                    .mode15()
                    .output50()
                    .cnf8()
                    .push_pull()
                    .cnf9()
                    .push_pull()
                    .cnf10()
                    .push_pull()
                    .cnf11()
                    .push_pull()
                    .cnf12()
                    .push_pull()
                    .cnf13()
                    .push_pull()
                    .cnf14()
                    .push_pull()
                    .cnf15()
                    .push_pull()
            });
            self.state = IOState::Writing;
        }
        let output = u32::from(byte) << 8;
        self.portb.odr.modify(|r, w| {
            let current = r.bits();
            // NOTE(unsafe) Not changing the reserved bits
            unsafe { w.bits((current & !0x0000_FF00) | output) }
        });
    }

    fn set_address(&mut self, addr: u16) {
        let addr_low = u32::from(addr & 0x07FF);
        let addr_high = u32::from((addr >> 11) & 0x000F);

        // NOTE(unsafe) Only access BlueIO exclusive registers
        let porta = unsafe { &(*pac::GPIOA::ptr()) };
        porta.odr.modify(|r, w| {
            let current = r.bits();
            // NOTE(unsafe) Not changing the reserved bits
            unsafe { w.bits((current & !0x0000_07FF) | addr_low) }
        });
        self.portb.odr.modify(|r, w| {
            let to_write = (r.bits() & !0x0000_0078) | ((addr_high << 3) & 0x0000_0078);
            // NOTE(unsafe) Not changing the reserved bits
            unsafe { w.bits(to_write) }
        });
    }

    /// This input in the eeprom chip is active low
    fn set_output_enable(&mut self, active: bool) {
        self.portb.bsrr.write(|w| {
            if active {
                w.br0().set_bit()
            } else {
                w.bs0().set_bit()
            }
        });
    }

    /// This input in the eeprom chip is active low
    fn set_write_enable(&mut self, active: bool) {
        self.portb.bsrr.write(|w| {
            if active {
                w.br1().set_bit()
            } else {
                w.bs1().set_bit()
            }
        });
    }
}

impl At28cIO for BlueIO {
    const WRITE_TIMEOUT_MS: u8 = 10;

    fn read_byte(&mut self, addr: u16) -> u8 {
        self.set_address(addr);
        self.set_output_enable(true);
        // delay for 1us
        delay(self.cycles_us);
        let byte = self.read();
        self.set_output_enable(false);
        // delay for 0.5us, this works because cycles_us is always >= 8 for stm32f103
        delay(self.cycles_us / 2);
        byte
    }

    fn write_byte(&mut self, byte: u8, addr: u16) -> Result<(), AtError> {
        self.set_address(addr);
        self.write(byte);
        self.set_write_enable(true);
        // delay for 0.5us, this works because cycles_us is always >= 8 for stm32f103
        delay(self.cycles_us / 2);
        self.set_write_enable(false);
        let mut attempts = 0;
        let timeout = loop {
            if self.read_byte(addr) == byte {
                break false;
            } else {
                attempts += 1;
                if attempts == Self::WRITE_TIMEOUT_MS {
                    break true;
                }
                // delay for 1ms
                delay(self.cycles_us * 1000);
            }
        };
        if timeout {
            Err(AtError::WriteTimeout)
        } else {
            Ok(())
        }
    }

    fn write_page(&mut self, buf: &[u8], start_addr: u16) -> Result<(), AtError> {
        if buf.is_empty() {
            return Ok(());
        }
        let len = u16::try_from(buf.len()).map_err(|_| AtError::PageOverflow)?;
        // Check if the buf fits in one single page
        if start_addr + len > start_addr | 0x003F {
            return Err(AtError::PageOverflow);
        }
        let mut addr = start_addr;
        for byte in buf.iter() {
            self.set_address(addr);
            self.write(*byte);
            self.set_write_enable(true);
            // delay for 0.5us, this works because cycles_us is always >= 8 for stm32f103
            delay(self.cycles_us / 2);
            self.set_write_enable(false);
            addr += 1;
        }
        let mut attempts = 0;
        let timeout = loop {
            if self.read_byte(addr) == buf[(len as usize) - 1] {
                break false;
            } else {
                attempts += 1;
                if attempts == Self::WRITE_TIMEOUT_MS {
                    break true;
                }
                // delay for 1ms
                delay(self.cycles_us * 1000);
            }
        };
        if timeout {
            Err(AtError::WriteTimeout)
        } else {
            Ok(())
        }
    }

    fn disable_write_protection(&mut self) {
        let data = &[0xAA, 0x55, 0x80, 0xAA, 0x55, 0x20];
        let addresses = &[0x5555, 0x2AAA, 0x5555, 0x5555, 0x2AAA, 0x5555];
        for (byte, addr) in data.iter().zip(addresses.iter()) {
            self.set_address(*addr);
            self.write(*byte);
            self.set_write_enable(true);
            // delay for 5us
            delay(self.cycles_us * 5);
            self.set_write_enable(false);
        }
    }
}
