use core::ops::Deref;

use super::{At28cIO, AtError};
use stm32f1xx_hal::{
    afio::MAPR,
    bb,
    pac::{self, AFIO, GPIOB, RCC},
};

#[derive(Copy, Clone, PartialEq)]
pub enum IOState {
    Reading,
    Writing,
}

/// B8 to B15 -> IO lines
/// A0 to A10 -> Lower address byte
/// B3 to B6 -> Higher address byte
pub struct BlueIO<T> {
    porta: T,
    portb: GPIOB,
    state: IOState,
}

impl<T: Deref<Target = pac::gpioa::RegisterBlock>> BlueIO<T> {
    #[inline(always)]
    pub fn new(porta: T, portb: GPIOB, _mapr: &mut MAPR) -> Self {
        // Enable and reset the GPIOB peripheral
        // NOTE(unsafe) atomic writes with no side-effects
        unsafe {
            // PORTA
            bb::set(&(&(*RCC::ptr()).apb2enr), 2);
            bb::set(&(&(*RCC::ptr()).apb2rstr), 2);
            bb::clear(&(&(*RCC::ptr()).apb2rstr), 2);

            // PORTB
            bb::set(&(&(*RCC::ptr()).apb2enr), 3);
            bb::set(&(&(*RCC::ptr()).apb2rstr), 3);
            bb::clear(&(&(*RCC::ptr()).apb2rstr), 3);
        }
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
        portb.crl.modify(|_, w| {
            w.mode3()
                .output50()
                .mode4()
                .output50()
                .mode5()
                .output50()
                .mode6()
                .output50()
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
            porta,
            portb,
            state: IOState::Reading,
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
            unsafe { w.bits(current | output) }
        });
    }

    fn set_address(&mut self, addr: u16) -> Result<(), AtError> {
        if addr > 0x7FFF {
            return Err(AtError::InvalidAddress);
        }
        let addr_low = u32::from(addr & 0x07FF);
        let addr_high = u32::from((addr >> 11) & 0x000F);
        self.porta.odr.modify(|r, w| {
            let current = r.bits();
            // NOTE(unsafe) Not changing the reserved bits
            unsafe { w.bits(current | addr_low) }
        });
        self.portb.odr.modify(|r, w| {
            let to_write = r.bits() | ((addr_high << 3) & 0x0000_0078);
            // NOTE(unsafe) Not changing the reserved bits
            unsafe { w.bits(to_write) }
        });
        Ok(())
    }
}
