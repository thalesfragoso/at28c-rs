#![no_main]
#![no_std]

use core::{
    panic::PanicInfo,
    ptr,
    sync::atomic::{self, Ordering},
};

use cortex_m::asm;
use embedded_hal::digital::v2::OutputPin;
use rtfm::app;
use stm32f1xx_hal::{
    pac,
    prelude::*,
    usb::{Peripheral, UsbBus, UsbBusType},
};
use usb_device::bus;
use usb_device::prelude::*;
use usbd_serial::{
    heapless::{pool, Box, Pool},
    typenum::consts::U64,
    PoolNode, PoolPort, USB_CLASS_CDC,
};

mod at28c;
use at28c::{At28cIO, BlueIO};

mod cmds;
use cmds::{Commands, Response, State};

const NR_NODES: usize = 6;
const POOL_MEM: usize = core::mem::size_of::<PoolNode>() * NR_NODES;

pool!(
    #[allow(non_upper_case_globals)]
    SerialPool: PoolNode
);

type Led =
    stm32f1xx_hal::gpio::gpioc::PC13<stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::PushPull>>;

#[app(device = stm32f1xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        usb_dev: UsbDevice<'static, UsbBusType>,
        serial: PoolPort<'static, UsbBusType, SerialPool, SerialPool, U64, U64>,
        led: Led,
        blue_io: BlueIO,
        machine_state: State,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        static mut USB_BUS: Option<bus::UsbBusAllocator<UsbBusType>> = None;
        static mut MEMORY: [u8; POOL_MEM] = [0; POOL_MEM];

        let mut flash = cx.device.FLASH.constrain();
        let mut rcc = cx.device.RCC.constrain();
        let mut afio = cx.device.AFIO.constrain(&mut rcc.apb2);

        let clocks = rcc
            .cfgr
            .use_hse(8.mhz())
            .sysclk(72.mhz())
            .pclk1(36.mhz())
            .pclk2(72.mhz())
            .freeze(&mut flash.acr);

        assert!(clocks.usbclk_valid());

        let mut gpioa = cx.device.GPIOA.split(&mut rcc.apb2);
        let mut gpioc = cx.device.GPIOC.split(&mut rcc.apb2);
        let led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

        // BluePill board has a pull-up resistor on the D+ line.
        // Pull the D+ pin down to send a RESET condition to the USB bus.
        // This forced reset is needed only for development, without it host
        // will not reset your device when you upload new firmware.
        let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
        usb_dp.set_low().ok();
        asm::delay(clocks.sysclk().0 / 100);

        let usb_dm = gpioa.pa11;
        let usb_dp = usb_dp.into_floating_input(&mut gpioa.crh);

        let usb = Peripheral {
            usb: cx.device.USB,
            pin_dm: usb_dm,
            pin_dp: usb_dp,
        };

        *USB_BUS = Some(UsbBus::new(usb));

        SerialPool::grow(MEMORY);
        let wb = SerialPool::alloc().unwrap().init(PoolNode::new());
        let rb = SerialPool::alloc().unwrap().init(PoolNode::new());
        let serial = PoolPort::new(USB_BUS.as_ref().unwrap(), Some(wb), Some(rb)).unwrap();

        let usb_dev = UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")
            .device_class(USB_CLASS_CDC)
            .build();

        let blue_io = BlueIO::new(cx.device.GPIOB, &mut afio.mapr, clocks.sysclk());

        init::LateResources {
            usb_dev,
            serial,
            led,
            blue_io,
            machine_state: State::default(),
        }
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            // This could be a asm::wfi() but I used a nop() to avoid problems when connecting a debug probe
            asm::nop();
        }
    }

    #[task(binds = USB_LP_CAN_RX0, priority = 3, resources = [usb_dev, serial, machine_state], spawn = [process])]
    fn usb_rx0(cx: usb_rx0::Context) {
        static mut PAGE_ADDR: u16 = 0;
        if !cx.resources.usb_dev.poll(&mut [cx.resources.serial]) {
            return;
        }
        if cx.resources.serial.flush().is_ok() && *cx.resources.machine_state == State::Sending {
            *cx.resources.machine_state = State::Idle;
        }
        match cx.resources.serial.process() {
            Ok(c) if c >= 4 => {
                if *cx.resources.machine_state != State::WaitingPage {
                    let buf = cx.resources.serial.reader_buf().unwrap();
                    let cmd = buf[0].into();
                    if *cx.resources.machine_state == State::Idle {
                        match cmd {
                            Commands::ReadByte
                            | Commands::DisableProctetion256
                            | Commands::DisableProctetion64
                            | Commands::ReadPage => {
                                cx.spawn
                                    .process(cmd, u16::from_le_bytes([buf[1], buf[2]]), None, None)
                                    .ok();
                            }
                            Commands::WriteByte => {
                                cx.spawn
                                    .process(
                                        cmd,
                                        u16::from_le_bytes([buf[1], buf[2]]),
                                        Some(buf[3]),
                                        None,
                                    )
                                    .ok();
                            }
                            Commands::WritePage => {
                                *PAGE_ADDR = u16::from_le_bytes([buf[1], buf[2]]);
                            }
                            _ => {}
                        }
                    }
                    let response = Commands::process(cmd, cx.resources.machine_state);
                    if response != Response::NoResponse {
                        cx.resources.serial.write(&[response.into()]).ok();
                        cx.resources.serial.flush().ok();
                    }
                } else {
                    let buf = cx.resources.serial.replace_reader(None).unwrap();
                    cx.spawn
                        .process(Commands::WritePage, *PAGE_ADDR, None, Some(buf))
                        .ok();
                    *cx.resources.machine_state = State::Busy;
                }
            }
            _ => {}
        }
        cx.resources.serial.clear_reader();
    }

    #[task(priority = 2, capacity = 2, resources = [serial, blue_io, machine_state, led])]
    fn process(
        mut cx: process::Context,
        cmd: Commands,
        addr: u16,
        byte_write: Option<u8>,
        buffer: Option<Box<SerialPool>>,
    ) {
        let done = match cmd {
            Commands::ReadByte => {
                let byte = cx.resources.blue_io.read_byte(addr);
                cx.resources.serial.lock(|shared| {
                    shared.write(&[byte]).ok();
                    shared.flush()
                })
            }
            Commands::WriteByte => {
                if let Some(byte) = byte_write {
                    let result = cx.resources.blue_io.write_byte(byte, addr);
                    let response = if result.is_ok() {
                        Response::WriteDone
                    } else {
                        Response::Error
                    };
                    cx.resources.serial.lock(|shared| {
                        shared.write(&[response.into()]).ok();
                        shared.flush()
                    })
                } else {
                    Ok(())
                }
            }
            Commands::DisableProctetion256 => {
                cx.resources.blue_io.disable_write_protection256();
                cx.resources.serial.lock(|shared| {
                    shared.write(&[Response::WriteDone.into()]).ok();
                    shared.flush()
                })
            }
            Commands::DisableProctetion64 => {
                cx.resources.blue_io.disable_write_protection64();
                cx.resources.serial.lock(|shared| {
                    shared.write(&[Response::WriteDone.into()]).ok();
                    shared.flush()
                })
            }
            Commands::ReadPage => {
                let mut node = SerialPool::alloc().unwrap().init(PoolNode::new());
                let mut current_addr = addr;
                unsafe {
                    node.write_with(|buf, _len| {
                        for elem in buf.iter_mut() {
                            let byte = cx.resources.blue_io.read_byte(current_addr);
                            ptr::write(elem.as_mut_ptr(), byte);
                            current_addr += 1;
                        }
                        buf.len()
                    });
                }
                cx.resources.serial.lock(|shared| {
                    shared.replace_writer(Some(node));
                    shared.flush()
                })
            }
            Commands::WritePage => {
                if let Some(buf) = buffer {
                    let result = cx.resources.blue_io.write_page(buf.read(), addr);
                    let response = if result.is_ok() {
                        Response::WriteDone
                    } else {
                        Response::Error
                    };
                    cx.resources.serial.lock(|shared| {
                        shared.write(&[response.into()]).ok();
                        shared.flush()
                    })
                } else {
                    Ok(())
                }
            }
            _ => Ok(()),
        };
        cx.resources.machine_state.lock(|shared| {
            if done.is_ok() {
                *shared = State::Idle;
            } else {
                *shared = State::Sending;
            }
        });
        cx.resources.led.toggle().ok();
    }

    extern "C" {
        fn USART1();
    }
};

#[inline(never)]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    let gpioc = unsafe { &*pac::GPIOC::ptr() };
    gpioc.bsrr.write(|w| w.bs13().set_bit());
    loop {
        atomic::compiler_fence(Ordering::SeqCst);
    }
}
