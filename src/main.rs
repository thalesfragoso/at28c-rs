#![no_main]
#![no_std]

//use panic_semihosting as _;
use core::{
    panic::PanicInfo,
    sync::atomic::{self, Ordering},
};

use cortex_m::asm;
//use cortex_m_semihosting::hprintln;
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
            .pclk2(8.mhz())
            .adcclk(1.mhz())
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
            asm::wfi();
        }
    }

    #[task(binds = USB_LP_CAN_RX0, priority = 3, resources = [usb_dev, serial, machine_state], spawn = [process])]
    fn usb_rx0(cx: usb_rx0::Context) {
        if !cx.resources.usb_dev.poll(&mut [cx.resources.serial]) {
            return;
        }
        match cx.resources.serial.process() {
            Ok(c) if c == 3 => {
                let buf = cx.resources.serial.reader_buf().unwrap();
                if *cx.resources.machine_state != State::WaitingPage {
                    let cmd = buf[0].into();
                    if *cx.resources.machine_state == State::Idle {
                        cx.spawn
                            .process(cmd, u16::from_le_bytes([buf[1], buf[2]]), None)
                            .ok();
                    }
                    let response = Commands::process(cmd, cx.resources.machine_state);
                    cx.resources.serial.clear_reader();
                    if cx.resources.serial.writer_len() == 64 {
                        cx.resources.serial.clear_writer();
                    }
                    if response != Response::NoResponse {
                        cx.resources.serial.write(&[response.into()]).ok();
                    }
                }
            }
            _ => {}
        }
    }

    #[task(priority = 2, capacity = 2, resources = [serial, blue_io, machine_state, led])]
    fn process(
        mut cx: process::Context,
        cmd: Commands,
        addr: u16,
        _buffer: Option<Box<SerialPool>>,
    ) {
        if cmd == Commands::ReadByte {
            let byte = cx.resources.blue_io.read_byte(addr);
            cx.resources
                .serial
                .lock(|shared| shared.write(&[byte]).ok());
            cx.resources
                .machine_state
                .lock(|shared| *shared = State::Idle);
        }
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
