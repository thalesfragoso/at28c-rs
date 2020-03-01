#![no_main]
#![no_std]

//use panic_semihosting as _;
use core::panic::PanicInfo;
use core::sync::atomic::{self, Ordering};

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
use usbd_serial::{SerialPort, USB_CLASS_CDC};

mod at28c;
use at28c::BlueIO;

type Led =
    stm32f1xx_hal::gpio::gpioc::PC13<stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::PushPull>>;

#[app(device = stm32f1xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        usb_dev: UsbDevice<'static, UsbBusType>,
        serial: SerialPort<'static, UsbBusType>,
        led: Led,
        clock: u32,
        blue_io: BlueIO<&'static pac::gpioa::RegisterBlock>,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        static mut USB_BUS: Option<bus::UsbBusAllocator<UsbBusType>> = None;

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

        //let serial = SerialPort::new_with_store(
        //    USB_BUS.as_ref().unwrap(),
        //    &mut SERIAL_RS[..],
        //    &mut SERIAL_WS[..],
        //);
        let serial = SerialPort::new(USB_BUS.as_ref().unwrap());

        let usb_dev = UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")
            .device_class(USB_CLASS_CDC)
            .build();

        // NOTE(unsafe) this is the GPIOA peripheral and we can safely use ODR register, since the
        // other pins used aren't configured as outputs
        let other_gpioa = unsafe { &(*pac::GPIOA::ptr()) };
        let blue_io = BlueIO::new(
            other_gpioa,
            cx.device.GPIOB,
            &mut afio.mapr,
            clocks.sysclk(),
        );

        init::LateResources {
            usb_dev,
            serial,
            led,
            clock: clocks.sysclk().0,
            blue_io,
        }
    }

    #[idle(resources = [serial, led, clock])]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            asm::wfi();
        }
    }

    //#[task(binds = USB_HP_CAN_TX, resources = [usb_dev, serial])]
    //fn usb_tx(mut cx: usb_tx::Context) {
    //    usb_poll(&mut cx.resources.usb_dev, &mut cx.resources.serial);
    //}

    #[task(binds = USB_LP_CAN_RX0, resources = [usb_dev, serial])]
    fn usb_rx0(cx: usb_rx0::Context) {
        if !cx.resources.usb_dev.poll(&mut [cx.resources.serial]) {
            return;
        }

        let mut buf = [0u8; 8];
        cx.resources.serial.read(&mut buf).ok();
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
