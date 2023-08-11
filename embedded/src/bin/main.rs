#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use usb_audio_tests as _; // global logger + panicking-behavior + memory layout
use stm32h7xx_hal as hal;

#[rtic::app(
    device = hal::pac,
    dispatchers = [EXTI0, EXTI1, EXTI2]
)]
mod app {
    #[allow(unused_imports)]
    use usb_audio_tests::*;
    use usb_audio_tests::debug_gpio;
    
    use super::*;
    #[allow(unused_imports)]
    use hal::{
        prelude::*,        
        gpio::{Output, PushPull, gpioe::PE1},
        rcc::rec::UsbClkSel,
        usb_hs::{UsbBus, USB1}
    };

    use rtic_monotonics::systick::*;
    use heapless::Vec;

    #[allow(unused_imports)]
    use core::fmt::Write; // for pretty formatting of the serial output

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
    }

    // Local resources go here
    #[local]
    struct Local {
        tx: hal::serial::Tx<hal::stm32::USART3>,
        usb_handler: usb::USBHandler<'static>,
        debug_handler: debug_gpio::DebugHandler
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");
        
        // Initialize the monotonic timer at 8 MHz
        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, 48_000_000, systick_mono_token); // default STM32F4 clock rate at 8 MHz


        let pwr = cx.device.PWR.constrain();
        let pwrcfg = pwr.freeze();

        // RCC
        let rcc = cx.device.RCC.constrain();
        let mut ccdr = rcc.sys_ck(80.MHz()).freeze(pwrcfg, &cx.device.SYSCFG);

        // 48MHz CLOCK
        let _ = ccdr.clocks.hsi48_ck().expect("HSI48 must run");
        ccdr.peripheral.kernel_usb_clk_mux(UsbClkSel::Hsi48);

        // TODO: This might need to changa
        // let gpioa = cx.device.GPIOA.split(ccdr.peripheral.GPIOA); 
        let gpiob = cx.device.GPIOB.split(ccdr.peripheral.GPIOB); 
        let gpioc = cx.device.GPIOC.split(ccdr.peripheral.GPIOC);

        let tx = gpioc.pc10.into_alternate();
        let rx = gpioc.pc11.into_alternate();

        let serial = cx.device
            .USART3
            .serial((tx, rx), 921_600.bps(), ccdr.peripheral.USART3, &ccdr.clocks)
            .unwrap();

        let (tx, _rx) = serial.split();

        let usb_peripherals = usb::USBPeripherals {
            otg_hs_global: cx.device.OTG1_HS_GLOBAL,
            otg_hs_device: cx.device.OTG1_HS_DEVICE,
            otg_hs_pwclk: cx.device.OTG1_HS_PWRCLK,
            usb_dm: gpiob.pb14,
            usb_dp: gpiob.pb15,
            prec: ccdr.peripheral.USB1OTG,
            clocks: ccdr.clocks,
        };

        let usb_handler = usb::init(usb_peripherals);

        let debug_gpio = debug_gpio::DebugGPIO {
            usb_interrupt: gpioc.pc3,
            usb_audio_packet_interrupt: gpioc.pc2
        };

        let debug_handler = debug_gpio::init(debug_gpio);

        //USB_interrupt::spawn().ok();

        (
            Shared {
                // Initialization of shared resources go here
            },
            Local {
                tx,
                usb_handler,
                debug_handler
            },
        )
    }

    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::info!("idle");

        loop {
            continue;
        }
    }
    //#[task(
        //priority = 5,
        //local = [tx, usb_handler, debug_handler, buffer: Vec<u8, 0x1000> = Vec::new()]
    //)]
    #[task(
        binds= OTG_FS,
        priority = 5,
        local = [tx, usb_handler, debug_handler, buffer: Vec<u8, 0x1000> = Vec::new()]
    )]
    fn USB_interrupt(cx: USB_interrupt::Context) {
    //async fn USB_interrupt(cx: USB_interrupt::Context) {
        //loop {
            debug_gpio::toggle_usb_interrupt(cx.local.debug_handler);
            if cx.local.usb_handler.usb_dev.poll(&mut [&mut cx.local.usb_handler.usb_audio]) {
                let mut buf = [0u8; usb::USB_BUFFER_SIZE];
                if let Ok(len) = cx.local.usb_handler.usb_audio.read(&mut buf) {
                    debug_gpio::toggle_usb_audio_packet_interrupt(cx.local.debug_handler);
                    // writeln!(cx.local.tx, "{len}").unwrap();
                    for i in 0..len/2 {
                        let val: u16 = u16::from_le_bytes(buf[i*2..i*2+2].try_into().unwrap());
                        if val != 0 {
                            // writeln!(cx.local.tx, "{val}").unwrap();
                        }
                    }
                }
            }
			//Systick::delay(100.nanos()).await;
        //}
    }
}
