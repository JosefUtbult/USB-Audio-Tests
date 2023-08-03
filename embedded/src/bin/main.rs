#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use usb_audio_tests as _; // global logger + panicking-behavior + memory layout

#[rtic::app(
    device = hal::pac,
    dispatchers = [EXTI0, EXTI1, EXTI2]
)]
mod app {
    use usb_audio_tests::*;
    
    use stm32f4xx_hal as hal;
    use rtic_monotonics::systick::*;
    use hal::pac as pac;
    use hal::prelude::*;
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
        tx: hal::serial::Tx<pac::USART1, u8>,
        usb_handler: usb::USBHandler<'static>,
        debug_handler: debug_gpio::DebugHandler
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");
        
        // Initialize the monotonic timer at 8 MHz
        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, 8_000_000, systick_mono_token); // default STM32F4 clock rate at 8 MHz

        let device = cx.device;
        let gpioa = device.GPIOA.split();
        //let gpiob = device.GPIOB.split();;
        let gpioc = device.GPIOC.split();

        let rcc = device.RCC.constrain();

        let clocks = rcc
            .cfgr
            .use_hse(8.MHz()) // Change this to the speed of the external crystal you're using
            .sysclk(48.MHz())
            .require_pll48clk()
            .freeze();

        let tx: hal::serial::Tx<pac::USART1, u8> = device.USART1.tx(
            gpioa.pa9, 
            115200.bps(), 
            &clocks
        ).unwrap();

        let usb_peripherals = usb::USBPeripherals {
            otg_fs_global: device.OTG_FS_GLOBAL,
            otg_fs_device: device.OTG_FS_DEVICE,
            otg_fs_pwclk: device.OTG_FS_PWRCLK,
            usb_dm: gpioa.pa11,
            usb_dp: gpioa.pa12,
            hclk: clocks.hclk() 
        };

        let usb_handler = usb::init(usb_peripherals);

        let debug_gpio = debug_gpio::DebugGPIO {
            usb_interrupt: gpioc.pc3,
            usb_audio_packet_interrupt: gpioc.pc2
        };

        let debug_handler = debug_gpio::init(debug_gpio);

        USB_interrupt::spawn().ok();

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

    #[task(
        priority = 1,
        local = [tx, usb_handler, debug_handler, buffer: Vec<u8, 0x1000> = Vec::new()]
    )]
    async fn USB_interrupt(cx: USB_interrupt::Context) {
        loop {
            debug_gpio::toggle_usb_interrupt(cx.local.debug_handler);
            if cx.local.usb_handler.usb_dev.poll(&mut [&mut cx.local.usb_handler.usb_audio]) {
                let mut buf = [0u8; usb::USB_BUFFER_SIZE];
                if let Ok(len) = cx.local.usb_handler.usb_audio.read(&mut buf) {
                    debug_gpio::toggle_usb_audio_packet_interrupt(cx.local.debug_handler);
                    writeln!(cx.local.tx, "{len}").unwrap();
                    for i in 0..len/2 {
                        let val: u16 = u16::from_le_bytes(buf[i*2..i*2+2].try_into().unwrap());
                        if val != 0 {
                            writeln!(cx.local.tx, "{val}").unwrap();
                        }
                    }
                }
            }
            Systick::delay(100.nanos()).await;
        }
    }
}
