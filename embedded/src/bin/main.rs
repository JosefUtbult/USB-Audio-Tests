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
    // use hal::gpio;
    #[allow(unused_imports)]
    use usb_audio_tests::*;
    use usb_audio_tests::debug_gpio;
    use usb_audio_tests::codec;
    
    use super::*;
    use num_traits::real::Real;

    #[allow(unused_imports)]
    use hal::{
        prelude::*,        
        stm32,
        gpio::{Output, PushPull, gpioe::PE1},
        rcc::rec::UsbClkSel,
        usb_hs::{UsbBus, USB1},
    };

    use rtic_monotonics::systick::*;
    // use heapless::Vec;

    #[allow(unused_imports)]
    use core::fmt::Write; // for pretty formatting of the serial output

    // Shared resources go here
    #[shared]
    struct Shared {
        #[lock_free]
        codec_container: codec::SaiContainer,
        debug_handler: debug_gpio::DebugHandler
    }

    // Local resources go here
    #[local]
    struct Local {
        tx1: hal::serial::Tx<hal::stm32::USART3>,
        tx2: hal::serial::Tx<hal::stm32::USART2>,
        usb_handler: usb::USBHandler<'static>,
    }

    fn format_buffer_length(integer: u16, fraction: u16) -> [u8; 3] {
        [(integer >> 2) as u8, (integer << 6) as u8 | (fraction >> 4) as u8, (fraction << 4) as u8]
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("Init");
        
        // Initialize the monotonic timer at 8 MHz
        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, 48_000_000, systick_mono_token); // default STM32F4 clock rate at 8 MHz


        let pwr = cx.device.PWR.constrain();
        let pwrcfg = pwr.freeze();

        // RCC
        // TODO: Look over clock configuration
        let rcc = cx.device.RCC.constrain();
        let mut ccdr = rcc
            .sys_ck(80.MHz())
            // Used for I2S master clock
            .pll3_p_ck(codec::PLL3_P_HZ)
            .pll1_q_ck(codec::PLL3_P_HZ)
            .freeze(pwrcfg, &cx.device.SYSCFG);

        // 48MHz CLOCK
        let _ = ccdr.clocks.hsi48_ck().expect("HSI48 must run");
        ccdr.peripheral.kernel_usb_clk_mux(UsbClkSel::Hsi48);

        // GPIO setup
        let gpioa = cx.device.GPIOA.split(ccdr.peripheral.GPIOA); 
        let gpiob = cx.device.GPIOB.split(ccdr.peripheral.GPIOB); 
        let gpioc = cx.device.GPIOC.split(ccdr.peripheral.GPIOC);
        let gpiod = cx.device.GPIOD.split(ccdr.peripheral.GPIOD);
        let gpioe = cx.device.GPIOE.split(ccdr.peripheral.GPIOE);

        // USART setup
        let tx1 = gpioc.pc10.into_alternate();
        let rx1 = gpioc.pc11.into_alternate();
        let tx2 = gpiod.pd5.into_alternate();
        let rx2 = gpiod.pd6.into_alternate();

        let serial1 = cx.device
            .USART3
            .serial((tx1, rx1), 921_600.bps(), ccdr.peripheral.USART3, &ccdr.clocks)
            .unwrap();

        let serial2 = cx.device
            .USART2
            .serial((tx2, rx2), 921_600.bps(), ccdr.peripheral.USART2, &ccdr.clocks)
            .unwrap();

        let (tx1, _rx1) = serial1.split();
        let (tx2, _rx2) = serial2.split();

        // USB setup
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

        // Debug pins setup
        let debug_gpio_pins = debug_gpio::DebugGPIO {
            usb_interrupt: gpiob.pb8,
            usb_audio_packet_interrupt: gpiob.pb9,
            codec_1_interrupt: gpioa.pa5,
            codec_2_interrupt: gpioa.pa6
        };

        let debug_handler = debug_gpio::init(debug_gpio_pins);

        // Codec setup
        let codec = codec::Codec {
            reset: gpiob.pb11.into(),
            sai1_mclk: gpioe.pe2,
            sai1_sck: gpioe.pe5,
            sai1_lrclk: gpioe.pe4,
            sai1_dout: gpioe.pe6,
            sai1_din: gpioe.pe3,
            sai2_sck: gpiod.pd13,
            sai2_lrckl: gpiod.pd12,
            sai2_mclk: gpioe.pe0,
            sai2_dout: gpiod.pd11,
            sai2_din: gpioa.pa0,
            i2c1_scl: gpiob.pb6,
            i2c1_sda: gpiob.pb7,
            sai1_rec: ccdr.peripheral.SAI1,
            sai2_rec: ccdr.peripheral.SAI2,
            sai1_dev: cx.device.SAI1,
            sai2_dev: cx.device.SAI2,
            clocks: ccdr.clocks,
            scb: cx.core.SCB,
            i2c1: cx.device.I2C1,
            i2c1_peripheral: ccdr.peripheral.I2C1,
        };

        let codec_container = codec::init(codec);

        (
            Shared {
                codec_container: codec_container,
                debug_handler
            },
            Local {
                tx1,
                tx2,
                usb_handler,
            },
        )
    }

    #[task(
        binds = SAI1, 
        priority = 6,
        shared = [
            codec_container,
            debug_handler
        ],
        local = [
            counter: u8 = 0
        ]
    )]
    fn passthru1(mut cx: passthru1::Context) {
        *cx.local.counter += 1;
        if *cx.local.counter >= 48 {
            *cx.local.counter = 0;

            cx.shared.debug_handler.lock(|debug_handler| {
                debug_gpio::toggle_codec_1_interrupt(debug_handler);
            });
        }
        if let Ok((left, right)) = hal::traits::i2s::FullDuplex::try_read(&mut cx.shared.codec_container.0) {
            hal::traits::i2s::FullDuplex::try_send(&mut cx.shared.codec_container.0, left, right).unwrap();
        }
    }

    #[task(
        binds = SAI2, 
        priority = 6,
        shared = [
            codec_container,
            debug_handler
        ],
        local = [
            counter: u8 = 0
        ]
    )]
    fn passthru2(mut cx: passthru2::Context) {
        *cx.local.counter += 1;
        if *cx.local.counter >= 48 {
            *cx.local.counter = 0;

            cx.shared.debug_handler.lock(|debug_handler| {
                debug_gpio::toggle_codec_2_interrupt(debug_handler);
            });
        }

        if let Ok((left, right)) = hal::traits::i2s::FullDuplex::try_read(&mut cx.shared.codec_container.1) {
            hal::traits::i2s::FullDuplex::try_send(&mut cx.shared.codec_container.1, left, right).unwrap();
        }
    } 
    
    #[task(
        binds= OTG_HS,
        priority = 5,
        local = [
            tx1,
            tx2, 
            usb_handler, 
            // buffer: Vec<u8, 0x1000> = Vec::new()
            // counter: u16 = 0
            counter: f64 = 0.0
        ],
        shared = [
            debug_handler, 
        ]
    )]
    fn USB_interrupt(mut cx: USB_interrupt::Context) {
        cx.shared.debug_handler.lock(|debug_handler| {
            debug_gpio::toggle_usb_interrupt(debug_handler);
        });
        if cx.local.usb_handler.usb_dev.poll(&mut [&mut cx.local.usb_handler.usb_audio]) {
            let mut buf = [0u8; usb::USB_BUFFER_SIZE];
            if let Ok(len) = cx.local.usb_handler.usb_audio.read(&mut buf) {
                cx.shared.debug_handler.lock(|debug_handler| {
                    debug_gpio::toggle_usb_audio_packet_interrupt(debug_handler);
                });
                writeln!(cx.local.tx1, "{len}").unwrap();
                
                // Simulate clock drift
                let scaler = 1.0;
                let modifier = 48.0 - scaler * (*cx.local.counter).sin();
                // let modifier: f64 = 47.5 + *cx.local.counter;
                // if modifier < 48.0 {*cx.local.counter += 0.01};
                *cx.local.counter += 0.005;
                // let modifier = if (*cx.local.counter > 2000) | (*cx.local.counter < 1000) {48.0} else {47.5};
                // *cx.local.counter += 1;

                // let modifier = 46.0 + *cx.local.counter;
                // *cx.local.counter += 0.000035;
                let integer: u16 = modifier as u16;
                let fraction: u16 = ((modifier - modifier as i64 as f64) * 1024_f64) as u16;
                cx.local.usb_handler.usb_audio.write_synch_interrupt(&format_buffer_length(integer, fraction)).ok();
                // defmt::info!("{}, {}", integer, fraction);
                writeln!(cx.local.tx2, "{modifier:.2}").unwrap();
            }
        }
    }

    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::info!("idle");

        loop {
            continue;
        }
    }
}
