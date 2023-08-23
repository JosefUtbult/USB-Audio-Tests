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
    use usb_audio_tests::codec;
    
    use super::*;

    #[allow(unused_imports)]
    use hal::{
        prelude::*,        
        stm32,
        gpio::{Output, PushPull, gpioe::PE1},
        rcc::rec::UsbClkSel,
        usb_hs::{UsbBus, USB1},
    };

    use rtic_monotonics::systick::*;
    use heapless::Vec;

    #[allow(unused_imports)]
    use core::fmt::Write; // for pretty formatting of the serial output

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Move to local
        #[lock_free]
        codec_container: codec::SaiContainer
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
        // TODO: Look over clock configuration
        let rcc = cx.device.RCC.constrain();
        let mut ccdr = rcc
            .sys_ck(80.MHz())
            // Used for I2S master clock
            .pll3_p_ck(codec::PLL3_P_HZ)
            .freeze(pwrcfg, &cx.device.SYSCFG);

        // 48MHz CLOCK
        let _ = ccdr.clocks.hsi48_ck().expect("HSI48 must run");
        ccdr.peripheral.kernel_usb_clk_mux(UsbClkSel::Hsi48);

        let gpioa = cx.device.GPIOA.split(ccdr.peripheral.GPIOA); 
        let gpiob = cx.device.GPIOB.split(ccdr.peripheral.GPIOB); 
        let gpioc = cx.device.GPIOC.split(ccdr.peripheral.GPIOC);
        let gpiod = cx.device.GPIOD.split(ccdr.peripheral.GPIOD);
        let gpioe = cx.device.GPIOE.split(ccdr.peripheral.GPIOE);

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

        let debug_gpio_pins = debug_gpio::DebugGPIO {
            usb_interrupt: gpiob.pb8,
            usb_audio_packet_interrupt: gpiob.pb9
        };

        let debug_handler = debug_gpio::init(debug_gpio_pins);
        let usb_handler = usb::init(usb_peripherals);

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
                codec_container: codec_container
            },
            Local {
                tx,
                usb_handler,
                debug_handler
            },
        )
    }

    #[task(binds = SAI1, shared = [codec_container] )]
    fn passthru1(cx: passthru1::Context) {
        if let Ok((left, right)) = hal::traits::i2s::FullDuplex::try_read(&mut cx.shared.codec_container.0) {
            hal::traits::i2s::FullDuplex::try_send(&mut cx.shared.codec_container.0, left, right).unwrap();
        }
    }

    #[task(binds = SAI2, shared = [codec_container] )]
    fn passthru2(cx: passthru2::Context) {
        if let Ok((left, right)) = hal::traits::i2s::FullDuplex::try_read(&mut cx.shared.codec_container.1) {
            hal::traits::i2s::FullDuplex::try_send(&mut cx.shared.codec_container.1, left, right).unwrap();
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
    //#[task(
        //priority = 5,
        //local = [tx, usb_handler, debug_handler, buffer: Vec<u8, 0x1000> = Vec::new()]
    //)]
    #[task(
        binds= OTG_HS,
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
                    writeln!(cx.local.tx, "{len}").unwrap();
                    for i in 0..len/2 {
                        let val: u16 = u16::from_le_bytes(buf[i*2..i*2+2].try_into().unwrap());
                        if val != 0 {
                            writeln!(cx.local.tx, "{val}").unwrap();
                        }
                    }
                }
            }
			//Systick::delay(100.nanos()).await;
        //}
    }
}
