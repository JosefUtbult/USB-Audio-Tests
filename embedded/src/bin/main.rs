#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![allow(dead_code)]

use usb_audio_tests as _; // global logger + panicking-behavior + memory layout
use stm32h7xx_hal as hal;
use heapless::spsc::{Queue, Producer, Consumer};

// const FM_MODIFIER: u8 = 8;
type QueueType = [u32; 2];
const QUEUE_SIZE: usize = 48 * 5;

#[rtic::app(
    device = hal::pac,
    dispatchers = [EXTI0, EXTI1, EXTI2]
)]
mod app {
    #[allow(unused_imports)]
    use usb_audio_tests::*;
    use usb_audio_tests::debug_gpio;
    use usb_audio_tests::codec;
    use usb_audio_tests::serial;
    
    use super::*;
    #[allow(unused_imports)]
    use num_traits::real::Real;

    #[allow(unused_imports)]
    use hal::{
        prelude::*,        
        stm32,
        gpio::{Pin, Output, PushPull, gpioe::PE1, Edge},
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
        fs_counter: u32,
    }
    
    // Local resources go here
    #[local]
    struct Local {
        usb_out_producer: Producer<'static, QueueType, QUEUE_SIZE>,
        usb_out_consumer: Consumer<'static, QueueType, QUEUE_SIZE>,
        serial: serial::Serial,
        codec_container: codec::SaiContainer,
        usb_handler: usb::USBHandler<'static>,
        usb_interrupt_pin: debug_gpio::UsbInterruptPin,
        usb_producer_cant_push_pin: debug_gpio::UsbProducerCantPush,
        codec_consumer_cant_pull_pin: debug_gpio::CodecConsumerCantPull,
        codec_interrupt_pin: debug_gpio::CodecInterruptPin,
        external_interrupt_pin: Pin<'C', 13, hal::gpio::Input>
    }

    fn sample_rate_to_buffer(rate: f64, debug: bool) -> [u8; 3] {
        let integer: u16 = rate as u16;
        let fraction: f64 = rate - rate as i64 as f64;
        let base_2_fraction: u16 = (fraction * 1024_f64) as u16;
        if debug {
            defmt::info!("Integer: {}", integer);
            defmt::info!("Base 2 integer: {:#012b}", integer);
            defmt::info!("Fraction: {}", fraction);
            defmt::info!("Base 2 fraction: {:#012b}", base_2_fraction);
        }
        [(integer >> 2) as u8, (integer << 6) as u8 | (base_2_fraction >> 4) as u8, (base_2_fraction << 4) as u8]
    }

    /// Handles if a USB poll event is a result of new packet of
    /// sampling data.
    /// It checks if new data is present, toggle the corresponding
    /// debug pin, applies the new packet length for debugging and
    /// tries to enqueue this new data in the queue.
    fn handle_usb_poll<'a>(
        usb_handler: &mut usb::USBHandler<'a>, 
        usb_out_producer: &mut Producer<'static, QueueType, QUEUE_SIZE>,
        frame_len: &mut Option<usize>,
        usb_producer_cant_push_pin: &mut debug_gpio::UsbProducerCantPush
    ) -> bool {
        let mut buf = [0u8; usb::USB_BUFFER_SIZE];
        if let Ok(len) = usb_handler.usb_audio.read(&mut buf) {
            
            *frame_len = Some(len);
            for i in 0..len/2 {
                let sample: u32 = (buf[i * 2] as u32) << 8 | buf[i * 2 + 1] as u32;
                match usb_out_producer.enqueue([sample, sample]).ok() {
                    Some(_res) => {},
                    None => {
                        usb_producer_cant_push_pin.toggle();
                        
                        #[cfg(debug_assertions)]
                        defmt::info!("Unable to push sample from USB");
                    }
                }
            }
            return true;
        }
        false
    }

    #[init(
        local = [
            usb_out_queue: Queue<QueueType, QUEUE_SIZE> = Queue::new()
        ]
    )]
    fn init(mut cx: init::Context) -> (Shared, Local) {
        defmt::info!("Init");
        defmt::info!("Res: {:#010b}", sample_rate_to_buffer(513.5009765625, true));
        
        let (mut usb_out_producer, usb_out_consumer) = cx.local.usb_out_queue.split();

        // Initialize the monotonic timer at 8 MHz
        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, 48_000_000, systick_mono_token); // default STM32F4 clock rate at 8 MHz


        let pwr = cx.device.PWR.constrain();
        let pwrcfg = pwr.freeze();

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
        let serial_pins = serial::SerialPeripherals {
            usart1_dev: cx.device.USART1,
            usart2_dev: cx.device.USART2,
            usart3_dev: cx.device.USART3,
            usart1_rec: ccdr.peripheral.USART1,
            usart2_rec: ccdr.peripheral.USART2,
            usart3_rec: ccdr.peripheral.USART3,
            clocks: ccdr.clocks,
            clock_rate: 921_600.bps(),
            tx1: gpioa.pa9,
            rx1: gpioa.pa10,
            tx2: gpiod.pd5,
            rx2: gpiod.pd6,
            tx3: gpioc.pc10,
            rx3: gpioc.pc11,
        };

        let serial = serial::init(serial_pins);

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

        let mut usb_handler = usb::init(usb_peripherals);

        // Debug pins setup
        let debug_gpio_pins = debug_gpio::DebugGPIO {
            usb_interrupt: gpiob.pb8,
            usb_producer_cant_push: gpiob.pb9,
            codec_consumer_cant_pull: gpioa.pa5,
            codec_interrupt: gpioa.pa6
        };

        let (
            usb_interrupt_pin,
            mut usb_producer_cant_push_pin,
            codec_consumer_cant_pull_pin,
            codec_interrupt_pin, 
        ) = debug_gpio::init(debug_gpio_pins);

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

        let mut codec_container = codec::init(codec);

        // for _i in 0..3*48 {
        //     usb_out_producer.enqueue([0, 0]).ok();
        // }

        let mut external_interrupt_pin: Pin<'C', 13, hal::gpio::Input> = gpioc.pc13.into_floating_input();
        hal::gpio::ExtiPin::trigger_on_edge(&mut external_interrupt_pin, &mut cx.device.EXTI, Edge::Rising);
        hal::gpio::ExtiPin::make_interrupt_source(&mut external_interrupt_pin, &mut cx.device.SYSCFG);

        // Wait until the first USB frame containing samples has
        // arrived. This is to make sure that the SAI interrupts won't
        // trigger in advance
        let mut frame_len: Option<usize> = None;
        loop {
            if usb_handler.usb_dev.poll(&mut [&mut usb_handler.usb_audio]) {
                if handle_usb_poll(
                    &mut usb_handler,
                    &mut usb_out_producer,
                    &mut frame_len,
                    &mut usb_producer_cant_push_pin
                ) {
                    break;
                }
            } 
            else {
                // Kickstart the sync interrupt
                usb_handler.usb_audio.write_synch_interrupt(&sample_rate_to_buffer(0.0, false)).ok();
            }
        }

        // Enable SAI interrupt
        codec::enable(&mut codec_container);
        hal::gpio::ExtiPin::enable_interrupt(&mut external_interrupt_pin, &mut cx.device.EXTI);
        defmt::info!("Starting");

        (
            Shared {
                fs_counter: 0,
            },
            Local {
                usb_out_producer,
                usb_out_consumer,
                codec_container: codec_container,
                serial,
                usb_handler,
                usb_interrupt_pin,
                usb_producer_cant_push_pin,
                codec_consumer_cant_pull_pin,
                codec_interrupt_pin, 
                external_interrupt_pin
            },
        )
    }

    #[task(
        binds = EXTI15_10,
        priority = 7,
        shared = [
            // fs_counter,
        ],
        local = [
            external_interrupt_pin,
            codec_consumer_cant_pull_pin,
        ]
    )]
    fn fs_counter_task(mut cx: fs_counter_task::Context) {
        cx.local.codec_consumer_cant_pull_pin.toggle();
        // cx.shared.fs_counter.lock(|fs_counter| {
        //     *fs_counter += 1;
        // }); 

        hal::gpio::ExtiPin::clear_interrupt_pending_bit(cx.local.external_interrupt_pin);
    }

    #[task(
        binds = SAI1, 
        priority = 6,
        local = [
            codec_container,
            usb_out_consumer,
            // codec_consumer_cant_pull_pin,
            codec_interrupt_pin
        ],
        shared = [
            fs_counter
        ]
    )]
    fn sai_task(mut cx: sai_task::Context) {
        // defmt::info!("Running SAI task");

        cx.local.codec_interrupt_pin.toggle();
        
        cx.shared.fs_counter.lock(|fs_counter| {
            *fs_counter += 1;
        }); 
         
        let _res = hal::traits::i2s::FullDuplex::try_read(&mut cx.local.codec_container.0).ok();
        match cx.local.usb_out_consumer.dequeue() {
            Some(_sample) => {
                // hal::traits::i2s::FullDuplex::try_send(&mut cx.local.codec_container.0, sample[0], sample[1]).unwrap()
            }
            None => {
                // cx.local.codec_consumer_cant_pull_pin.toggle();
                // defmt::info!("Unable to get sample from USB");
                // hal::traits::i2s::FullDuplex::try_send(&mut cx.local.codec_container.0, 0, 0).unwrap();
            }

        }
        // if let Ok((left, right)) = hal::traits::i2s::FullDuplex::try_read(&mut cx.local.codec_container.0) {
            // hal::traits::i2s::FullDuplex::try_send(&mut cx.local.codec_container.0, left, right).unwrap();
        // }
    }

    /// Clear the Fs counter on USB wake up
    #[task(
        binds=OTG_HS_WKUP,
        priority = 6,
        shared = [
            fs_counter
        ]
    )]
    fn USB_wakeup_interrupt(mut cx: USB_wakeup_interrupt::Context) {
        cx.shared.fs_counter.lock(|fs_counter| {
            *fs_counter = 0;
        });
    }

    #[task(
        binds= OTG_HS,
        priority = 5,
        local = [
            usb_out_producer,
            serial,
            usb_handler, 
            usb_interrupt_pin,
            usb_producer_cant_push_pin,            
            counter: u16 = 0
        ],
        shared = [
            fs_counter,
        ]
    )]
    fn USB_interrupt(mut cx: USB_interrupt::Context) {
        let mut frame_len: Option<usize> = None;
        let mut modifier: Option<f64> = None;
        
        // Check for an usb poll, which should have happened, as the
        // interrupt has occurred
        if cx.local.usb_handler.usb_dev.poll(&mut [&mut cx.local.usb_handler.usb_audio]) {
            cx.local.usb_interrupt_pin.toggle();

            // If no new data is present, the interrupt is probably caused
            // by the Ff rate being read
            if !handle_usb_poll(
                cx.local.usb_handler,
                cx.local.usb_out_producer,
                &mut frame_len,
                cx.local.usb_producer_cant_push_pin
            ) {
                let mut rate: f64 = 0.0;
                cx.shared.fs_counter.lock(|fs_counter| {
                    rate = *fs_counter as f64;
                    *fs_counter = 0;
                });

                cx.local.usb_handler.usb_audio.write_synch_interrupt(&sample_rate_to_buffer(rate, false)).ok();
                modifier = Some(rate);
            }
        }
        
        if let Some(var) = frame_len {
            writeln!(cx.local.serial.tx1, "{}", var).ok();
            writeln!(cx.local.serial.tx2, "{}", cx.local.usb_out_producer.len()).ok();
        }
        
        if let Some(var) = modifier {
            writeln!(cx.local.serial.tx3, "{:.2}", var).ok();
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
