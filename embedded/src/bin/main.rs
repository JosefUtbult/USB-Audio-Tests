#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use usb_audio_tests as _; // global logger + panicking-behavior + memory layout
use stm32h7xx_hal as hal;
use heapless::spsc::{Queue, Producer, Consumer};

// const FM_MODIFIER: u8 = 8;
type QueueType = [u32; 2];
const QUEUE_SIZE: usize = 48 * 5;

const TEST: bool = true;

#[rtic::app(
    device = hal::pac,
    dispatchers = [EXTI0, EXTI1, EXTI2]
)]
mod app {
    use hal::device::quadspi::cr::DFM_R;
    #[allow(unused_imports)]
    use usb_audio_tests::*;
    use usb_audio_tests::debug_gpio;
    use usb_audio_tests::codec;
    
    use super::*;
    #[allow(unused_imports)]
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
        debug_handler: debug_gpio::DebugHandler,
        fs_counter: u16,
    }
    
    // Local resources go here
    #[local]
    struct Local {
        usb_out_producer: Producer<'static, QueueType, QUEUE_SIZE>,
        usb_out_consumer: Consumer<'static, QueueType, QUEUE_SIZE>,
        codec_container: codec::SaiContainer,
        tx1: hal::serial::Tx<hal::stm32::USART3>,
        tx2: hal::serial::Tx<hal::stm32::USART2>,
        usb_handler: usb::USBHandler<'static>,
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

    #[init(
        local = [
            usb_out_queue: Queue<QueueType, QUEUE_SIZE> = Queue::new()
        ]
    )]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("Init");
        defmt::info!("Res: {:#010b}", sample_rate_to_buffer(513.5009765625, true));
        
        let (mut usb_out_producer, usb_out_consumer) = cx.local.usb_out_queue.split();

        // for i in 0..QUEUE_SIZE / 2 {
        //     usb_out_producer.enqueue([0, 0]).ok();
        // }

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

        let mut usb_handler = usb::init(usb_peripherals);

        // Debug pins setup
        let debug_gpio_pins = debug_gpio::DebugGPIO {
            usb_interrupt: gpiob.pb8,
            usb_audio_packet_interrupt: gpiob.pb9,
            codec_1_interrupt: gpioa.pa5,
            codec_2_interrupt: gpioa.pa6
        };

        let mut debug_handler = debug_gpio::init(debug_gpio_pins);

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

        let mut buf = [0u8; usb::USB_BUFFER_SIZE];
        loop {
            if usb_handler.usb_dev.poll(&mut [&mut usb_handler.usb_audio]) {
                if let Ok(len) = usb_handler.usb_audio.read(&mut buf) {
                    debug_gpio::toggle_usb_interrupt(&mut debug_handler);
                    for i in 0..len/2 {
                        let sample: u32 = (buf[i * 2] as u32) << 8 | buf[i * 2 + 1] as u32;
                        match usb_out_producer.enqueue([sample, sample]).ok() {
                            Some(_res) => {},
                            None => {
                                debug_gpio::toggle_usb_audio_packet_interrupt(&mut debug_handler);
                                if TEST {
                                    defmt::info!("Unable to push sample from USB");
                                }
                            }
                        }
                    }
                    break;
                }
                else {
                    // Kickstart the sync interrupt
                    usb_handler.usb_audio.write_synch_interrupt(&sample_rate_to_buffer(0.0, false)).ok();
                }
            }
        }

        // Enable SAI interrupt
        codec::enable(&mut codec_container);
        defmt::info!("Starting");

        (
            Shared {
                debug_handler,
                fs_counter: 0,
            },
            Local {
                usb_out_producer,
                usb_out_consumer,
                codec_container: codec_container,
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
            debug_handler,
            fs_counter,
        ],
        local = [
                codec_container,
                usb_out_consumer,
        ]
    )]
    fn sai_task(mut cx: sai_task::Context) {
        // defmt::info!("Running SAI task");

        cx.shared.fs_counter.lock(|fs_counter| {
            *fs_counter += 1;
        }); 
        
        cx.shared.debug_handler.lock(|debug_handler| {
            debug_gpio::toggle_codec_1_interrupt(debug_handler);
        });
        
        let _res = hal::traits::i2s::FullDuplex::try_read(&mut cx.local.codec_container.0).ok();
        match cx.local.usb_out_consumer.dequeue() {
            Some(_sample) => {
                // hal::traits::i2s::FullDuplex::try_send(&mut cx.local.codec_container.0, sample[0], sample[1]).unwrap()
            }
            None => {
                // defmt::info!("Unable to get sample from USB");
                // hal::traits::i2s::FullDuplex::try_send(&mut cx.local.codec_container.0, 0, 0).unwrap();
            }

        }
        // if let Ok((left, right)) = hal::traits::i2s::FullDuplex::try_read(&mut cx.local.codec_container.0) {
            // hal::traits::i2s::FullDuplex::try_send(&mut cx.local.codec_container.0, left, right).unwrap();
        // }
    }

    #[task(
        binds= OTG_HS,
        priority = 5,
        local = [
            usb_out_producer,
            tx1,
            tx2, 
            usb_handler, 
            counter: u16 = 0,
            first_interrupt: bool = true
        ],
        shared = [
            debug_handler, 
            fs_counter,
        ]
    )]
    fn USB_interrupt(mut cx: USB_interrupt::Context) {
        let mut frame_len: Option<usize> = None;
        let mut modifier: Option<f64> = None;
        
        cx.shared.debug_handler.lock(|debug_handler| {
            debug_gpio::toggle_usb_interrupt(debug_handler);
        });

        if cx.local.usb_handler.usb_dev.poll(&mut [&mut cx.local.usb_handler.usb_audio]) {
            let mut buf = [0u8; usb::USB_BUFFER_SIZE];
            if let Ok(len) = cx.local.usb_handler.usb_audio.read(&mut buf) {
                cx.shared.debug_handler.lock(|debug_handler| {
                    debug_gpio::toggle_usb_audio_packet_interrupt(debug_handler);
                });
                if *cx.local.first_interrupt {
                    *cx.local.first_interrupt = false;
                }
                frame_len = Some(len);
                for i in 0..len/2 {
                    let sample: u32 = (buf[i * 2] as u32) << 8 | buf[i * 2 + 1] as u32;
                    match cx.local.usb_out_producer.enqueue([sample, sample]).ok() {
                        Some(_res) => {},
                        None => {
                            // cx.shared.debug_handler.lock(|debug_handler| {
                            //     debug_gpio::toggle_usb_audio_packet_interrupt(debug_handler);
                            // });
                            if TEST {
                                defmt::info!("Unable to push sample from USB");
                            }
                        }
                    }
                }
            }
            // If no new data is present, the interrupt is probably caused
            // by the Ff rate being read
            else {
                cx.shared.fs_counter.lock(|fs_counter| {
                    let rate = *fs_counter as f64 - 1.0;
                    *fs_counter = 0;
                    if *cx.local.first_interrupt {
                        cx.local.usb_handler.usb_audio.write_synch_interrupt(&sample_rate_to_buffer(0.0, false)).ok();
                    }
                    else {
                        // let rate = 48.1;
                        cx.local.usb_handler.usb_audio.write_synch_interrupt(&sample_rate_to_buffer(rate, false)).ok();
                        modifier = Some(rate);
                    }

                });
                    
                // writeln!(cx.local.tx2, "{modifier:.2}").unwrap();
            }
        }
        
        if let Some(_var) = frame_len {
            // writeln!(cx.local.tx1, "{}", var).ok();
            writeln!(cx.local.tx1, "{}", cx.local.usb_out_producer.len()).ok();
        }
        
        if let Some(var) = modifier {
            writeln!(cx.local.tx2, "{}", var).ok();
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
