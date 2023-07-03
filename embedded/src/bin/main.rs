#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use usb_audio_tests as _; // global logger + panicking-behavior + memory layout

#[rtic::app(
	device = stm32f4xx_hal::pac,
	dispatchers = [EXTI0, EXTI1, EXTI2]
)]
mod app {

	use usb_audio_tests::*;
	use stm32f4xx_hal::prelude::*;

	// Shared resources go here
	#[shared]
	struct Shared {
		// TODO: Add resources
	}

	// Local resources go here
	#[local]
	struct Local {
		usb_handler: usb::USBHandler<'static>,
		debug_handler: debug_gpio::DebugHandler
	}

	#[init]
	fn init(cx: init::Context) -> (Shared, Local) {
		defmt::info!("init");
		
		let device = cx.device;
		let gpioa = device.GPIOA.split();
		//let gpiob = device.GPIOB.split();
		let gpioc = device.GPIOC.split();

		// TODO setup monotonic if used
		// let sysclk = { /* clock setup + returning sysclk as an u32 */ };
		// let token = rtic_monotonics::create_systick_token!();
		// rtic_monotonics::systick::Systick::new(cx.core.SYST, sysclk, token);
		
		let rcc = device.RCC.constrain();

		let clocks = rcc
			.cfgr
			.use_hse(8.MHz()) // Change this to the speed of the external crystal you're using
			.sysclk(48.MHz())
			.require_pll48clk()
			.freeze();

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
			pc3: gpioc.pc3,
		};

		let debug_handler = debug_gpio::init(debug_gpio);

		(
			Shared {
				// Initialization of shared resources go here
			},
			Local {
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
		priority = 5,
		// TODO: Change from general interrupt to correct one
		binds = OTG_FS,
		local = [usb_handler, debug_handler]
	)]
	fn USB_interrupt(cx: USB_interrupt::Context) {
		if cx.local.usb_handler.usb_dev.poll(&mut [&mut cx.local.usb_handler.usb_audio]) {
			let mut buf = [0u8; 1024];
			if let Ok(_len) = cx.local.usb_handler.usb_audio.read(&mut buf) {
				debug_gpio::toggle(cx.local.debug_handler);
				//defmt::info!("Usb Interrupt");
			}
		}
	}
}
