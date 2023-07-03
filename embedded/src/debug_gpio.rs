use stm32f4xx_hal::gpio::{
	Pin,
	Output
};

pub struct DebugGPIO {
	pub pc3: Pin<'C', 3>
}

pub struct DebugHandler {
	pub usb_interrupt_pin: Pin<'C', 3, Output>
}

pub fn init(debug_gpio: DebugGPIO) -> DebugHandler {
	
	let mut usb_interrupt_pin = debug_gpio.pc3.into_push_pull_output();
	usb_interrupt_pin.set_low();

	DebugHandler {
		usb_interrupt_pin
	}
}

pub fn toggle(debug_handler: &mut DebugHandler) {
	debug_handler.usb_interrupt_pin.toggle();
}
