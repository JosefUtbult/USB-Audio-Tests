use stm32h7xx_hal::gpio::{
	Pin,
	Output
};

pub struct DebugGPIO {
	pub usb_interrupt: Pin<'C', 8>,
    pub usb_audio_packet_interrupt: Pin<'C', 9>
}

pub struct DebugHandler {
	pub usb_interrupt_pin: Pin<'C', 8, Output>,
	pub usb_audio_packet_pin: Pin<'C', 9, Output>
}

pub fn init(debug_gpio: DebugGPIO) -> DebugHandler {
	
	let mut usb_interrupt_pin = debug_gpio.usb_interrupt.into_push_pull_output();
	let mut usb_audio_packet_interrupt_pin = debug_gpio.usb_audio_packet_interrupt.into_push_pull_output();

	usb_interrupt_pin.set_low();
    usb_audio_packet_interrupt_pin.set_low();

	DebugHandler {
		usb_interrupt_pin,
        usb_audio_packet_pin: usb_audio_packet_interrupt_pin
	}
}

pub fn toggle_usb_interrupt(debug_handler: &mut DebugHandler) {
	debug_handler.usb_interrupt_pin.toggle();
}

pub fn toggle_usb_audio_packet_interrupt(debug_handler: &mut DebugHandler) {
	debug_handler.usb_audio_packet_pin.toggle();
}
