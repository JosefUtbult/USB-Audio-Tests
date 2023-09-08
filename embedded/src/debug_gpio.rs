use stm32h7xx_hal::gpio::{
	Pin,
	Output
};

pub struct DebugGPIO {
	pub usb_interrupt: Pin<'B', 8>,
    pub usb_audio_packet_interrupt: Pin<'B', 9>,
	pub codec_1_interrupt: Pin<'A', 5>,
	pub codec_2_interrupt: Pin<'A', 6>
}

pub struct DebugHandler {
	pub usb_interrupt_pin: Pin<'B', 8, Output>,
	pub usb_audio_packet_interrupt_pin: Pin<'B', 9, Output>,
	pub codec_1_interrupt_pin: Pin<'A', 5, Output>,
	pub codec_2_interrupt_pin: Pin<'A', 6, Output> 
}

pub fn init(debug_gpio: DebugGPIO) -> DebugHandler {
	
	let mut usb_interrupt_pin = debug_gpio.usb_interrupt.into_push_pull_output();
	let mut usb_audio_packet_interrupt_pin = debug_gpio.usb_audio_packet_interrupt.into_push_pull_output();
	let mut codec_1_interrupt_pin = debug_gpio.codec_1_interrupt.into_push_pull_output();
	let mut codec_2_interrupt_pin = debug_gpio.codec_2_interrupt.into_push_pull_output();

	usb_interrupt_pin.set_low();
    usb_audio_packet_interrupt_pin.set_low();

	codec_1_interrupt_pin.set_low();
	codec_2_interrupt_pin.set_low();

	DebugHandler {
		usb_interrupt_pin,
        usb_audio_packet_interrupt_pin,
		codec_1_interrupt_pin,
		codec_2_interrupt_pin
	}
}

pub fn toggle_usb_interrupt(debug_handler: &mut DebugHandler) {
	debug_handler.usb_interrupt_pin.toggle();
}

pub fn toggle_usb_audio_packet_interrupt(debug_handler: &mut DebugHandler) {
	debug_handler.usb_audio_packet_interrupt_pin.toggle();
}

pub fn toggle_codec_1_interrupt(debug_handler: &mut DebugHandler) {
	debug_handler.codec_1_interrupt_pin.toggle();
}

pub fn toggle_codec_2_interrupt(debug_handler: &mut DebugHandler) {
	debug_handler.codec_2_interrupt_pin.toggle();
}
