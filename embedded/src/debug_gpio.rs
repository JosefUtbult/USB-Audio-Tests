use stm32h7xx_hal::gpio::{
	Pin,
	Output
};

pub struct DebugGPIO {
	pub usb_interrupt: Pin<'B', 8>,
    pub usb_producer_cant_push: Pin<'B', 9>,
	pub codec_consumer_cant_pull: Pin<'A', 5>,
	pub codec_interrupt: Pin<'A', 6>
}

pub type UsbInterruptPin = Pin<'B', 8, Output>;
pub type UsbProducerCantPush = Pin<'B', 9, Output>;
pub type CodecConsumerCantPull = Pin<'A', 5, Output>;
pub type CodecInterruptPin = Pin<'A', 6, Output>;


pub struct DebugHandler {
	pub usb_interrupt_pin: UsbInterruptPin,
	pub usb_producer_cant_push_pin: UsbProducerCantPush,
	pub codec_consumer_cant_pull_pin: CodecConsumerCantPull,
	pub codec_interrupt_pin: CodecInterruptPin 
}

pub fn init(debug_gpio: DebugGPIO) -> (
	UsbInterruptPin,
	UsbProducerCantPush,
	CodecConsumerCantPull,
	CodecInterruptPin
) {
	
	let mut usb_interrupt_pin = debug_gpio.usb_interrupt.into_push_pull_output();
	let mut usb_producer_cant_push_pin = debug_gpio.usb_producer_cant_push.into_push_pull_output();
	let mut codec_consumer_cant_pull_pin = debug_gpio.codec_consumer_cant_pull.into_push_pull_output();
	let mut codec_interrupt_pin = debug_gpio.codec_interrupt.into_push_pull_output();

	usb_interrupt_pin.set_low();
    usb_producer_cant_push_pin.set_low();

	codec_consumer_cant_pull_pin.set_low();
	codec_interrupt_pin.set_low();

	(
		usb_interrupt_pin,
        usb_producer_cant_push_pin,
		codec_consumer_cant_pull_pin,
		codec_interrupt_pin
	)
}
