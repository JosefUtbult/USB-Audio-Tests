use super::hal;
use hal::pac;

use usb_device::device::{
	UsbDeviceBuilder, 
	UsbVidPid
};

use usbd_audio::{
	AudioClassBuilder, 
	StreamConfig, 
	TerminalType, 
	Format
};

#[allow(unused_imports)]
use hal::{
	prelude::*,        
	gpio::{Output, PushPull, gpioe::PE1},
	rcc::{CoreClocks, rec::UsbClkSel},
	usb_hs::{UsbBus, USB1}
};

pub const USB_BUFFER_SIZE: usize = 1024;
// pub const USB_BUFFER_SIZE: usize = 100;
static mut EP_MEMORY: [u32; USB_BUFFER_SIZE] = [0; USB_BUFFER_SIZE];

// Container for all peripherals needed to setup an USB OTG HS device
pub struct USBPeripherals {
	pub otg_hs_global: pac::OTG1_HS_GLOBAL,
	pub otg_hs_device: pac::OTG1_HS_DEVICE,
	pub otg_hs_pwclk: pac::OTG1_HS_PWRCLK,
	pub usb_dm: hal::gpio::Pin<'B', 14>,
	pub usb_dp: hal::gpio::Pin<'B', 15>,
	pub prec: hal::rcc::rec::Usb1Otg,
	pub clocks: CoreClocks
}
pub struct USBHandler<'a> {
	pub usb_dev: usb_device::prelude::UsbDevice<'a, UsbBus<USB1>>, 
	pub usb_audio: usbd_audio::AudioClass<'a, UsbBus<USB1>>
}

// Create a `usb_bus` that is stored statically in a bullshit way. Then create `usb_dev` and a
// `usb_audio` instances and return them as an USBHandler struct instance
pub fn init(usb_peripherals: USBPeripherals) -> USBHandler<'static>{
	let usb = USB1::new(
		usb_peripherals.otg_hs_global,
		usb_peripherals.otg_hs_device,
		usb_peripherals.otg_hs_pwclk,
		usb_peripherals.usb_dm.into_alternate(),
		usb_peripherals.usb_dp.into_alternate(),
		usb_peripherals.prec,
		&usb_peripherals.clocks
	);

	let usb_bus = cortex_m::singleton!(
		: usb_device::class_prelude::UsbBusAllocator<UsbBus<USB1>> =
			UsbBus::new(usb, unsafe { &mut EP_MEMORY })
	)
	.unwrap();

	let usb_audio = AudioClassBuilder::new()
	// .input(
		// 	StreamConfig::new_discrete(
		// 	// Signed 24 bit little endian
		// 	Format::S24le,
		// 	1,
		// 	&[48000],
		// 	TerminalType::InMicrophone).unwrap())
	.output(
		StreamConfig::new_discrete(
		// Signed 16 bit little endian
		Format::S16le,
		1,
		&[8000],
		// &[48000],
		TerminalType::OutSpeaker).unwrap())
	.build(usb_bus)
	.unwrap();

	let usb_dev: usb_device::prelude::UsbDevice<UsbBus<USB1>> = 
		UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
			.manufacturer("Josef Labs")
			.product("USB audio test")
			.serial_number("42")
			.build();

	USBHandler {
		usb_dev,
		usb_audio
	}
}
