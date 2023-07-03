use stm32f4xx_hal;
use fugit::Rate;

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

use stm32f4xx_hal::{
	pac,
	otg_fs::{USB, UsbBus}
};

static mut EP_MEMORY: [u32; 1024] = [0; 1024];

// From this abomination of an example 
// https://github.com/stm32-rs/stm32f1xx-hal/blob/master/examples/usb_serial_rtic.rs
static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<stm32f4xx_hal::otg_fs::UsbBusType>> = None;

// Container for all peripherals needed to setup an USB OTG_FS device
pub struct USBPeripherals {
	pub otg_fs_global: pac::OTG_FS_GLOBAL,
	pub otg_fs_device: pac::OTG_FS_DEVICE,
	pub otg_fs_pwclk: pac::OTG_FS_PWRCLK,
	pub usb_dm: stm32f4xx_hal::gpio::Pin<'A', 11>,
	pub usb_dp: stm32f4xx_hal::gpio::Pin<'A', 12>,
	pub hclk: Rate<u32, 1, 1>
}

// Container for the different handlers needed to integrate with USB
#[allow(dead_code)]
pub struct USBHandler<'a> {
	pub usb_dev: usb_device::prelude::UsbDevice<'a, UsbBus<USB>>, 
	pub usb_audio: usbd_audio::AudioClass<'a, UsbBus<USB>>
}

// Create a `usb_bus` that is stored statically in a bullshit way. Then create `usb_dev` and a
// `usb_audio` instances and return them as an USBHandler struct instance
pub fn init(usb_peripherals: USBPeripherals) -> USBHandler<'static>{
	let usb = USB {
		  usb_global: usb_peripherals.otg_fs_global,
		  usb_device: usb_peripherals.otg_fs_device,
		  usb_pwrclk: usb_peripherals.otg_fs_pwclk,
		  pin_dp: stm32f4xx_hal::gpio::alt::otg_fs::Dp::PA12(usb_peripherals.usb_dp.into_alternate()),
		  pin_dm: stm32f4xx_hal::gpio::alt::otg_fs::Dm::PA11(usb_peripherals.usb_dm.into_alternate()),
		  hclk: usb_peripherals.hclk
	};

	// TODO: Change this to use the rtic 2.0 functionality for lifetime objects
	unsafe {
		USB_BUS.replace(UsbBus::new(usb, &mut EP_MEMORY));
	}

	let usb_audio = AudioClassBuilder::new()
		.input(
			StreamConfig::new_discrete(
			// Signed 24 bytes little endian
			Format::S24le,
			2,
			&[48000],
			TerminalType::InMicrophone).unwrap())
		.output(
			StreamConfig::new_discrete(
			Format::S24le,
			2,
			&[48000],
			TerminalType::OutSpeaker).unwrap())
		.build(unsafe { USB_BUS.as_ref().unwrap() })
		.unwrap();

		let usb_dev: usb_device::prelude::UsbDevice<UsbBus<USB>> = UsbDeviceBuilder::new(unsafe { USB_BUS.as_ref().unwrap() }, UsbVidPid(0x16c0, 0x27dd))
			.manufacturer("Josef Labs")
			.product("USB audio test")
			.serial_number("42")
			.build();

	USBHandler {
		usb_dev,
		usb_audio
	}

}
