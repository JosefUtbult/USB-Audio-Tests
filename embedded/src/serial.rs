use super::*;
use fugit::Rate;
use rtic_monotonics::systick::*;

use hal::{
    device::{USART1, USART2, USART3},
    gpio::Pin,
    rcc::{rec::{Usart1, Usart2, Usart3}, CoreClocks}, serial::Tx
};
use hal::serial::SerialExt;

pub struct SerialPeripherals {

    pub usart1_dev: USART1,
    pub usart2_dev: USART2,
    pub usart3_dev: USART3,
    pub usart1_rec: Usart1,
    pub usart2_rec: Usart2,
    pub usart3_rec: Usart3,
	pub clocks: CoreClocks,
    pub clock_rate: Rate<u32, 1, 1>,
    pub tx1: Pin<'A', 9>,
    pub rx1: Pin<'A', 10>,
    pub tx2: Pin<'D', 5>,
    pub rx2: Pin<'D', 6>,
    pub tx3: Pin<'C', 10>,
    pub rx3: Pin<'C', 11>,
}

pub struct Serial {
    pub tx1: Tx<USART1>,
    pub tx2: Tx<USART2>,
    pub tx3: Tx<USART3>,
}

pub fn init(serial_peripherals: SerialPeripherals) -> Serial {

    let serial1_pins = (
        serial_peripherals.tx1.into_alternate(), 
        serial_peripherals.rx1.into_alternate()
    );

    let serial2_pins = (
        serial_peripherals.tx2.into_alternate(), 
        serial_peripherals.rx2.into_alternate()
    );

    let serial3_pins = (
        serial_peripherals.tx3.into_alternate(), 
        serial_peripherals.rx3.into_alternate()
    );

    let serial1 = serial_peripherals.usart1_dev
        .serial(
            serial1_pins, 
            serial_peripherals.clock_rate, 
            serial_peripherals.usart1_rec,
            &serial_peripherals.clocks
        )
        .unwrap();

    let serial2 = serial_peripherals.usart2_dev
        .serial(
            serial2_pins, 
            serial_peripherals.clock_rate, 
            serial_peripherals.usart2_rec,
            &serial_peripherals.clocks
        )
        .unwrap();

    let serial3 = serial_peripherals.usart3_dev
        .serial(
            serial3_pins, 
            serial_peripherals.clock_rate, 
            serial_peripherals.usart3_rec,
            &serial_peripherals.clocks
        )
        .unwrap();

    let (tx1, _rx1) = serial1.split();
    let (tx2, _rx2) = serial2.split();
    let (tx3, _rx3) = serial3.split();

    Serial {
        tx1,
        tx2,
        tx3
    }

}