use cortex_m::{asm, prelude::_embedded_hal_blocking_i2c_Write};

use super::*;

use hal::{
    stm32,
    time::Hertz,
    gpio::Output,
    device::{SAI1, SAI2},
    sai::SaiI2sExt, rcc::CoreClocks, traits::i2s::FullDuplex, i2c::I2c, prelude::_stm32h7xx_hal_i2c_I2cExt
};

pub const AUDIO_SAMPLE_HZ: Hertz = Hertz::from_raw(48_000);
// Using PLL3_P for SAI1 clock
// The rate should be equal to sample rate * 256
// But not less than so targetting 257
pub const PLL3_P_HZ: Hertz = Hertz::from_raw(AUDIO_SAMPLE_HZ.raw() * 257);

pub type SaiContainer = (hal::sai::Sai<stm32::SAI1, hal::sai::I2S>, hal::sai::Sai<stm32::SAI2, hal::sai::I2S>);

pub struct Codec {
    pub reset: hal::gpio::Pin<'B', 11, Output>,
    pub sai1_mclk: hal::gpio::Pin<'E', 2>,
    pub sai1_sck: hal::gpio::Pin<'E', 5>,
    pub sai1_lrclk: hal::gpio::Pin<'E', 4>,
    pub sai1_dout: hal::gpio::Pin<'E', 6>,
    pub sai1_din: hal::gpio::Pin<'E', 3>,
    pub sai2_sck: hal::gpio::Pin<'D', 13>,
    pub sai2_lrckl: hal::gpio::Pin<'D', 12>,
    pub sai2_mclk: hal::gpio::Pin<'E', 0>,
    pub sai2_dout: hal::gpio::Pin<'D', 11>,
    pub sai2_din: hal::gpio::Pin<'A', 0>,
    pub i2c1_scl: hal::gpio::Pin<'B', 6>,
    pub i2c1_sda: hal::gpio::Pin<'B', 7>,
    pub sai1_rec: hal::rcc::rec::Sai1,
    pub sai2_rec: hal::rcc::rec::Sai2,
    pub sai1_dev: SAI1,
    pub sai2_dev: SAI2,
    pub clocks: CoreClocks,
    pub scb: hal::pac::SCB,
    pub i2c1: hal::pac::I2C1,
    pub i2c1_peripheral: hal::rcc::rec::I2c1,
}

// Why is this something I need to create myself?...
macro_rules! i2c_error_to_string {
    ($err:expr) => {
        match $err {
            hal::i2c::Error::NotAcknowledge => {defmt::error!("I2C: No Acknowledge")},
            hal::i2c::Error::Bus => {defmt::error!("I2C: Bus")},
            hal::i2c::Error::Arbitration => {defmt::error!("I2C: Arbitration")}
            _ => todo!()
        }
    };
}

#[allow(dead_code)]
fn codec_setup(i2c: &mut I2c<hal::pac::I2C1>, mut reset: hal::gpio::Pin<'B', 11, Output>) {
    
    // Reset the codec chip
    // Hold it low for ~1ms
    reset.set_low();
    
    // Delay 
    // TODO: Calculate time
    for _ in 0..100000 {
        asm::nop();
    }

    reset.set_high();
    for _ in 0..100000 {
        asm::nop();
    }

    // Write a bunch of commands

    const CODEC_ADDR: u8 = 0b001000_1; // on wire 001000_[ADDR=1]

    let control_1_reg = &[
        0x01u8, // Control Mode 
        // 01 Mode Control 1
        // M1           0   Single-Speed Mode: 4 to 50 kHz sample rates (default)    
        // M0           0
        // Ratio1       0   Single Speed Mode
        // Ratio0       0
        // M/S          0   Slave mode
        // DAC_DIF2     0   Right Justified, 16-bit Data DAC input
        // DAC_DIF1     1
        // DAC_DIF0     0
        0b00_00_0_010,
    ];

    let mut buf = *control_1_reg;
    defmt::debug!("control_1_reg {:?}", buf);
    match i2c.write(CODEC_ADDR, &buf) {
        Ok(_) => {
            defmt::debug!("Ok");
        }
        Err(err) => {
            i2c_error_to_string!(err);
            panic!("error {:?}", err);
        }
    }

    let control_2_reg = &[
        0x07u8, // Control Mode 2
        // 07 Mode Control 2
        // Reserved       0
        // Reserved       0
        // Reserved       0
        // LOOP           0    No Loopback
        // MUTE A AND B   0    Individual Mute Signals
        // FREEZE         0    Immediate updates
        // CPEN           1    Enable port mode
        // PDEN           1    Hold in power down while setting up
        0b000_0_0_0_1_1,
    ];

    buf = *control_2_reg;
    defmt::debug!("control_2_reg {:?}", buf);
    match i2c.write(CODEC_ADDR, &buf) {
        Ok(_) => {
            defmt::debug!("Ok");
        }
        Err(err) => {
            i2c_error_to_string!(err);
            panic!("error {:?}", err);
        }
    }

    let init = &[
        0x01u8 | 1 << 7, //   incremental mmap from address 1

        // 01, Mode Control 1
        // M              00   Single Speed
        // R              00   MCLK/LRCK (FS) = 256 -> 4MHz/256 = 15.6kHz
        // M/S            0    Slave Mode
        // DAC_DIF        001  I2S, up to 24 bit
        0b00_00_0_001,
        
        // 02, DAC Control
        // AMUTE          1    Auto Mute Enable (mute on consecutive 0 or-1)
        // FILT_SEL       0    Fast Roll off
        // DEM            00   Deemphasis disabled
        // RMP_UP         0    Immediate
        // RMP_Down       0    Immediate
        // INV            00   0 phase
        0b1_0_00_0_0_00,
        
        // 03, DAC Volume & Mixing Control
        // Reserved       0
        // B=0            1    Linked
        // Soft           1    Soft ramping of volume
        // ZeroCross      1    Minimize artifacts on volume change
        // ATAPI          1001 L/R
        0b0_1_1_1_1001,

        // 04, DAC A Volume Control
        // MUTE           0    Un-muted
        // VOL            0    Full Volume (no attenuation)
        0b0_0000000,

        // 05, DAC B Volume Control
        // MUTE           0    Un-muted
        // VOL            0    Full Volume (no attenuation)
        0b0_0000000,

        // 06 ADC Control
        // Reserved       0
        // Reserved       0
        // Dither16       0    No dither for 16 bit
        // ADC_DIF        1    I2S
        // MUTE_A         0    Un-Muted
        // MUTE_B         0    Un-Muted
        // HPF A Disable  1    DC Mode
        // HPF B Disable  1    DC Mode
        0b00_0_1_00_11,

        // 07 Mode Control 2
        // Reserved       0
        // Reserved       0
        // Reserved       0
        // LOOP           0    No Loopback
        // MUTE A AND B   0    Individual Mute Signals
        // FREEZE         0    Immediate updates
        // CPEN           1    Enable port mode
        // PDEN           1    Still hold in power down
        0b000_0_0_0_1_1,
    ];

    let buf = *init;
    defmt::debug!("init {:?}", buf);
    match i2c.write(CODEC_ADDR, &buf) {
        Ok(_) => {
            defmt::debug!("Ok");
        }
        Err(err) => {
            i2c_error_to_string!(err);
            panic!("error {:?}", err);
        }
    }

    let start = &[
        0x07u8, // 07 Mode Control 2
        // 07 Mode Control 2
        // Reserved       0
        // Reserved       0
        // Reserved       0
        // LOOP           0    No Loopback
        // MUTE A AND B   0    Individual Mute Signals
        // FREEZE         0    Immediate updates
        // CPEN           1    Enable port mode
        // PDEN           0    Power up
        0b000_0_0_0_1_0,
    ];

    let buf = *start;
        defmt::debug!("boot {:?}", buf);
        match i2c.write(CODEC_ADDR, &buf) {
            Ok(_) => {
                defmt::debug!("Ok");
            }
            Err(err) => {
                i2c_error_to_string!(err);
                panic!("error {:?}", err);
            }
        }
}

pub fn init(mut codec_handler: Codec) -> SaiContainer {

    #[allow(unused)]
    let mut i2c1 = codec_handler.i2c1.i2c(
        (
            codec_handler.i2c1_scl.into_alternate().set_open_drain(), 
            codec_handler.i2c1_sda.into_alternate().set_open_drain()), 
        Hertz::from_raw(4_000), codec_handler.i2c1_peripheral, 
        &codec_handler.clocks
    );

    // codec_setup(&mut i2c1, codec_handler.reset);

    // Use PLL3_P for the SAI1 clock
    let sai1_rec = codec_handler.sai1_rec.kernel_clk_mux(hal::rcc::rec::Sai1ClkSel::Pll3P);

    let sai1_tx_config =
        hal::sai::I2SChanConfig::new(hal::sai::I2SDir::Tx)
        .set_sync_type(hal::sai::I2SSync::Master)
        .set_frame_sync_active_high(true);

    let sai1_rx_config = 
        hal::sai::I2SChanConfig::new(hal::sai::I2SDir::Rx)
        .set_sync_type(hal::sai::I2SSync::Internal)
        .set_frame_sync_active_high(true);

    let sai2_tx_config = 
        hal::sai::I2SChanConfig::new(hal::sai::I2SDir::Tx)
        .set_sync_type(hal::sai::I2SSync::Internal)
        .set_frame_sync_active_high(true);

    let sai2_rx_config = 
        hal::sai::I2SChanConfig::new(hal::sai::I2SDir::Rx)
        .set_sync_type(hal::sai::I2SSync::Internal)
        .set_frame_sync_active_high(true);

    let sai1_pins = (
        codec_handler.sai1_mclk.into_alternate(),
        codec_handler.sai1_sck.into_alternate(),
        codec_handler.sai1_lrclk.into_alternate(),
        codec_handler.sai1_dout.into_alternate(),
        Some(codec_handler.sai1_din.into_alternate())
    );

    let sai2_pins = (
        codec_handler.sai2_mclk.into_alternate(),
        codec_handler.sai2_sck.into_alternate(),
        codec_handler.sai2_lrckl.into_alternate(),
        codec_handler.sai2_dout.into_alternate(),
        Some(codec_handler.sai2_din.into_alternate())
    );

    let mut audio1 = codec_handler.sai1_dev.i2s_ch_a(
        sai1_pins,
        AUDIO_SAMPLE_HZ,
        hal::sai::I2SDataSize::BITS_24,
        sai1_rec,
        &codec_handler.clocks,
        hal::sai::I2sUsers::new(sai1_tx_config).add_slave(sai1_rx_config),
    );

    let mut audio2 = codec_handler.sai2_dev.i2s_ch_a(
        sai2_pins,
        AUDIO_SAMPLE_HZ,
        hal::sai::I2SDataSize::BITS_24,
        codec_handler.sai2_rec,
        &codec_handler.clocks,
        hal::sai::I2sUsers::new(sai2_tx_config).add_slave(sai2_rx_config),
    );

    // Setup cache
    // Sound breaks up without this enabled
    codec_handler.scb.enable_icache();

    // Try to synchronize the second SAI instance with the first
    audio2.set_sync_input(0);

    // audio1.listen(hal::sai::SaiChannel::ChannelB, hal::sai::Event::Data);
    // audio2.listen(hal::sai::SaiChannel::ChannelB, hal::sai::Event::Data);
    // audio1.enable();
    // audio2.enable();

    // Jump start audio
    // Each of the audio blocks in the SAI are enabled by SAIEN bit in the SAI_xCR1 register.
    // As soon as this bit is active, the transmitter or the receiver is sensitive
    // to the activity on the clock line, data line and synchronization line in slave mode.
    // In master TX mode, enabling the audio block immediately generates the bit clock for the
    // external slaves even if there is no data in the FIFO, However FS signal generation
    // is conditioned by the presence of data in the FIFO.
    // After the FIFO receives the first data to transmit, this data is output to external slaves.
    // If there is no data to transmit in the FIFO, 0 values are then sent in the audio frame
    // with an underrun flag generation.
    // From the reference manual (rev7 page 2259)

    // A "word" is a u32
    audio1.try_send(0, 0).unwrap();
    audio2.try_send(0, 0).unwrap();


    (audio1, audio2)
}
