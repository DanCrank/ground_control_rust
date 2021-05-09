/**************************************************************
 * Prototype rover ground station code
 * This code targets the following hardware...
 *
 * Raspberry Pi (any)
 *
 * Adafruit RFM69HCW Transceiver Radio Bonnet - 868 / 915 MHz
 * https://learn.adafruit.com/adafruit-radio-bonnets
 * https://cdn-shop.adafruit.com/product-files/3076/RFM69HCW-V1.1.pdf
 *
 * TODO: create a new error type to improve error handling
 *     since neither rfm69 nor ssd1306 impl Error for their
 *     error types. See:
 *     https://doc.rust-lang.org/stable/rust-by-example/error/multiple_error_types/wrap_error.html
 **************************************************************/

// `error_chain!` can recurse deeply
#![recursion_limit = "1024"]

#[macro_use]
extern crate error_chain;
extern crate serde;
#[macro_use]
extern crate serde_derive;
extern crate rmp_serde as rmps;

use errors::*;
use rfm69:: {
    Rfm69,
    registers:: { DataMode, DccCutoff, InterPacketRxDelay, Modulation, ModulationShaping, ModulationType,
                  PacketConfig, PacketDc, PacketFiltering, PacketFormat, Registers, RxBw, RxBwFsk }
};
use rppal:: {
    gpio::{Gpio, OutputPin},
    i2c::I2c,
    spi::{Bus, SlaveSelect, Spi}
};
use ssd1306:: {
    mode::TerminalMode,
    prelude::*,
    Builder,
    I2CDIBuilder
};
use std:: {
    fmt::Write,
    thread,
    time
};
use crate::messages::*;

mod errors;
mod messages;


// this macro consumes a valueless Result that may contain an Error
// we want to just print and ignore
macro_rules! dont_panic {
    ($a:expr) => {
        {
            match $a {
                Ok(_) => (),
                Err(e) => println!("{:#?}", e),
            }
        }
    }
}

// set up the OLED display on the RFM69 bonnet
fn setup_display() -> Result<TerminalMode<I2CInterface<I2c>, DisplaySize128x32>> {
    // initialize the display on the RFM69 bonnet
    let i2c = I2c::new()?;
    let interface = I2CDIBuilder::new().init(i2c);
    let mut disp: TerminalMode<_, _> = Builder::new()
        .size(DisplaySize128x32)
        .connect(interface)
        .into();
    match disp.init() {
        Err(e) => return Err(format!("Error while initializing display: {:?}", e).into()),
        _ => {}
    }
    match disp.clear() {
        Err(e) => return Err(format!("Error while clearing display: {:?}", e).into()),
        _ => {}
    }
    Ok(disp)
}

// set up the RFM69
fn setup_radio() -> Result<Rfm69<OutputPin, Spi, linux_embedded_hal::Delay>> {
    // initialize the RFM69 radio
    // see https://github.com/almusil/rfm69/blob/master/examples/receive.rs
    let gpio = Gpio::new()?;
    // configure CS pin
    let mut cs = gpio.get(7)?.into_output();
    cs.set_high();
    cs.set_reset_on_drop(false);
    // configure reset pin
    let mut reset = gpio.get(25)?.into_output();
    reset.set_low();
    reset.set_reset_on_drop(false);
    // reset the RFM69 the same way the CircuitPython code does
    reset.set_high();
    thread::sleep(time::Duration::from_millis(100));
    reset.set_low();
    // configure SPI 8 bits, Mode 0
    let spi = Spi::new(Bus::Spi0, SlaveSelect::Ss0, 2_000_000, rppal::spi::Mode::Mode0)?;
    let mut rfm = Rfm69::new(spi, cs, linux_embedded_hal::Delay);
    dont_panic!(rfm.modulation(Modulation { data_mode: DataMode::Packet,
                                              modulation_type: ModulationType::Fsk,
                                              shaping: ModulationShaping::Shaping00 })); //no shaping
    dont_panic!(rfm.bit_rate(9600.0));
    dont_panic!(rfm.frequency(868_000_000.0));
    // don't know if it matters, but the value computed by fdev() is off by 1 from what the sender has.
    // therefore, set the exact value.
    // instead of: dont_panic!(rfm.fdev(19200.0));
    dont_panic!(rfm.write(Registers::FdevMsb, 0x01));
    dont_panic!(rfm.write(Registers::FdevLsb, 0x3b));
    // preamble - default 4 octets per RadioHead
    dont_panic!(rfm.preamble(4));
    // sync - default 2 octets (0x2d, 0xd4) per RadioHead
    dont_panic!(rfm.sync(&[0x2d, 0xd4]));
    dont_panic!(rfm.packet(PacketConfig { format: PacketFormat::Variable(64),
                                            dc: PacketDc::Whitening,
                                            crc: true,
                                            filtering: PacketFiltering::None, // TODO use Address
                                            interpacket_rx_delay: InterPacketRxDelay::Delay1Bit, // ???
                                            auto_rx_restart: true }));
    dont_panic!(rfm.rx_bw(RxBw { dcc_cutoff: DccCutoff::Percent0dot125, rx_bw: RxBwFsk::Khz25dot0 }));
    dont_panic!(rfm.rx_afc_bw(RxBw { dcc_cutoff: DccCutoff::Percent0dot125, rx_bw: RxBwFsk::Khz25dot0 }));
    // rfm69 library never appears to set power level
    dont_panic!(rfm.write(Registers::PaLevel, 0x7c));
    // TODO aes encryption
    // the rfm69 lib defines Mode::Receiver as 0x10, which appears to be incorrect.
    // therefore, directly set the correct receiver mode (0x08).
    // instead of: dont_panic!(rfm.mode(Mode::Receiver));
    dont_panic!(rfm.write(Registers::OpMode, 0x08));
    // check for good connection by reading back version register
    // see https://github.com/adafruit/Adafruit_CircuitPython_RFM69/blob/ad33b2948a13df1c0e036605ef1fb5e6484ea97e/adafruit_rfm69.py#L263
    match rfm.read(Registers::Version) {
        Ok(i) => {
            println!("RFM69 version: 0x{:02x}", i);
            if i != 0x24 {
                panic!("Expected version 0x24, exiting.");
            }
        },
        Err(e) => panic!("Error connecting to RFM69: {:#?}", e)
    }
    println!("Carrier frequency: {} MHz", get_frequency(&mut rfm));
    Ok(rfm)
}

// get the carrier frequency currently set in the RFM69
fn get_frequency(rfm: &mut Rfm69<OutputPin, Spi, linux_embedded_hal::Delay>) -> u32 {
    let mut freq: u32 = 0;
    for reg in &[Registers::FrfMsb, Registers::FrfMid, Registers::FrfLsb] {
        freq = freq << 8 + match rfm.read(*reg) {
            Ok(b) => b,
            Err(e) => {
                println!("Error reading frequency register: {:#?}", e);
                0
            }
        };
    }
    freq
}

fn process_telemetry(telemetry: &RoverMessage) {
    match telemetry {
        RoverMessage::TelemetryMessage { .. }
            => println!("Telemetry packet received:\n{:#?}", telemetry),
        _ => println!("Wrong message type received in process_telemetry:\n{:#?}", telemetry)
    }
}

fn run() -> Result<()> {
    let mut disp = setup_display().unwrap();
    dont_panic!(disp.write_str("Rover Ground\nControl v0.0"));
    let mut rfm = setup_radio().unwrap();
    // loop and receive telemetry packets
    loop {
        let telemetry: RoverMessage = RoverMessage::TelemetryMessage { timestamp: Default::default(),
                                                                       location: Default::default(),
                                                                       status: "".to_string()
        };
        match telemetry.receive(&mut rfm, 10000) {
            Ok(()) => process_telemetry(&telemetry),
            Err(e) => println!("{:#?}", e)
        }
    }
}

fn main() {
    if let Err(ref e) = run() {
        println!("error: {}", e);
        for e in e.iter().skip(1) {
            println!("caused by: {}", e);
        }
        if let Some(backtrace) = e.backtrace() {
            println!("backtrace: {:?}", backtrace);
        }
        ::std::process::exit(1);
    }
}