/**************************************************************
* Prototype rover ground station code
* This code targets the following hardware...
*
* Raspberry Pi (any)
*
* Adafruit RFM69HCW Transceiver Radio Bonnet - 868 / 915 MHz
* https://learn.adafruit.com/adafruit-radio-bonnets
**************************************************************/

use display_interface::DisplayError;
//use linux_embedded_hal::spidev::{SpiModeFlags, SpidevOptions};
//use linux_embedded_hal::sysfs_gpio::Direction;
//use linux_embedded_hal::{Delay, Pin, Spidev};
use rfm69::Rfm69;
use rppal::gpio::{Gpio, OutputPin};
use rppal::i2c::I2c;
use rppal::spi::{Bus, Mode, /* Segment, */ SlaveSelect, Spi};
use ssd1306::{
    mode::TerminalMode,
    mode::terminal::TerminalModeError,
    prelude::*,
    Builder,
    I2CDIBuilder
};
use std::fmt::Write;
use std::io::Error;
use std::io::ErrorKind;

// this macro consumes a valueless Result that may contain an Error
// we want to just print and ignore
macro_rules! ignore_error {
    ($a:expr) => {
        {
            match $a {
                Ok(_) => (),
                Err(e) => println!("{}", e),
            }
        }
    }
}

// init() and clear() have to be in their own function because they use an incompatible error type
// (until I figure out the right way to do this)
fn init_display(disp: &mut TerminalMode<I2CInterface<I2c>, DisplaySize128x32>) -> Result<(), TerminalModeError> {
    disp.init()?;
    disp.clear()?;
    Ok(())
}

fn setup_display() -> Result<TerminalMode<I2CInterface<I2c>, DisplaySize128x32>, Box<dyn std::error::Error>> {
    // initialize the display on the RFM69 bonnet
    let i2c = I2c::new()?;
    let interface = I2CDIBuilder::new().init(i2c);
    let mut disp: TerminalMode<_, _> = Builder::new()
        .size(DisplaySize128x32)
        .connect(interface)
        .into();
    // TerminalModeError does not implement std::error::Error, so we have to handle its
    // errors separately here
    match init_display(&mut disp) {
        Ok(_)=> Ok(disp),
        Err(e) =>
            match e {
                TerminalModeError::InterfaceError(d) => match d {
                    DisplayError::InvalidFormatError => Err(Box::new(Error::new(ErrorKind::Other, "InterfaceError (InvalidFormat) in init_display()"))),
                    DisplayError::BusWriteError => Err(Box::new(Error::new(ErrorKind::Other, "InterfaceError (BusWrite) in init_display()"))),
                    DisplayError::DCError => Err(Box::new(Error::new(ErrorKind::Other, "InterfaceError (DC) in init_display()"))),
                    DisplayError::CSError => Err(Box::new(Error::new(ErrorKind::Other, "InterfaceError (CS) in init_display()"))),
                    DisplayError::DataFormatNotImplemented => Err(Box::new(Error::new(ErrorKind::Other, "InterfaceError (DataFormat) in init_display()"))),
                    _ => Err(Box::new(Error::new(ErrorKind::Other, "InterfaceError (undefined) in init_display()"))),
                }
                TerminalModeError::OutOfBounds => Err(Box::new(Error::new(ErrorKind::Other, "OutOfBounds in init_display()"))),
                TerminalModeError::Uninitialized => Err(Box::new(Error::new(ErrorKind::Other, "Uninitialized in init_display()")))
            },
    }
}

fn setup_radio() -> Result<Rfm69<OutputPin, Spi, linux_embedded_hal::Delay>, Box<dyn std::error::Error>> {
    // initialize the RFM69 radio
    // see https://github.com/almusil/rfm69/blob/master/examples/send.rs
    let gpio = Gpio::new()?;
    // Configure CS pin
    let mut cs = gpio.get(26)?.into_output();
    cs.set_high();
    // Configure SPI 8 bits, Mode 0
    let spi = Spi::new(Bus::Spi0, SlaveSelect::Ss0, 1_000_000, Mode::Mode0)?;
    // Create rfm struct with defaults that are set after reset
    Ok(Rfm69::new(spi, cs, linux_embedded_hal::Delay))
}

fn main() {
    let mut disp = setup_display().unwrap();
    ignore_error!(disp.write_str("Rover Ground\nControl v0.0"));

    let mut rfm = setup_radio().unwrap();

    // print content of all RFM registers
    for (index, val) in rfm.read_all_regs().ok().unwrap().iter().enumerate() {
        println!("Register 0x{:02x} = 0x{:02x}", index + 1, val);
    }

    // prepare buffer to store the received data
    let mut buffer = [0; 64];
    // loop and receive packets
    loop {
        // recv "blocks until there are any bytes available"
        rfm.recv(&mut buffer).ok().unwrap();
        // print received data
        for (index, val) in buffer.iter().enumerate() {
            println!("Value at {} = {}", index, val)
        }
    }
}