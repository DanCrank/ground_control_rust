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
* TODO: find the read timeout, and any other registers we need
*   to set
*
* TODO: improve error handling with either error-chain or failure
*     https://github.com/rust-lang-nursery/error-chain
*     https://docs.rs/failure/0.1.8/failure/
**************************************************************/

use display_interface::DisplayError;
use rfm69:: {
    Rfm69,
    registers:: { DataMode, InterPacketRxDelay, Modulation, ModulationShaping, ModulationType,
                  PacketConfig, PacketDc, PacketFiltering, PacketFormat, Registers }
};
use rppal:: {
    gpio::{Gpio, OutputPin},
    i2c::I2c,
    spi::{Bus, SlaveSelect, Spi}
};
use ssd1306:: {
    mode::TerminalMode,
    mode::terminal::TerminalModeError,
    prelude::*,
    Builder,
    I2CDIBuilder
};
use std:: {
    fmt::Write,
    io:: { Error, ErrorKind },
    thread,
    time
};

// this macro consumes a valueless Result that may contain an Error
// we want to just print and ignore
// TODO this is likely not the right way to do this
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
    // errors separately in this abomination
    // TODO this cannot possibly be the right way to do this - look at
    //     https://doc.rust-lang.org/core/result/enum.Result.html#method.err
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
    // see https://github.com/almusil/rfm69/blob/master/examples/receive.rs
    let gpio = Gpio::new()?;
    // configure CS pin
    let mut cs = gpio.get(7)?.into_output();
    cs.set_high();
    // configure reset pin
    let mut reset = gpio.get(25)?.into_output();
    reset.set_low();
    // just guessing here, but wait a sec, then reset the RFM69 the same way the CircuitPython code does
    thread::sleep(time::Duration::from_secs(1));
    reset.set_high();
    thread::sleep(time::Duration::from_millis(10));
    reset.set_low();
    thread::sleep(time::Duration::from_secs(1));
    // Configure SPI 8 bits, Mode 0
    let spi = Spi::new(Bus::Spi0, SlaveSelect::Ss0, 2_000_000, rppal::spi::Mode::Mode0)?;
    // Create rfm struct with defaults that are set after reset
    let mut rfm = Rfm69::new(spi, cs, linux_embedded_hal::Delay);
    dont_panic!(rfm.modulation(Modulation { data_mode: DataMode::Packet,
                                              modulation_type: ModulationType::Fsk,
                                              shaping: ModulationShaping::Shaping00 }));
    // TODO preamble
    // TODO sync
    dont_panic!(rfm.packet(PacketConfig { format: PacketFormat::Variable(0),
                                            dc: PacketDc::None,
                                            crc: true,
                                            filtering: PacketFiltering::None, // TODO use Address
                                            interpacket_rx_delay: InterPacketRxDelay::Delay1Bit, // ???
                                            auto_rx_restart: true }));
    // TODO node_address
    // TODO aes
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

    // debug
    // print content of all RFM registers
    // match rfm.read_all_regs() {
    //     Ok(regs) => {
    //         for (index, val) in regs.iter().enumerate() {
    //             println!("Register 0x{:02x} = 0x{:02x}", index + 1, val);
    //         }
    //     },
    //     Err(e) => println!("Could not read RFM69 registers! {:#?}", e),
    // }

    Result::Ok(rfm)
}

fn main() {
    let mut disp = setup_display().unwrap();
    dont_panic!(disp.write_str("Rover Ground\nControl v0.0"));

    let mut rfm = setup_radio().unwrap();

    // print content of all RFM registers
    // match rfm.read_all_regs() {
    //     Ok(regs) => {
    //         for (index, val) in regs.iter().enumerate() {
    //             println!("Register 0x{:02x} = 0x{:02x}", index + 1, val);
    //         }
    //     },
    //     Err(e) => println!("Could not read RFM69 registers! {:#?}", e),
    // }

    // prepare buffer to store the received data
    let mut buffer = [0; 64];
    // loop and receive packets
    loop {
        // recv claims it "blocks until there are any bytes available"
        // but this is a lie; it actually has a hardcoded timeout of 100ms
        // and returns a timeout error if there are no packets in that time.
        match rfm.recv(&mut buffer) {
            Ok(_) => {
                // print received data
                for (index, val) in buffer.iter().enumerate() {
                    println!("Value at {} = {}", index, val)
                }
            },
            Err(e) => {
                match e {
                    rfm69::Error::Timeout => (),
                    _ => println!("Ow! {:#?}", e)
                }
                // thread::sleep(time::Duration::from_secs(1));
            },
        }
    }
}