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
 * TODO: improve error handling with either error-chain or failure
 *     https://github.com/rust-lang-nursery/error-chain
 *     https://docs.rs/failure/0.1.8/failure/
 **************************************************************/

use display_interface::DisplayError;
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

fn print_frequency(rfm: &mut Rfm69<OutputPin, Spi, linux_embedded_hal::Delay>)
{
    let freq_msb = match rfm.read(Registers::FrfMsb) {
        Ok(b) => b,
        Err(e) => {
            println!("Error reading freq_msb: {:#?}", e);
            0
        }
    };
    let freq_mid = match rfm.read(Registers::FrfMid) {
        Ok(b) => b,
        Err(e) => {
            println!("Error reading freq_mid: {:#?}", e);
            0
        }
    };
    let freq_lsb = match rfm.read(Registers::FrfLsb) {
        Ok(b) => b,
        Err(e) => {
            println!("Error reading freq_lsb: {:#?}", e);
            0
        }
    };
    // println!("Raw frequency value: 0x{:02x}{:02x}{:02x}", freq_msb, freq_mid, freq_lsb);
    println!("Carrier frequency: {} MHz", ((((freq_msb as u32) << 16) +
        ((freq_mid as u32) << 8) +
        (freq_lsb as u32)) as f32) * 61.035);
}

fn print_rfm69_register(rfm: &mut Rfm69<OutputPin, Spi, linux_embedded_hal::Delay>, reg: Registers) -> u8
{
    let val = match rfm.read(reg) {
        Ok(b) => b,
        Err(e) => {
            println!("Error reading RFM69 register: {:#?}", e);
            0
        }
    };
    val
}

fn setup_radio() -> Result<Rfm69<OutputPin, Spi, linux_embedded_hal::Delay>, Box<dyn std::error::Error>> {
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
    // just guessing here, but wait a sec, then reset the RFM69 the same way the CircuitPython code does
    thread::sleep(time::Duration::from_secs(1));
    reset.set_high();
    thread::sleep(time::Duration::from_millis(100));
    reset.set_low();
    thread::sleep(time::Duration::from_secs(1));
    // Configure SPI 8 bits, Mode 0
    let spi = Spi::new(Bus::Spi0, SlaveSelect::Ss0, 2_000_000, rppal::spi::Mode::Mode0)?;
    // Create rfm struct with defaults that are set after reset
    let mut rfm = Rfm69::new(spi, cs, linux_embedded_hal::Delay);
    // sender settings: FSK, Whitening, Rb = 9.6kbs, Fd = 19.2kHz, carrier freq 868.0
    dont_panic!(rfm.modulation(Modulation { data_mode: DataMode::Packet,
                                              modulation_type: ModulationType::Fsk,
                                              shaping: ModulationShaping::Shaping00 })); //no shaping
    dont_panic!(rfm.bit_rate(9600.0));
    dont_panic!(rfm.frequency(868_000_000.0));
    // don't know if it matters, but the value computed by fdev() is off by 1 from what the sender has.
    // therefore, set the exact value.
    //dont_panic!(rfm.fdev(19200.0));
    dont_panic!(rfm.write(Registers::FdevMsb, 0x01));
    dont_panic!(rfm.write(Registers::FdevLsb, 0x3b));
    // preamble - default 4 octets per RadioHead
    dont_panic!(rfm.preamble(4)); // assuming it's set in bytes here, not seeing anything that says it wants octets
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
    // rfm69 library never appears to set power level - this just matches what I see on the sender
    dont_panic!(rfm.write(Registers::PaLevel, 0x7c));
    // TODO node_address
    // TODO aes
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

    Ok(rfm)
}

fn main() {
    let mut disp = setup_display().unwrap();
    dont_panic!(disp.write_str("Rover Ground\nControl v0.0"));

    let mut rfm = setup_radio().unwrap();

    print_frequency(&mut rfm);

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
                // buffer[0] is a length byte
                // buffer[1] through buffer[4] appears to be a preamble? these bytes are counted in length
                // actual payload starts at buffer[5] and ends at buffer[payload_length]
                let payload_length = usize::from(buffer[0]);
                match std::str::from_utf8(&buffer[5..(payload_length + 1)]) {
                    Ok(s) => println!("{}", s),
                    Err(e) => {
                        println!("Cannot convert payload ({:#?})", e);
                        for b in buffer.iter() {
                            print!("{:02x} ", b);
                        }
                        print!("\n");
                    }
                }
            },
            Err(e) => {
                match e {
                    rfm69::Error::Timeout => (),
                    _ => println!("Receive error {:#?}", e)
                }
                thread::sleep(time::Duration::from_secs(1));
            },
        }
    }
}