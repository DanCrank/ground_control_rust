// message structures used by ground_control

use crate::errors::*;
use rfm69::Rfm69;
use rmps::{ Deserializer, Serializer };
use rppal::{ gpio::OutputPin, spi::Spi };
use serde::{ Deserialize, Serialize };
use std::{ thread };
use std::time::{ Duration, Instant };

const ACK_TIMEOUT: u64 = 2000;   // millis to wait for an ack msg
const MSG_DELAY: u64 = 250;      // millis to wait between Rx and Tx, to give the other side time to switch from Tx to Rx
const LISTEN_DELAY: u64 = 100;   // millis to wait between checks of the receive buffer when receiving
const USE_ENCRYPTION: bool = true;

#[derive(Debug, PartialEq, Serialize, Deserialize)]
pub enum RoverMessage {
    TelemetryMessage { timestamp: RoverTimestamp,  // sent by the rover to communicate location and status.
                       location: RoverLocData,     // max status length should be 35 ASCII chars with encryption
                       status: String },           // turned on, 226 chars with it turned off

    TelemetryAck { timestamp: RoverTimestamp,      // sent by the station to acknowledge a TelemetryMessage
                   command_waiting: bool,          // and possibly tell the rover to switch to command mode
                   msg: String },                  // (if commandWaiting = true). max msg length = 58/249

    CommandReady { timestamp: RoverTimestamp,      // sent by the rover to indicate it is ready to receive commands.
                   msg: String },                  // max msg length = 59/250

    CommandMessage { timestamp: RoverTimestamp,    // sent by the station to communicate part of a command
                     command: String,              // sequence and possibly tell the rover that the sequence
                     sequence_complete: bool },    // is complete (if sequenceComplete = true). max command length = 58/249

    CommandAck { timestamp: RoverTimestamp,        // sent by the rover to acknowledge a CommandMessage. max msg
                 msg: String },                    // length = 59/250
}

#[derive(Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct RoverTimestamp {  // should serialize to 3 bytes (3x +fixint@1)
pub hour: u8,
    pub minute: u8,
    pub second: u8,
}

#[derive(Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct RoverLocData { // should serialize to 24 bytes (4x float-32@5, +fixint@1, int-16@3)
pub gps_lat: f32,
    pub gps_long: f32,
    pub gps_alt: f32,
    pub gps_speed: f32,
    pub gps_sats: u8,
    pub mag_hdg: u16,
}

impl RoverMessage {
    // send msg via radio rfm; wait up to ack_timeout milliseconds for an
    // acknowledgement if needed. ACK logic is encapsulated here - e.g.,
    // a CommandMessage expects an ACK, but a TelemetryAck does not.
    pub fn send(&self,
            rfm: &mut Rfm69<OutputPin, Spi, linux_embedded_hal::Delay>) -> Result<()> {
        let mut max_message_length = 251;
        if USE_ENCRYPTION { max_message_length = 60; }
        // serialize the message
        // first four bytes are used by RadioHead as TO, FROM, ID, FLAGS
        // so push those onto the Vec before serializing the rest of the payload
        // TODO: if we ever need to put real values for these, do it here
        let mut buf = vec![0xff, 0xff, 0x00, 0x00];
        RoverMessage::serialize(&self, &mut Serializer::new(&mut buf)).unwrap();
        // check message length
        if buf.len() > max_message_length {
            return Err(format!("Cannot send: message too long! {:?}", self).into())
        }
        // send it
        // DEBUG
        println!("DEBUG: sending this message:\n{:?}", buf);
        match rfm.send(buf.as_slice()) {
            Err(e) => return Err(format!("Error while sending message: {:?}", e).into()),
            _ => {}
        }
        // receive ack if appropriate
        match self {
            RoverMessage::CommandMessage { .. } => {
                let ack: RoverMessage = RoverMessage::CommandAck { timestamp: Default::default(),
                                                                   msg: "".to_string() };
                ack.receive(rfm, ACK_TIMEOUT)?
            },
            _ => (), // no ack needed
        }
        Ok(())
    }

    // receive the next message via radio rfm, ack if necessary, and return
    // the received message. ACK logic is encapsulated here - e.g., a
    // TelemetryMessage should be ACKed but a CommandAck message should not.
    // this gets slightly awkward if the rover responds with an inappropriate
    // message (e.g., station sends CommandMessage, then rover sends TelemetryMessage
    // instead of CommandAck - station will still ACK the TelemetryMessage before
    // bubbling back and reporting the error).
    pub fn receive(&self,
               rfm: &mut Rfm69<OutputPin, Spi, linux_embedded_hal::Delay>,
               timeout: u64) -> Result<()> {
        let mut buf = [0 as u8; 64];
        // recv claims it "blocks until there are any bytes available"
        // but this is a lie; it actually has a hardcoded timeout of 100ms
        // and returns a timeout error if there are no packets in that time.
        let start = Instant::now();
        let mut complete = false;
        while !complete {
            match rfm.recv(&mut buf) {
                Ok(_) => { complete = true; },
                Err(e) => {
                    match e {
                        rfm69::Error::Timeout => {
                            thread::sleep(Duration::from_millis(LISTEN_DELAY));
                            // eat timeouts but cough up anything else
                        },
                        _ => {
                            return Err(format!("Error while waiting for RoverMessage: {:?}", e).into())
                        }
                    }
                }
            }
            if Instant::now().duration_since(start) > Duration::from_millis(timeout) { break };
            thread::sleep(Duration::from_millis(LISTEN_DELAY));
        }
        if !complete { return Err("Timed out while waiting for RoverMessage.".into()) }
        // DEBUG
        println!("DEBUG: received this message:\n{:?}", buf);
        // deserialize the message
        // first four bytes are used by RadioHead as TO, FROM, ID, FLAGS
        // so strip those off before deserializing the rest of the payload
        // TODO: if those values are ever needed, grab them here
        match RoverMessage::deserialize(&mut Deserializer::new(&mut buf[4..].as_ref())) {
            Err(e) => return Err(format!("Error while deserializing response: {:?}", e).into()),
            _ => {}
        }
        // ACK if necessary
        match self {
            RoverMessage::TelemetryMessage{..} => {
                let ack: RoverMessage = RoverMessage::TelemetryAck { timestamp: Default::default(),
                                                                     command_waiting: false,
                                                                     msg: "".to_string()
                };
                thread::sleep(Duration::from_millis(MSG_DELAY));
                ack.send(rfm)?
            },
            _ => (), // no ack needed
        }
        Ok(())
    }
}
