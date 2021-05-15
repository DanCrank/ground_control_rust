// message structures used by ground_control

use chrono::prelude::*;
use crate::errors::*;
use rfm69::Rfm69;
use rppal::{ gpio::OutputPin, spi::Spi };
use std::{ thread };
use std::time::{ Duration, Instant };
use rfm69::registers::Registers;

const ACK_TIMEOUT: u64 = 30000;   // millis to wait for an ack msg
const MSG_DELAY: u64 = 250;      // millis to wait between Rx and Tx, to give the other side time to switch from Tx to Rx
const LISTEN_DELAY: u64 = 100;   // millis to wait between checks of the receive buffer when receiving
const USE_ENCRYPTION: bool = true;

// message IDs for serialization
const MESSAGE_TELEMETRY: u8 = 0;
const MESSAGE_TELEMETRY_ACK: u8 = 1;
const MESSAGE_COMMAND_READY: u8 = 2;
const MESSAGE_COMMAND: u8 = 3;
const MESSAGE_COMMAND_ACK: u8 = 4;

// serialization / deserialization code on this end currently assumes that we will have
// five extra header bytes on the head of the payload that we need to (for the moment)
// ignore. RadioHead invisibly deals with these on the rover end but the rfm69 library
// we use on this end does not take them back off. the first byte is the total payload
// length (including the five header bytes) and the next four are TO, FROM, ID, FLAGS
// currently hardcoded to vec![0xff, 0xff, 0x00, 0x00]
#[derive(Debug)]
pub enum RoverMessage {
    TelemetryMessage { timestamp: RoverTimestamp,  // sent by the rover to communicate location and status.
                       location: RoverLocData,     // max status length should be 31 ASCII chars with encryption
                       signal_strength: i16,       // turned on, 222 chars with it turned off
                       free_memory: u16,
                       status: String },

    TelemetryAck { timestamp: RoverTimestamp,      // sent by the station to acknowledge a TelemetryMessage
                   ack: bool,                      // and possibly tell the rover to switch to command mode
                   command_waiting: bool },        // (if commandWaiting = true). max msg length = 58/249

    CommandReady { timestamp: RoverTimestamp,      // sent by the rover to indicate it is ready to receive commands.
                   ready: bool },                  // max msg length = 59/250

    CommandMessage { timestamp: RoverTimestamp,    // sent by the station to communicate part of a command
                     sequence_complete: bool,      // sequence and possibly tell the rover that the sequence
                     command: String },            // is complete (if sequenceComplete = true). max command length = 58/249

    CommandAck { timestamp: RoverTimestamp,        // sent by the rover to acknowledge a CommandMessage. max msg
                 ack: bool },                      // length = 59/250
}

#[derive(Debug)]
pub struct RoverTimestamp {  // should serialize to 3 bytes (3x +fixint@1)
    pub hour: u8,
    pub minute: u8,
    pub second: u8,
}

impl RoverTimestamp {
    fn serialize(&self, buf: &mut Vec<u8>) {
        buf.push(self.hour);
        buf.push(self.minute);
        buf.push(self.second);
    }

    fn deserialize(&mut self, buf: &mut &[u8]) {
        self.hour = buf[0];
        self.minute = buf[1];
        self.second = buf[2];
    }
}

impl Default for RoverTimestamp {
    fn default() -> Self {
        let local_time: DateTime<Local> = Local::now();
        Self {
            hour: local_time.time().hour() as u8,
            minute: local_time.time().minute() as u8,
            second: local_time.time().second() as u8
        }
    }
}

#[derive(Debug, Default)]
pub struct RoverLocData { // should serialize to 24 bytes (4x float-32@5, +fixint@1, int-16@3)
    pub gps_lat: f32,
    pub gps_long: f32,
    pub gps_alt: f32,
    pub gps_speed: f32,
    pub gps_sats: u8,
    pub mag_hdg: u16,
}

impl RoverLocData {
    fn serialize(&self, buf: &mut Vec<u8>) {
        for byte in self.gps_lat.to_le_bytes().iter() {
            buf.push(*byte);
        }
        for byte in self.gps_long.to_le_bytes().iter() {
            buf.push(*byte);
        }
        for byte in self.gps_alt.to_le_bytes().iter() {
            buf.push(*byte);
        }
        for byte in self.gps_speed.to_le_bytes().iter() {
            buf.push(*byte);
        }
        buf.push(self.gps_sats);
        for byte in self.mag_hdg.to_le_bytes().iter() {
            buf.push(*byte);
        }
    }

    fn deserialize(&mut self, buf: &mut &[u8]) {
        self.gps_lat = f32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
        self.gps_long = f32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]);
        self.gps_alt = f32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]);
        self.gps_speed = f32::from_le_bytes([buf[12], buf[13], buf[14], buf[15]]);
        self.gps_sats = buf[16];
        self.mag_hdg = u16::from_le_bytes([buf[17], buf[18]]);
    }}

impl RoverMessage {
    fn get_message_id(&self) -> u8 {
        match self {
            RoverMessage::TelemetryMessage { .. } => MESSAGE_TELEMETRY,
            RoverMessage::TelemetryAck { .. } => MESSAGE_TELEMETRY_ACK,
            RoverMessage::CommandReady { .. } => MESSAGE_COMMAND_READY,
            RoverMessage::CommandMessage { .. } => MESSAGE_COMMAND,
            RoverMessage::CommandAck { .. } => MESSAGE_COMMAND_ACK
        }
    }

    fn get_message_type(id: u8) -> &'static str {
        match id {
            MESSAGE_TELEMETRY => "MESSAGE_TELEMETRY",
            MESSAGE_TELEMETRY_ACK => "MESSAGE_TELEMETRY_ACK",
            MESSAGE_COMMAND_READY => "MESSAGE_COMMAND_READY",
            MESSAGE_COMMAND => "MESSAGE_COMMAND",
            MESSAGE_COMMAND_ACK => "MESSAGE_COMMAND_ACK",
            _ => "MESSAGE_UNKNOWN"
        }
    }

    // this assumes the string is ASCII. if you give it a UTF-8 string that uses multibyte
    // characters or codes above 127, the rover end will be very confused.
    // TODO: check for that? somehow?
    fn serialize_string(s: &String, buf: &mut Vec<u8>) {
        for byte in s.as_bytes().iter() {
            buf.push(*byte);
        }
        buf.push(0);
    }

    fn deserialize_string(s: &mut String, buf: &mut &[u8]) {
        for byte in buf.iter() {
            if *byte == 0 { break; }
            s.push(char::from_u32(*byte as u32).expect("deserialize_string: invalid character skipped"));
        }
    }

    fn serialize_bool(b: bool, buf: &mut Vec<u8>) {
        if b {
            buf.push(1);
        } else {
            buf.push(0);
        }
    }

    fn deserialize_bool(byte: u8) -> bool {
        byte > 0
    }

    fn serialize_u16(i: &u16, buf: &mut Vec<u8>) {
        for byte in i.to_le_bytes().iter() {
            buf.push(*byte);
        }
    }

    fn deserialize_u16(buf: &mut &[u8]) -> u16 {
        u16::from_le_bytes([buf[0], buf[1]])
    }

    fn serialize_i16(i: &i16, buf: &mut Vec<u8>) {
        for byte in i.to_le_bytes().iter() {
            buf.push(*byte);
        }
    }

    fn deserialize_i16(buf: &mut &[u8]) -> i16 {
        i16::from_le_bytes([buf[0], buf[1]])
    }

    fn serialize(&self, buf: &mut Vec<u8>) -> Result<()> {
        // first byte is buffer length - we'll add that at the end
        // next four bytes are used by RadioHead as TO, FROM, ID, FLAGS
        // so push those onto the Vec before serializing the rest of the payload
        // TODO: if we ever need to put real values for these, do it here
        buf.push(0xff); // TO
        buf.push(0xff); // FROM
        buf.push(0x00); // ID
        buf.push(0x00); // FLAGS
        match self {
            RoverMessage::TelemetryMessage { timestamp,
                                             location,
                                             signal_strength,
                                             free_memory,
                                             status } => {
                buf.push(self.get_message_id());
                timestamp.serialize(buf);
                location.serialize(buf);
                RoverMessage::serialize_i16(signal_strength, buf);
                RoverMessage::serialize_u16(free_memory, buf);
                RoverMessage::serialize_string(status, buf);
            }
            RoverMessage::TelemetryAck { timestamp, ack, command_waiting } => {
                buf.push(self.get_message_id());
                timestamp.serialize(buf);
                RoverMessage::serialize_bool(*ack, buf);
                RoverMessage::serialize_bool(*command_waiting, buf);
            }
            RoverMessage::CommandReady { timestamp, ready } => {
                buf.push(self.get_message_id());
                timestamp.serialize(buf);
                RoverMessage::serialize_bool(*ready, buf);
            }
            RoverMessage::CommandMessage { timestamp, sequence_complete, command } => {
                buf.push(self.get_message_id());
                timestamp.serialize(buf);
                RoverMessage::serialize_bool(*sequence_complete, buf);
                RoverMessage::serialize_string(command, buf);
            }
            RoverMessage::CommandAck { timestamp, ack } => {
                buf.push(self.get_message_id());
                timestamp.serialize(buf);
                RoverMessage::serialize_bool(*ack, buf);
            }
        }
        // finally push the length byte onto the *front* of the buffer
        buf.insert(0, (buf.len() + 1) as u8);
        Ok(())
    }

    fn deserialize(&mut self, buf: &mut [u8; 64]) -> Result<()> {
        // first byte is a length
        // next four bytes are used by RadioHead as TO, FROM, ID, FLAGS
        // so strip those off before deserializing the rest of the payload
        // TODO: if those values are ever needed, grab them here
        match self {
            RoverMessage::TelemetryMessage { ref mut timestamp,
                                             ref mut location,
                                             signal_strength,
                                             free_memory,
                                             ref mut status } => {
                if buf[5] != MESSAGE_TELEMETRY {
                    return Err(format!("Wrong message type: expected MESSAGE_TELEMETRY, got {}", RoverMessage::get_message_type(buf[5])).into());
                }
                timestamp.deserialize(&mut &buf[6..9]);
                location.deserialize(&mut &buf[9..28]);
                *signal_strength = RoverMessage::deserialize_i16(&mut &buf[28..30]);
                *free_memory = RoverMessage::deserialize_u16(&mut &buf[30..32]);
                RoverMessage::deserialize_string(status, &mut &buf[32..]);
            }
            RoverMessage::TelemetryAck { ref mut timestamp, ref mut ack, ref mut command_waiting } => {
                if buf[5] != MESSAGE_TELEMETRY_ACK {
                    return Err(format!("Wrong message type: expected MESSAGE_TELEMETRY_ACK, got {}", RoverMessage::get_message_type(buf[5])).into());
                }
                timestamp.deserialize(&mut &buf[6..9]);
                *ack = RoverMessage::deserialize_bool(buf[9]);
                *command_waiting = RoverMessage::deserialize_bool(buf[10]);
            }
            RoverMessage::CommandReady { ref mut timestamp, ref mut ready } => {
                if buf[5] != MESSAGE_COMMAND_READY {
                    return Err(format!("Wrong message type: expected MESSAGE_COMMAND_READY, got {}", RoverMessage::get_message_type(buf[5])).into());
                }
                timestamp.deserialize(&mut &buf[6..9]);
                *ready = RoverMessage::deserialize_bool(buf[9]);
            }
            RoverMessage::CommandMessage { ref mut timestamp, ref mut sequence_complete, ref mut command } => {
                if buf[5] != MESSAGE_COMMAND {
                    return Err(format!("Wrong message type: expected MESSAGE_COMMAND, got {}", RoverMessage::get_message_type(buf[5])).into());
                }
                timestamp.deserialize(&mut &buf[6..9]);
                *sequence_complete = RoverMessage::deserialize_bool(buf[9]);
                RoverMessage::deserialize_string(command, &mut &buf[29..]);
            }
            RoverMessage::CommandAck { ref mut timestamp, ref mut ack } => {
                if buf[5] != MESSAGE_COMMAND_ACK {
                    return Err(format!("Wrong message type: expected MESSAGE_COMMAND_ACK, got {}", RoverMessage::get_message_type(buf[5])).into());
                }
                timestamp.deserialize(&mut &buf[6..9]);
                *ack = RoverMessage::deserialize_bool(buf[9]);
            }
        }
        Ok(())
    }

    // send msg via radio rfm; wait up to ack_timeout milliseconds for an
    // acknowledgement if needed. ACK logic is encapsulated here - e.g.,
    // a CommandMessage expects an ACK, but a TelemetryAck does not.
    pub fn send(&self,
            rfm: &mut Rfm69<OutputPin, Spi, linux_embedded_hal::Delay>) -> Result<()> {
        let mut max_message_length = 255;
        if USE_ENCRYPTION { max_message_length = 64; }
        // serialize the message
        let mut buf = Vec::new();
        RoverMessage::serialize(&self, &mut buf).unwrap();
        // check message length
        if buf.len() > max_message_length {
            return Err(format!("Cannot send: message too long! {:?}", self).into())
        }
        // send it
        // DEBUG
        // println!("DEBUG: sending this message:");
        // for byte in buf.iter() {
        //     print!("{:x} ", byte);
        // }
        // println!();
        match rfm.send(buf.as_slice()) {
        //match RoverMessage::debug_send(rfm, buf.as_slice()) {
            Err(e) => return Err(format!("Error while sending message: {:?}", e).into()),
            _ => {}
        }
        // receive ack if appropriate
        match self {
            RoverMessage::CommandMessage { .. } => {
                let mut ack: RoverMessage = RoverMessage::CommandAck { timestamp: Default::default(),
                                                                   ack: false };
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
    pub fn receive(&mut self,
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
        // println!("DEBUG: received this message:");
        // for byte in buf.iter() {
        //     print!("{:x} ", byte);
        // }
        // println!();
        println!("Received message from rover; signal strength {}", rfm.rssi());
        // deserialize the message
        match self.deserialize(&mut buf) {
            Err(e) => return Err(format!("Error while deserializing response: {:?}", e).into()),
            _ => {}
        }
        // ACK if necessary
        match self {
            RoverMessage::TelemetryMessage{..} => {
                let ack: RoverMessage = RoverMessage::TelemetryAck { timestamp: Default::default(),
                                                                     ack: true,
                                                                     command_waiting: false };
                thread::sleep(Duration::from_millis(MSG_DELAY));
                ack.send(rfm)?
            },
            _ => (), // no ack needed
        }
        Ok(())
    }
}
