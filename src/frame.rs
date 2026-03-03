/// frame.rs - ISO 15765-2 CAN Frames
/// Implements all frame types defined by the DoCAN standard

use heapless::Vec;

/// Represents the type of a CAN frame in the ISO-TP protocol
#[derive(Debug, Clone, PartialEq)]
pub enum FrameType {
    SingleFrame,
    FirstFrame,
    ConsecutiveFrame,
    FlowControl,
}

/// FlowControl status codes
#[derive(Debug, Clone, PartialEq)]
pub enum FlowControlStatus {
    ContinueSending = 0x00,
    Wait = 0x01,
    Overflow = 0x02,
}

impl TryFrom<u8> for FlowControlStatus {
    type Error = IsoTpError;
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x00 => Ok(FlowControlStatus::ContinueSending),
            0x01 => Ok(FlowControlStatus::Wait),
            0x02 => Ok(FlowControlStatus::Overflow),
            _ => Err(IsoTpError::InvalidFlowControlStatus),
        }
    }
}

/// Error types for the DoCAN driver
#[derive(Debug, Clone, PartialEq)]
pub enum IsoTpError {
    InvalidFrameType,
    InvalidLength,
    InvalidSequenceNumber,
    InvalidFlowControlStatus,
    BufferOverflow,
    Timeout,
    UnexpectedFrame,
}

/// Single Frame (SF): carries data of 1-7 bytes
#[derive(Debug, Clone)]
pub struct SingleFrame {
    pub data_length: u8, // 0-7
    pub data: Vec<u8, 7>,
}

impl SingleFrame {
    pub fn new(data: Vec<u8, 7>) -> Result<Self, IsoTpError> {
        let len = data.len();
        if len == 0 || len > 7 {
            return Err(IsoTpError::InvalidLength);
        }
        Ok(SingleFrame {
            data_length: len as u8,
            data,
        })
    }

    pub fn to_can_data(&self) -> Vec<u8, 8> {
        let mut out: Vec<u8, 8> = Vec::new();
        out.push(self.data_length & 0x0F).ok();
        for &b in &self.data {
            out.push(b).ok();
        }
        // Pad to 8 bytes
        while out.len() < 8 {
            out.push(0xCC).ok();
        }
        out
    }
}

/// First Frame (FF): first frame of a multi-frame message
#[derive(Debug, Clone)]
pub struct FirstFrame {
    pub total_length: u16, // 8-4095
    pub data: Vec<u8, 6>,
}

impl FirstFrame {
    pub fn new(len: u16, data: Vec<u8, 6>) -> Result<Self, IsoTpError> {
        if len < 8 || len > 4095 {
            return Err(IsoTpError::InvalidLength);
        }
        Ok(FirstFrame {
            total_length: len,
            data,
        })
    }

    pub fn to_can_data(&self) -> Vec<u8, 8> {
        let mut out: Vec<u8, 8> = Vec::new();
        let nibble_high = (0x10 | ((self.total_length >> 8) & 0x0F)) as u8;
        let nibble_low = (self.total_length & 0xFF) as u8;
        out.push(nibble_high).ok();
        out.push(nibble_low).ok();
        for &b in &self.data {
            out.push(b).ok();
        }
        out
    }
}

/// Consecutive Frame (CF): subsequent frames of a multi-frame message
#[derive(Debug, Clone)]
pub struct ConsecutiveFrame {
    pub sequence_number: u8, // 0-15
    pub data: Vec<u8, 7>,
}

impl ConsecutiveFrame {
    pub fn new(sn: u8, data: Vec<u8, 7>) -> Result<Self, IsoTpError> {
        if sn > 15 {
            return Err(IsoTpError::InvalidSequenceNumber);
        }
        Ok(ConsecutiveFrame {
            sequence_number: sn,
            data,
        })
    }

    pub fn to_can_data(&self) -> Vec<u8, 8> {
        let mut out: Vec<u8, 8> = Vec::new();
        out.push(0x20 | (self.sequence_number & 0x0F)).ok();
        for &b in &self.data {
            out.push(b).ok();
        }
        while out.len() < 8 {
            out.push(0xCC).ok();
        }
        out
    }
}

/// Flow Control Frame (FC): controls the flow of consecutive frames
#[derive(Debug, Clone)]
pub struct FlowControlFrame {
    pub status: FlowControlStatus,
    pub block_size: u8,
    pub separation_time: u8,
}

impl FlowControlFrame {
    pub fn new(
        status: FlowControlStatus,
        block: u8,
        stmin: u8,
    ) -> Result<Self, IsoTpError> {
        Ok(FlowControlFrame {
            status,
            block_size: block,
            separation_time: stmin,
        })
    }

    pub fn to_can_data(&self) -> Vec<u8, 8> {
        let mut out: Vec<u8, 8> = Vec::new();
        out.push(0x30 | (self.status.clone() as u8)).ok();
        out.push(self.block_size).ok();
        out.push(self.separation_time).ok();
        while out.len() < 8 {
            out.push(0x00).ok();
        }
        out
    }
}

/// Main CAN Frame enum encapsulating all ISO-TP frame types
#[derive(Debug, Clone)]
pub enum CanFrame {
    SingleFrame(SingleFrame),
    FirstFrame(FirstFrame),
    ConsecutiveFrame(ConsecutiveFrame),
    FlowControl(FlowControlFrame),
}

impl CanFrame {
    /// Parse raw CAN data bytes into the appropriate frame type
    pub fn from_can_data(data: &Vec<u8, 8>) -> Result<CanFrame, IsoTpError> {
        if data.is_empty() {
            return Err(IsoTpError::InvalidFrameType);
        }
        let nibble = (data[0] >> 4) & 0x0F;
        match nibble {
            0x0 => {
                let dl = data[0] & 0x0F;
                let mut payload: Vec<u8, 7> = Vec::new();
                for i in 1..=(dl as usize).min(7) {
                    payload.push(data[i]).ok();
                }
                Ok(CanFrame::SingleFrame(SingleFrame::new(payload)?))
            }
            0x1 => {
                let total = (((data[0] & 0x0F) as u16) << 8) | data[1] as u16;
                let mut payload: Vec<u8, 6> = Vec::new();
                for i in 2..8 {
                    if i < data.len() {
                        payload.push(data[i]).ok();
                    }
                }
                Ok(CanFrame::FirstFrame(FirstFrame::new(total, payload)?))
            }
            0x2 => {
                let sn = data[0] & 0x0F;
                let mut payload: Vec<u8, 7> = Vec::new();
                for i in 1..8 {
                    if i < data.len() {
                        payload.push(data[i]).ok();
                    }
                }
                Ok(CanFrame::ConsecutiveFrame(ConsecutiveFrame::new(sn, payload)?))
            }
            0x3 => {
                let status = FlowControlStatus::try_from(data[0] & 0x0F)?;
                let bs = if data.len() > 1 { data[1] } else { 0 };
                let stmin = if data.len() > 2 { data[2] } else { 0 };
                Ok(CanFrame::FlowControl(FlowControlFrame::new(status, bs, stmin)?))
            }
            _ => Err(IsoTpError::InvalidFrameType),
        }
    }

    /// Serialize the frame into raw CAN data bytes
    pub fn to_can_data(&self) -> Vec<u8, 8> {
        match self {
            CanFrame::SingleFrame(sf) => sf.to_can_data(),
            CanFrame::FirstFrame(ff) => ff.to_can_data(),
            CanFrame::ConsecutiveFrame(cf) => cf.to_can_data(),
            CanFrame::FlowControl(fc) => fc.to_can_data(),
        }
    }

    /// Returns the FrameType of this frame
    pub fn frame_type(&self) -> FrameType {
        match self {
            CanFrame::SingleFrame(_) => FrameType::SingleFrame,
            CanFrame::FirstFrame(_) => FrameType::FirstFrame,
            CanFrame::ConsecutiveFrame(_) => FrameType::ConsecutiveFrame,
            CanFrame::FlowControl(_) => FrameType::FlowControl,
        }
    }
}
