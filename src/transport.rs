/// transport.rs - ISO 15765-2 Transport Layer
/// Implements TransportLayer, TxState, RxState, Config

use heapless::Vec;
use crate::frame::{CanFrame, FlowControlFrame, FlowControlStatus, IsoTpError};
use crate::utils::{TimeoutConfig, TimeoutCounter};

// ─── State ───────────────────────────────────────────────────────────────────

#[derive(Debug, Clone, PartialEq)]
pub enum State {
    Idle,
    Sending,
    AwaitingFlowControl,
    Receiving,
}

// ─── FlowControlConfig ────────────────────────────────────────────────────────

#[derive(Debug, Clone)]
pub struct FlowControlConfig {
    pub block_size: u8,
    pub separation_time_ms: u8,
}

impl Default for FlowControlConfig {
    fn default() -> Self {
        FlowControlConfig {
            block_size: 0,       // 0 = send all frames without waiting
            separation_time_ms: 10,
        }
    }
}

// ─── Config ───────────────────────────────────────────────────────────────────

#[derive(Debug, Clone)]
pub struct Config {
    pub tx_id: u32,
    pub rx_id: u32,
    pub flow_control: FlowControlConfig,
    pub reception_timeout_ms: u32,
}

impl Config {
    pub fn new(tx_id: u32, rx_id: u32) -> Self {
        Config {
            tx_id,
            rx_id,
            flow_control: FlowControlConfig::default(),
            reception_timeout_ms: 1000,
        }
    }
}

// ─── TxState ─────────────────────────────────────────────────────────────────
/// Manages the sending state machine (N_As, N_Bs, N_Cs — ISO 15765-2)

pub struct TxState {
    pub state: State,
    pub data: Vec<u8, 4096>,
    pub total_length: u16,
    pub current_pos: u16,
    pub sequence_number: u8,
    pub frames_in_block: u8,
    pub timeout_counter: TimeoutCounter,
}

impl TxState {
    pub fn new() -> Self {
        TxState {
            state: State::Idle,
            data: Vec::new(),
            total_length: 0,
            current_pos: 0,
            sequence_number: 0,
            frames_in_block: 0,
            timeout_counter: TimeoutCounter::new(250),
        }
    }

    pub fn reset(&mut self) {
        self.state = State::Idle;
        self.data.clear();
        self.total_length = 0;
        self.current_pos = 0;
        self.sequence_number = 0;
        self.frames_in_block = 0;
        self.timeout_counter.reset();
    }
}

// ─── RxState ─────────────────────────────────────────────────────────────────
/// Manages the reception state machine (N_Ar, N_Br, N_Cr — ISO 15765-2)

pub struct RxState {
    pub state: State,
    pub data: Vec<u8, 4096>,
    pub total_length: u16,
    pub next_sequence: u8,
    pub timeout_counter: TimeoutCounter,
    pub block_counter: u8,
}

impl RxState {
    pub fn new() -> Self {
        RxState {
            state: State::Idle,
            data: Vec::new(),
            total_length: 0,
            next_sequence: 1,
            timeout_counter: TimeoutCounter::new(1000),
            block_counter: 0,
        }
    }

    pub fn reset(&mut self) {
        self.state = State::Idle;
        self.data.clear();
        self.total_length = 0;
        self.next_sequence = 1;
        self.block_counter = 0;
        self.timeout_counter.reset();
    }
}

// ─── TransportLayer ───────────────────────────────────────────────────────────

pub struct TransportLayer {
    config: Config,
    tx: TxState,
    rx: RxState,
}

impl TransportLayer {
    /// Create a new TransportLayer with the given configuration
    pub fn new(config: Config) -> Self {
        TransportLayer {
            config,
            tx: TxState::new(),
            rx: RxState::new(),
        }
    }

    // ── Transmission ─────────────────────────────────────────────────────────

    /// Queue data for transmission
    pub fn send(&mut self, data: &[u8]) -> Result<(), IsoTpError> {
        if data.is_empty() || data.len() > 4095 {
            return Err(IsoTpError::InvalidLength);
        }
        self.tx.reset();
        for &b in data {
            self.tx.data.push(b).map_err(|_| IsoTpError::BufferOverflow)?;
        }
        self.tx.total_length = data.len() as u16;
        self.tx.state = State::Sending;
        Ok(())
    }

    /// Get the next CAN frame to transmit
    pub fn get_next_tx_frame(&mut self) -> Result<Option<CanFrame>, IsoTpError> {
        match self.tx.state {
            State::Idle => Ok(None),
            State::Sending => {
                if self.tx.total_length <= 7 {
                    // Single Frame
                    let mut payload: Vec<u8, 7> = Vec::new();
                    for &b in self.tx.data.iter().take(7) {
                        payload.push(b).ok();
                    }
                    let sf = crate::frame::SingleFrame::new(payload)?;
                    self.tx.state = State::Idle;
                    Ok(Some(CanFrame::SingleFrame(sf)))
                } else if self.tx.current_pos == 0 {
                    // First Frame
                    let mut payload: Vec<u8, 6> = Vec::new();
                    for &b in self.tx.data.iter().take(6) {
                        payload.push(b).ok();
                    }
                    self.tx.current_pos = 6;
                    self.tx.sequence_number = 1;
                    let ff = crate::frame::FirstFrame::new(self.tx.total_length, payload)?;
                    self.tx.state = State::AwaitingFlowControl;
                    Ok(Some(CanFrame::FirstFrame(ff)))
                } else {
                    // Consecutive Frame
                    let start = self.tx.current_pos as usize;
                    let end = (start + 7).min(self.tx.total_length as usize);
                    let mut payload: Vec<u8, 7> = Vec::new();
                    for &b in self.tx.data[start..end].iter() {
                        payload.push(b).ok();
                    }
                    let sn = self.tx.sequence_number & 0x0F;
                    self.tx.sequence_number = (self.tx.sequence_number + 1) % 16;
                    self.tx.current_pos = end as u16;
                    let cf = crate::frame::ConsecutiveFrame::new(sn, payload)?;
                    if self.tx.current_pos >= self.tx.total_length {
                        self.tx.state = State::Idle;
                    }
                    Ok(Some(CanFrame::ConsecutiveFrame(cf)))
                }
            }
            State::AwaitingFlowControl => Ok(None),
            State::Receiving => Err(IsoTpError::UnexpectedFrame),
        }
    }

    /// Handle a received FlowControl frame
    pub fn handle_flow_control(&mut self, fc: &FlowControlFrame) -> Result<(), IsoTpError> {
        match fc.status {
            FlowControlStatus::ContinueSending => {
                self.tx.frames_in_block = fc.block_size;
                self.tx.state = State::Sending;
                Ok(())
            }
            FlowControlStatus::Wait => Ok(()),
            FlowControlStatus::Overflow => {
                self.tx.reset();
                Err(IsoTpError::BufferOverflow)
            }
        }
    }

    // ── Réception ────────────────────────────────────────────────────────────

    /// Process an incoming CAN frame
    pub fn handle_rx_frame(&mut self, frame: CanFrame) -> Result<Option<Vec<u8, 4096>>, IsoTpError> {
        match frame {
            CanFrame::SingleFrame(sf) => self.handle_single_frame(sf),
            CanFrame::FirstFrame(ff) => {
                self.handle_first_frame(ff)?;
                Ok(None)
            }
            CanFrame::ConsecutiveFrame(cf) => self.handle_consecutive_frame(cf),
            CanFrame::FlowControl(fc) => {
                self.handle_flow_control(&fc)?;
                Ok(None)
            }
        }
    }

    fn handle_single_frame(
        &mut self,
        sf: crate::frame::SingleFrame,
    ) -> Result<Option<Vec<u8, 4096>>, IsoTpError> {
        self.rx.reset();
        let mut out: Vec<u8, 4096> = Vec::new();
        for &b in sf.data.iter().take(sf.data_length as usize) {
            out.push(b).map_err(|_| IsoTpError::BufferOverflow)?;
        }
        Ok(Some(out))
    }

    fn handle_first_frame(
        &mut self,
        ff: crate::frame::FirstFrame,
    ) -> Result<(), IsoTpError> {
        self.rx.reset();
        self.rx.total_length = ff.total_length;
        self.rx.state = State::Receiving;
        for &b in &ff.data {
            self.rx.data.push(b).map_err(|_| IsoTpError::BufferOverflow)?;
        }
        Ok(())
    }

    fn handle_consecutive_frame(
        &mut self,
        cf: crate::frame::ConsecutiveFrame,
    ) -> Result<Option<Vec<u8, 4096>>, IsoTpError> {
        if self.rx.state != State::Receiving {
            return Err(IsoTpError::UnexpectedFrame);
        }
        if cf.sequence_number != self.rx.next_sequence {
            return Err(IsoTpError::InvalidSequenceNumber);
        }
        self.rx.next_sequence = (self.rx.next_sequence + 1) % 16;
        for &b in &cf.data {
            if self.rx.data.len() < self.rx.total_length as usize {
                self.rx.data.push(b).map_err(|_| IsoTpError::BufferOverflow)?;
            }
        }
        self.rx.timeout_counter.reset();
        if self.rx.data.len() >= self.rx.total_length as usize {
            let mut out: Vec<u8, 4096> = Vec::new();
            for &b in self.rx.data.iter().take(self.rx.total_length as usize) {
                out.push(b).ok();
            }
            self.rx.reset();
            return Ok(Some(out));
        }
        Ok(None)
    }

    // ── Utilitaires ──────────────────────────────────────────────────────────

    /// Build a FlowControl frame using current config
    pub fn create_flow_control(&self) -> Result<CanFrame, IsoTpError> {
        let fc = FlowControlFrame::new(
            FlowControlStatus::ContinueSending,
            self.config.flow_control.block_size,
            self.config.flow_control.separation_time_ms,
        )?;
        Ok(CanFrame::FlowControl(fc))
    }

    /// Advance all internal timers by delta_ms
    pub fn update_timers(&mut self, data_ms: u32) {
        self.tx.timeout_counter.update(data_ms);
        self.rx.timeout_counter.update(data_ms);
    }

    /// Returns true if reception is in progress
    pub fn is_rx_busy(&self) -> bool {
        self.rx.state == State::Receiving
    }

    /// Returns true if transmission is in progress
    pub fn is_tx_busy(&self) -> bool {
        self.tx.state != State::Idle
    }
}
