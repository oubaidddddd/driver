/// utils.rs - Utils Layer (docan_driver/src/utils.rs)
/// Provides utility structs: ChecksumCalculator, MessageValidator,
/// TimeoutConfig, TimeoutCounter, Crc32Calculator, CrcConfig

// ─── ChecksumCalculator ──────────────────────────────────────────────────────

/// Static utility functions for checksum calculation
pub struct ChecksumCalculator;

impl ChecksumCalculator {
    /// Simple sum of all bytes, truncated to u8
    pub fn sum_validate(data: &[u8]) -> u8 {
        data.iter().fold(0u8, |acc, &b| acc.wrapping_add(b))
    }

    /// XOR of all bytes
    pub fn xor_validate(data: &[u8]) -> u8 {
        data.iter().fold(0u8, |acc, &b| acc ^ b)
    }

    /// Fletcher-16 checksum
    pub fn fletcher(data: &[u8]) -> u16 {
        let mut sum1: u16 = 0;
        let mut sum2: u16 = 0;
        for &b in data {
            sum1 = (sum1 + b as u16) % 255;
            sum2 = (sum2 + sum1) % 255;
        }
        (sum2 << 8) | sum1
    }
}

// ─── MessageValidator ─────────────────────────────────────────────────────────

/// Static utility for message validation (ISO 15765-2)
pub struct MessageValidator;

impl MessageValidator {
    /// Maximum message size per ISO-TP: 4095 bytes
    const MAX_MSG_SIZE: usize = 4095;
    /// Max bytes per CAN frame payload
    const FRAME_PAYLOAD: usize = 7;

    /// Checks if the message length is valid (1..=4095)
    pub fn is_valid_length(msg_size: usize) -> bool {
        msg_size > 0 && msg_size <= Self::MAX_MSG_SIZE
    }

    /// Calculates how many CAN frames are needed for a message of given length
    pub fn calc_frames_needed(msg_len: usize) -> u32 {
        if msg_len <= 7 {
            return 1; // Single Frame
        }
        // First frame carries 6 bytes, consecutive frames carry 7 bytes each
        let remaining = msg_len - 6;
        1 + ((remaining + Self::FRAME_PAYLOAD - 1) / Self::FRAME_PAYLOAD) as u32
    }

    /// Validates that the number of frames matches expected segmentation
    pub fn is_valid_segmentation(msg_len: usize, frames: u32) -> bool {
        Self::calc_frames_needed(msg_len) == frames
    }
}

// ─── TimeoutConfig ───────────────────────────────────────────────────────────

/// Configuration for ISO-TP timing parameters
#[derive(Debug, Clone)]
pub struct TimeoutConfig {
    pub reception_timeout_ms: u32,
    pub flow_control_timeout_ms: u32,
    pub transmission_timeout_ms: u32,
    pub frame_separation_time_ms: u8,
}

impl TimeoutConfig {
    pub fn new(rx: u32, fc: u32, tx: u32) -> Self {
        TimeoutConfig {
            reception_timeout_ms: rx,
            flow_control_timeout_ms: fc,
            transmission_timeout_ms: tx,
            frame_separation_time_ms: 10,
        }
    }

    /// Automotive-grade timing per ISO 15765-2
    pub fn automotive() -> Self {
        TimeoutConfig {
            reception_timeout_ms: 1000,
            flow_control_timeout_ms: 250,
            transmission_timeout_ms: 25,
            frame_separation_time_ms: 10,
        }
    }

    /// Realtime / low-latency timing
    pub fn realtime() -> Self {
        TimeoutConfig {
            reception_timeout_ms: 100,
            flow_control_timeout_ms: 50,
            transmission_timeout_ms: 5,
            frame_separation_time_ms: 1,
        }
    }

    pub fn with_reception_timeout(mut self, ms: u32) -> Self {
        self.reception_timeout_ms = ms;
        self
    }

    pub fn with_flow_control_timeout(mut self, ms: u32) -> Self {
        self.flow_control_timeout_ms = ms;
        self
    }
}

impl Default for TimeoutConfig {
    fn default() -> Self {
        Self::automotive()
    }
}

// ─── TimeoutCounter ──────────────────────────────────────────────────────────

/// Runtime timeout counter — tracks elapsed time against a configured timeout.
/// Used in TxState and RxState for N_As, N_Bs, N_Cs, N_Ar, N_Br, N_Cr timers.
#[derive(Debug, Clone)]
pub struct TimeoutCounter {
    elapsed_ms: u32,
    timeout_ms: u32,
}

impl TimeoutCounter {
    pub fn new(timeout_ms: u32) -> Self {
        TimeoutCounter {
            elapsed_ms: 0,
            timeout_ms,
        }
    }

    /// Advance the counter by delta_ms milliseconds
    pub fn update(&mut self, delta_ms: u32) {
        self.elapsed_ms = self.elapsed_ms.saturating_add(delta_ms);
    }

    /// Returns true if the timeout has been reached
    pub fn is_expired(&self) -> bool {
        self.elapsed_ms >= self.timeout_ms
    }

    /// Remaining time before expiry (0 if already expired)
    pub fn remaining_ms(&self) -> u32 {
        self.timeout_ms.saturating_sub(self.elapsed_ms)
    }

    /// Reset the counter to zero
    pub fn reset(&mut self) {
        self.elapsed_ms = 0;
    }
}

// ─── CrcConfig ───────────────────────────────────────────────────────────────

/// Configuration for CRC-32/16 calculation
/// Used by Crc32Calculator
#[derive(Debug, Clone)]
pub struct CrcConfig {
    pub enable_crc: bool,
    pub polynomial: u32,
    pub initial_value: u32,
    pub final_xor: u32,
    pub reflect_input: bool,
    pub reflect_output: bool,
}

impl CrcConfig {
    /// Standard CRC-32 (IEEE 802.3)
    pub fn new_crc32() -> Self {
        CrcConfig {
            enable_crc: true,
            polynomial: 0x04C11DB7,
            initial_value: 0xFFFFFFFF,
            final_xor: 0xFFFFFFFF,
            reflect_input: true,
            reflect_output: true,
        }
    }

    pub fn enable(&mut self) -> &mut Self {
        self.enable_crc = true;
        self
    }

    pub fn disable(&mut self) -> &mut Self {
        self.enable_crc = false;
        self
    }
}

impl Default for CrcConfig {
    fn default() -> Self {
        Self::new_crc32()
    }
}

// ─── Crc32Calculator ─────────────────────────────────────────────────────────

/// CRC-32 calculator with configurable polynomial and options
pub struct Crc32Calculator {
    config: CrcConfig,
    table: [u32; 256],
}

impl Crc32Calculator {
    pub fn new() -> Self {
        let config = CrcConfig::default();
        let table = Self::build_table(&config);
        Crc32Calculator { config, table }
    }

    pub fn with_config(config: CrcConfig) -> Self {
        let table = Self::build_table(&config);
        Crc32Calculator { config, table }
    }

    fn reflect_u32(mut val: u32, bits: u8) -> u32 {
        let mut reflected = 0u32;
        for i in 0..bits {
            if val & 1 != 0 {
                reflected |= 1 << (bits - 1 - i);
            }
            val >>= 1;
        }
        reflected
    }

    pub fn build_table(config: &CrcConfig) -> [u32; 256] {
        let mut table = [0u32; 256];
        for i in 0..256u32 {
            let mut crc = i << 24;
            for _ in 0..8 {
                crc = if crc & 0x80000000 != 0 {
                    (crc << 1) ^ config.polynomial
                } else {
                    crc << 1
                };
            }
            table[i as usize] = crc;
        }
        table
    }

    pub fn calculate(&self, data: &[u8]) -> u32 {
        if !self.config.enable_crc {
            return 0;
        }
        let mut crc = self.config.initial_value;
        for &byte in data {
            let b = if self.config.reflect_input {
                Self::reflect_u32(byte as u32, 8) as u8
            } else {
                byte
            };
            let idx = ((crc >> 24) ^ b as u32) as usize;
            crc = (crc << 8) ^ self.table[idx];
        }
        if self.config.reflect_output {
            crc = Self::reflect_u32(crc, 32);
        }
        crc ^ self.config.final_xor
    }

    pub fn validate(&self, data: &[u8], expected: u32) -> bool {
        self.calculate(data) == expected
    }
}
