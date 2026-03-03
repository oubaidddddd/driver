/// main.rs - DoCAN Driver Entry Point for STM32H7
/// Project: ISO 15765-2 Transport Layer in Rust (no_std embedded)

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

mod frame;
mod transport;
mod utils;

use frame::CanFrame;
use heapless::Vec;
use transport::{Config, TransportLayer};

#[entry]
fn main() -> ! {
    info!("DoCAN Driver — ISO 15765-2 on STM32H7");

    // ── Hardware init ────────────────────────────────────────────────────────
    // let dp = stm32h7xx_hal::pac::Peripherals::take().unwrap();
    // let rcc = dp.RCC.constrain();
    // let clocks = rcc.cfgr.sysclk(400.MHz()).freeze();
    // (CAN peripheral init omitted: depends on board BSP)

    // ── Transport Layer init ─────────────────────────────────────────────────
    let config = Config::new(
        0x7E0, // UDS Tester Present TX ID
        0x7E8, // UDS ECU response RX ID
    );
    let mut tl = TransportLayer::new(config);

    // ── Demo: send a multi-frame UDS message ─────────────────────────────────
    let uds_request: [u8; 20] = [
        0x10, 0x03, 0x27, 0x01, 0xAA, 0xBB, 0x22, 0xF1,
        0x90, 0x19, 0x02, 0xFF, 0x31, 0x01, 0xFF, 0x00,
        0x3E, 0x00, 0x85, 0x82,
    ];

    match tl.send(&uds_request) {
        Ok(_) => info!("Message queued for transmission"),
        Err(e) => error!("Send error: {:?}", e),
    }

    // Main loop — in a real system this would be interrupt/DMA driven
    loop {
        // 1. Get next frame to send over CAN bus
        match tl.get_next_tx_frame() {
            Ok(Some(frame)) => {
                let raw = frame.to_can_data();
                info!("TX frame: {:02X}", raw.as_slice());
                // hal_can_send(raw); // → CAN peripheral send call
            }
            Ok(None) => {}
            Err(e) => error!("TX error: {:?}", e),
        }

        // 2. Simulate receiving a FlowControl frame from ECU
        // In production: triggered by CAN RX interrupt
        // let rx_bytes = hal_can_receive();
        // if let Ok(frame) = CanFrame::from_can_data(&rx_bytes) {
        //     match tl.handle_rx_frame(frame) {
        //         Ok(Some(payload)) => info!("Message received: {} bytes", payload.len()),
        //         Ok(None) => {},
        //         Err(e) => error!("RX error: {:?}", e),
        //     }
        // }

        // 3. Update timers (call every 1ms via SysTick)
        tl.update_timers(1);

        cortex_m::asm::wfe();
    }
}
