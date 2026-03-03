/// demo_single_frame/src/main.rs
/// Demo ISO-TP Single Frame : STM32H7 <-> PCAN-USB
///
/// Scénario :
///   1. STM32H7 envoie un Single Frame (SF) avec un payload UDS
///   2. STM32H7 attend un Single Frame en réponse du PC (PCAN-USB)
///   3. La LED verte clignote si OK, LED rouge si erreur/timeout

#![no_std]
#![no_main]

use cortex_m::delay::Delay;
use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use stm32h7xx_hal::{
    can::Can,
    pac,
    prelude::*,
    rcc::rec::CanClkSel,
};

// bxCAN driver (stm32h7xx-hal re-exports bxcan)
use bxcan::{filter::Mask32, Frame, Id, StandardId};

mod frame;
mod transport;
mod utils;

use frame::{CanFrame, SingleFrame, IsoTpError};
use heapless::Vec;
use utils::TimeoutCounter;

// ── CAN IDs ──────────────────────────────────────────────────────────────────
/// STM32H7 transmits on this ID  (Tester → ECU in UDS convention)
const TX_CAN_ID: u16 = 0x7E0;
/// STM32H7 listens on this ID   (ECU → Tester)
const RX_CAN_ID: u16 = 0x7E8;

// ── UDS payload to send ───────────────────────────────────────────────────────
/// UDS TesterPresent request: SID=0x3E, SubFunction=0x00  (fits in 1 SF)
const UDS_REQUEST: &[u8] = &[0x02, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x00];

#[entry]
fn main() -> ! {
    // ── Peripherals ───────────────────────────────────────────────────────────
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // Clock config: 400 MHz sysclk, APB1 = 100 MHz (CAN clock source)
    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.freeze();
    let rcc = dp.RCC.constrain();
    let ccdr = rcc
        .sys_ck(400.MHz())
        .pclk1(100.MHz())
        .freeze(pwrcfg, &dp.SYSCFG);

    let mut delay = Delay::new(cp.SYST, ccdr.clocks);

    // ── GPIO ──────────────────────────────────────────────────────────────────
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);

    // STM32H743 Nucleo: LED1=PB0 (green), LED3=PB14 (red)
    let mut led_ok  = gpiob.pb0.into_push_pull_output();
    let mut led_err = gpiob.pb14.into_push_pull_output();

    // CAN1 pins: PB8=RX, PB9=TX (AF9)
    let can_rx = gpiob.pb8.into_alternate::<9>();
    let can_tx = gpiob.pb9.into_alternate::<9>();

    // ── CAN peripheral ────────────────────────────────────────────────────────
    // CAN clock source = HSE (8 MHz typical on Nucleo)
    let can = dp.FDCAN1; // bxCAN on H7 uses FDCAN peripheral in classic mode

    // Use stm32h7xx-hal CAN wrapper → bxcan
    let mut can = {
        let can = Can::new(can, can_rx, can_tx, ccdr.peripheral.FDCAN, &ccdr.clocks);
        bxcan::Can::builder(can)
            // 500 kbit/s @ APB1=100MHz: prescaler=10, ts1=13, ts2=2
            // Tq = 1/(100MHz/10) = 100ns → NBT = (1+13+2)*100ns = 1600ns → 625kbit... adjust:
            // For 500kbit/s: prescaler=10, ts1=11, ts2=4 → (1+11+4)=16 Tq → 1/(100e6/10/16)=500kbps
            .set_bit_timing(0x001C000B) // SJW=1, ts2=4, ts1=12, prescaler=12 → ~500kbps
            .leave_disabled()
    };

    // Accept only frames with RX_CAN_ID
    can.modify_filters().enable_bank(
        0,
        Mask32::frames_with_std_id(
            StandardId::new(RX_CAN_ID).unwrap(),
            StandardId::new(0x7FF).unwrap(),
        ),
    );

    // Enable CAN
    can.enable_non_blocking().ok();
    nb::block!(can.enable_non_blocking()).unwrap();

    info!("=== DoCAN Single Frame Demo — STM32H7 <-> PCAN-USB ===");
    info!("TX ID: 0x{:03X}  |  RX ID: 0x{:03X}", TX_CAN_ID, RX_CAN_ID);

    // ── Build ISO-TP Single Frame ──────────────────────────────────────────────
    let mut sf_data: Vec<u8, 7> = Vec::new();
    for &b in UDS_REQUEST {
        sf_data.push(b).ok();
    }
    let sf = SingleFrame::new(sf_data).expect("SF build failed");
    let raw_tx = sf.to_can_data();

    info!("SF payload: {:02X}", raw_tx.as_slice());

    // ── Main demo loop ────────────────────────────────────────────────────────
    loop {
        // ── STEP 1: Transmit Single Frame ─────────────────────────────────────
        let tx_id = StandardId::new(TX_CAN_ID).unwrap();
        let tx_frame = Frame::new_data(tx_id, raw_tx.as_slice());
        match tx_frame {
            Ok(f) => {
                nb::block!(can.transmit(&f)).ok();
                info!(">> TX Single Frame [0x{:03X}]: {:02X}", TX_CAN_ID, raw_tx.as_slice());
            }
            Err(_) => {
                error!("Failed to build bxcan frame");
            }
        }

        // ── STEP 2: Wait for response Single Frame (timeout = 1000 ms) ────────
        let mut timeout = TimeoutCounter::new(1000);
        let mut response_received = false;

        while !timeout.is_expired() {
            if let Ok(rx_frame) = nb::block!(can.receive()) {
                if let Some(data) = rx_frame.data() {
                    // Parse as ISO-TP frame
                    let mut raw: Vec<u8, 8> = Vec::new();
                    for &b in data.iter() {
                        raw.push(b).ok();
                    }

                    match CanFrame::from_can_data(&raw) {
                        Ok(CanFrame::SingleFrame(resp_sf)) => {
                            info!(
                                "<< RX Single Frame [0x{:03X}]: DL={} DATA={:02X}",
                                RX_CAN_ID,
                                resp_sf.data_length,
                                resp_sf.data.as_slice()
                            );

                            // Validate UDS Positive Response: SID = 0x3E + 0x40 = 0x7E
                            if resp_sf.data.len() >= 2
                                && resp_sf.data[0] == 0x02
                                && resp_sf.data[1] == 0x7E
                            {
                                info!("✓ UDS TesterPresent Positive Response OK");
                                blink_ok(&mut led_ok, &mut delay);
                            } else {
                                warn!("? Unexpected response payload");
                                blink_err(&mut led_err, &mut delay);
                            }
                            response_received = true;
                            break;
                        }
                        Ok(other) => {
                            warn!("Unexpected frame type: {:?}", other.frame_type() as u8);
                        }
                        Err(e) => {
                            error!("Frame parse error");
                        }
                    }
                }
            }
            // Simulate 1ms tick (in real project: use SysTick interrupt)
            delay.delay_ms(1u32);
            timeout.update(1);
        }

        if !response_received {
            error!("✗ Timeout — No response from PCAN-USB");
            blink_err(&mut led_err, &mut delay);
        }

        // Wait 2 seconds before next cycle
        delay.delay_ms(2000u32);
    }
}

/// Blink green LED 3 times (success)
fn blink_ok<T: embedded_hal::digital::v2::OutputPin>(
    led: &mut T,
    delay: &mut Delay,
) {
    for _ in 0..3 {
        led.set_high().ok();
        delay.delay_ms(150u32);
        led.set_low().ok();
        delay.delay_ms(150u32);
    }
}

/// Blink red LED 5 times fast (error)
fn blink_err<T: embedded_hal::digital::v2::OutputPin>(
    led: &mut T,
    delay: &mut Delay,
) {
    for _ in 0..5 {
        led.set_high().ok();
        delay.delay_ms(80u32);
        led.set_low().ok();
        delay.delay_ms(80u32);
    }
}
