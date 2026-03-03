#!/usr/bin/env python3
"""
pcan_demo.py — Demo Single Frame ISO-TP via PCAN-USB
=====================================================
Côté PC : répond aux Single Frames envoyés par le STM32H7.

Dépendances :
    pip install python-can

Connexion physique :
    PCAN-USB <----CAN H/L----> Transceiver CAN (SN65HVD230) <--> STM32H7 (FDCAN1)
    Terminaison 120Ω aux deux extrémités du bus.

Usage :
    python pcan_demo.py [--bitrate 500000] [--tx 0x7E8] [--rx 0x7E0]
"""

import can
import time
import argparse
import struct
from dataclasses import dataclass
from typing import Optional

# ── ISO-TP Frame Types ────────────────────────────────────────────────────────

FRAME_TYPE_SF = 0x0  # Single Frame
FRAME_TYPE_FF = 0x1  # First Frame
FRAME_TYPE_CF = 0x2  # Consecutive Frame
FRAME_TYPE_FC = 0x3  # Flow Control


@dataclass
class SingleFrame:
    data_length: int
    data: bytes

    def __repr__(self):
        return f"SingleFrame(DL={self.data_length}, DATA={self.data.hex(' ').upper()})"


def parse_isotp_frame(raw: bytes) -> Optional[SingleFrame]:
    """Parse raw 8-byte CAN data into an ISO-TP frame."""
    if not raw:
        return None
    nibble = (raw[0] >> 4) & 0x0F
    if nibble == FRAME_TYPE_SF:
        dl = raw[0] & 0x0F
        data = raw[1:1 + dl]
        return SingleFrame(data_length=dl, data=bytes(data))
    return None  # FF/CF/FC not handled in this demo


def build_single_frame(payload: bytes) -> bytes:
    """Build an ISO-TP Single Frame from a payload (max 7 bytes)."""
    assert 1 <= len(payload) <= 7, "Single Frame payload must be 1–7 bytes"
    frame = bytes([len(payload) & 0x0F]) + payload
    # Pad to 8 bytes
    frame = frame.ljust(8, b'\xCC')
    return frame


def decode_uds_service(sid: int) -> str:
    """Return a human-readable UDS service name."""
    UDS = {
        0x10: "DiagnosticSessionControl",
        0x11: "ECUReset",
        0x27: "SecurityAccess",
        0x3E: "TesterPresent",
        0x22: "ReadDataByIdentifier",
        0x2E: "WriteDataByIdentifier",
        0x14: "ClearDiagnosticInformation",
        0x19: "ReadDTCInformation",
        0x31: "RoutineControl",
    }
    return UDS.get(sid, f"Unknown(0x{sid:02X})")


# ── Main Demo ─────────────────────────────────────────────────────────────────

def run_demo(bitrate: int, tx_id: int, rx_id: int):
    print("=" * 60)
    print("  DoCAN Single Frame Demo — PCAN-USB <-> STM32H7")
    print("=" * 60)
    print(f"  Interface : PCAN-USB")
    print(f"  Bitrate   : {bitrate // 1000} kbit/s")
    print(f"  Listen ID : 0x{rx_id:03X}  (STM32H7 transmits here)")
    print(f"  Reply  ID : 0x{tx_id:03X}  (PCAN-USB transmits here)")
    print("=" * 60)

    # ── Open PCAN-USB interface ───────────────────────────────────────────────
    bus = can.interface.Bus(
        interface='pcan',
        channel='PCAN_USBBUS1',
        bitrate=bitrate,
    )
    print(f"[INFO] PCAN-USB opened at {bitrate // 1000} kbit/s\n")

    try:
        msg_count = 0
        while True:
            # ── Wait for a CAN frame from STM32H7 ────────────────────────────
            msg = bus.recv(timeout=5.0)

            if msg is None:
                print("[WARN] Timeout — No frame received in 5s. Waiting...")
                continue

            # Filter by RX ID (frames sent by STM32H7)
            if msg.arbitration_id != rx_id:
                continue

            raw = bytes(msg.data).ljust(8, b'\x00')
            msg_count += 1

            print(f"\n{'─'*55}")
            print(f"[RX #{msg_count}] t={msg.timestamp:.3f}s  ID=0x{msg.arbitration_id:03X}")
            print(f"  Raw  : {raw.hex(' ').upper()}")

            # ── Parse ISO-TP Single Frame ─────────────────────────────────────
            sf = parse_isotp_frame(raw)

            if sf is None:
                print(f"  [WARN] Not a Single Frame (byte0=0x{raw[0]:02X}) — ignored")
                continue

            print(f"  Type : Single Frame (ISO-TP)")
            print(f"  DL   : {sf.data_length} bytes")
            print(f"  Data : {sf.data.hex(' ').upper()}")

            # ── Decode UDS ────────────────────────────────────────────────────
            if len(sf.data) >= 2:
                pci_len = sf.data[0]   # UDS length byte
                sid     = sf.data[1]   # Service ID
                service = decode_uds_service(sid)
                print(f"  UDS  : SID=0x{sid:02X} ({service}), LEN={pci_len}")

                # ── Build UDS Positive Response ───────────────────────────────
                # Positive response SID = Request SID | 0x40
                pos_resp_sid = sid | 0x40
                subfunc      = sf.data[2] if len(sf.data) > 2 else 0x00

                uds_response = bytes([0x02, pos_resp_sid, subfunc])
                tx_raw       = build_single_frame(uds_response)

                print(f"\n  >> TX Positive Response:")
                print(f"     SID=0x{pos_resp_sid:02X}, Payload={tx_raw.hex(' ').upper()}")

                # ── Send response over PCAN-USB ───────────────────────────────
                reply = can.Message(
                    arbitration_id=tx_id,
                    data=tx_raw,
                    is_extended_id=False,
                )
                bus.send(reply)
                print(f"  [OK] Response sent on ID=0x{tx_id:03X}")

            else:
                print(f"  [WARN] Payload too short for UDS decode")

    except KeyboardInterrupt:
        print(f"\n\n[INFO] Demo stopped by user. {msg_count} frames processed.")
    finally:
        bus.shutdown()
        print("[INFO] PCAN-USB interface closed.")


# ── Optional: Send a Single Frame from PC to STM32H7 ─────────────────────────

def send_single_frame_to_stm32(bus, tx_id: int, payload: bytes):
    """Utility: manually send a Single Frame to the STM32H7."""
    raw = build_single_frame(payload)
    msg = can.Message(
        arbitration_id=tx_id,
        data=raw,
        is_extended_id=False,
    )
    bus.send(msg)
    print(f"[PC→STM32] SF sent on 0x{tx_id:03X}: {raw.hex(' ').upper()}")


# ── Entry Point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="PCAN-USB ISO-TP Single Frame Demo"
    )
    parser.add_argument(
        "--bitrate", type=lambda x: int(x, 0), default=500_000,
        help="CAN bitrate in bps (default: 500000)"
    )
    parser.add_argument(
        "--tx", type=lambda x: int(x, 0), default=0x7E8,
        help="ID used by PCAN-USB to transmit (default: 0x7E8)"
    )
    parser.add_argument(
        "--rx", type=lambda x: int(x, 0), default=0x7E0,
        help="ID to listen on — STM32H7 TX (default: 0x7E0)"
    )
    args = parser.parse_args()
    run_demo(args.bitrate, args.tx, args.rx)
