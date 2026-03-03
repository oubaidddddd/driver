# DoCAN Driver — ISO 15765-2 en Rust embarqué (STM32H7)

## Description

Projet de fin d'études visant à développer un driver **DoCAN (ISO 15765-2)** performant pour le protocole de diagnostic automobile **UDS**, implanté en **Rust embarqué** sur microcontrôleur **STM32H743**.

## Architecture du projet

```
docan_driver/
├── Cargo.toml
├── memory.x                  # Linker script STM32H743
├── .cargo/
│   └── config.toml           # Cible thumbv7em-none-eabihf
└── src/
    ├── main.rs               # Point d'entrée (no_std / no_main)
    ├── frame.rs              # ISO 15765-2 CAN Frames
    ├── transport.rs          # Couche Transport (TransportLayer, TxState, RxState)
    └── utils.rs              # Utilitaires (Checksum, Validator, Timeout, CRC)
```

## Modules

### `frame.rs` — ISO 15765-2 CAN Frames
Définit les 4 types de trames :
| Type | Description |
|------|-------------|
| `SingleFrame` (SF) | Données ≤ 7 octets |
| `FirstFrame` (FF) | Première trame (message 8–4095 octets) |
| `ConsecutiveFrame` (CF) | Trames suivantes avec numéro de séquence (0–15) |
| `FlowControlFrame` (FC) | Contrôle de flux (BS, STmin) |

### `transport.rs` — Couche de transport
- **`TransportLayer`** : orchestration émission/réception
- **`TxState`** : machine d'état émetteur (Idle / Sending / AwaitingFlowControl)
- **`RxState`** : machine d'état récepteur (Idle / Receiving)
- **`Config`** : TX ID, RX ID, FlowControlConfig, timeout

### `utils.rs` — Utilitaires
- **`ChecksumCalculator`** : sum, xor, fletcher
- **`MessageValidator`** : validation longueur et segmentation
- **`TimeoutConfig`** : paramètres de timeout ISO (automotive / realtime)
- **`TimeoutCounter`** : suivi du temps réel (N_As, N_Bs, N_Cs, N_Ar, N_Br, N_Cr)
- **`Crc32Calculator`** + **`CrcConfig`** : CRC-32/16

## Séquence ISO-TP Multi-Frame

```
Sender                    CAN Bus                   Receiver
  |                          |                          |
  |--- FirstFrame (FF) ----->|-----> handle_rx_frame -->|
  |                          |<----- FlowControl (FC) --|
  |<-- handle_flow_control --|                          |
  |      loop:               |                          |
  |--- ConsecutiveFrame ---->|-----> append_data ------>|
  |         STmin wait       |                          |
  |--- ConsecutiveFrame ---->|---- Full Data ready ---->|
```

## Compilation

```bash
# Installer la cible ARM Cortex-M7
rustup target add thumbv7em-none-eabihf

# Compiler
cargo build --release

# Flasher (avec probe-run)
cargo run --release
```

## Conformité ISO 15765-2

| Timer | Rôle | Valeur (automotive) |
|-------|------|---------------------|
| N_As  | Timeout émission SF/FF/CF | 25 ms |
| N_Bs  | Timeout attente FC | 250 ms |
| N_Cs  | Séparation entre CF | STmin (10 ms) |
| N_Ar  | Timeout émission FC | 25 ms |
| N_Br  | Timeout émission FC après FF | 250 ms |
| N_Cr  | Timeout entre deux CF consécutifs | 1000 ms |

## Licence

Projet académique — ACTIA 2024
# driver
