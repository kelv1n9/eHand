![Platform](https://img.shields.io/badge/platform-LGT8F328P-blue)
![License](https://img.shields.io/badge/license-MIT-green)

# 🎙️ eHand

The project provides push-to-talk audio streaming, quick on-device controls with an encoder + PTT button, and several utility modes (scanner, beacon, parrot, SOS) designed for lightweight field use.

---

## ✨ Key Features

### Voice Communication (NRF24L01+)
* **Push-to-Talk (PTT) Audio Streaming:** Live RF audio TX/RX over NRF24L01+.
* **Adjustable RF Data Rate:** `250 kbps`, `1 Mbps`, `2 Mbps` selectable from the device.
* **Adjustable RF Channel:** Cycles through predefined channels (`90`, `100`, `110`).
* **Adjustable TX Power:** `MIN / LOW / HIGH / MAX`.
* **Volume Control:** Local speaker playback volume adjustment.

### Utility Modes
* **Beacon Mode:** Periodic tone transmission with adjustable interval.
* **Parrot Mode:** Measures received audio burst duration and re-transmits it after cooldown.
* **Scanner Mode:** Scans channel + data rate combinations and detects stable incoming signals.
* **SOS Messaging:** Sends/receives a small command packet (`SOS`) and plays an alert tone when received.
* **Roger Beep:** Optional confirmation beep after RX stream ends.

### Device UX / System
* **Encoder + PTT Controls:** Single-device control without screen.
* **LED Feedback System:** Multi-pattern blink codes for battery level, mode changes, and status.
* **Battery Monitoring:** Voltage measurement via `Arduino_Vcc` with low/mid battery indication.
* **EEPROM Settings Persistence:** Saves channel, volume, data rate, TX power, and roger-beep state.
* **PTT Lock:** Lock/unlock transmit flow directly from controls.

---

## 🛠️ Hardware Components

The device is built around a few key parts:
* **MCU:** LGT8F328P
* **2.4 GHz Radio:** **NRF24L01+**
* **Audio Input:** Electret / analog microphone front-end (to `A0`)
* **Audio Output:** Speaker / amplifier driven from PWM audio pin
* **Controls:** Rotary encoder with push switch + separate PTT button
* **Indicator:** Single status LED
* **Power:** Battery-powered handheld design with voltage monitoring

---

## 🔌 Pinout 

### Main Controls / Indicators

| Function | Pin |
|---|---|
| PTT button | `D2` |
| Encoder A | `D4` |
| Encoder B | `D3` |
| Encoder switch | `D5` |
| Status LED | `D6` |
| Speaker (PWM audio out) | `D9` |
| Analog audio input | `A0` |

### NRF24L01+

| Function | Pin |
|---|---|
| CE | `D7` |
| CSN | `D8` |
| SPI | Hardware SPI pins of the board |

Notes:
* `D8` is used as `NRF24 CSN`; keep this in mind when wiring the encoder/speaker section.
* `D9` is used by audio tone/PWM output (speaker / buzzer path).

---

## 🎮 Controls

### Normal Operation

- **PTT hold**: start transmitting audio (push-to-talk)
- **PTT release**: return to receive mode (with short release debounce)
- **Encoder turn** (idle RX): adjust **volume**
- **Encoder press + turn** (idle RX): change **RF channel**
- **PTT hold + encoder turn**: change **TX power**

### Encoder Click Actions (when not transmitting)

- **1 click**: toggle **data rate** (`250k / 1M / 2M`)
- **2 clicks**: enter **Scanner** mode
- **3 clicks**: toggle **Roger Beep**
- **4 clicks**: enter **Beacon** mode
- **5 clicks**: toggle **Parrot** mode

### Extra Actions

- **PTT held + encoder click(s)**: enables **PTT lock** behavior
- **Encoder held** while in **Beacon / Scanner / Parrot**: exit that mode
- **Encoder held + PTT triple-click**: send **SOS** command packet

---

## ⚙️ How It Works

1. **Startup:** The radio is initialized in receive mode, settings are loaded from EEPROM, and LED blinks indicate battery level.
2. **Receive by Default:** The device listens for incoming NRF24 audio payloads or command packets.
3. **PTT Transmit:** Holding PTT switches to TX mode and streams microphone audio over NRF24.
4. **Local Controls:** Encoder actions adjust volume, channel, data rate, TX power, and utility modes.
5. **Utility Modes:** Scanner/Beacon/Parrot reuse the same radio/audio path with mode-specific behavior.
6. **Persistence:** Changed settings are saved to EEPROM after a short delay.

---

## 📁 Project Structure

* `src/main.cpp` — application loop, input handling, mode switching
* `src/functions.h` — feature logic, settings, modes, LED/battery/audio helpers
* `lib/DataTransfer/` — NRF24 audio streaming + command packet transport
* `lib/Tone/` — timer-free tone generation helpers
* `lib/Arduino_Vcc/` — battery voltage measurement
* `3D Model/` — enclosure/mechanical files

---

## 🚨 Disclaimer

> **For Educational & Research Use Only**
> Use this hardware and software responsibly and only on frequencies and systems you are legally allowed to operate on.
> The author accepts no liability for misuse, damage, data loss, or legal consequences.
> **All actions are at your own risk.** Always comply with local laws and regulations.
