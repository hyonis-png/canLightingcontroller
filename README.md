# CAN Bus Lighting Controller

**Embedded C/C++ | ESP32 | Arduino Mega | MCP2515 | SPI | CAN Bus | Python | Hardware-in-the-Loop Testing**

An embedded automotive lighting controller that uses the **Controller Area Network (CAN)** protocol to transmit lighting commands between an ESP32 and an Arduino Mega.

The project focuses on low-level embedded firmware development, SPI communication, CAN controller configuration, hardware debugging, and automated testing.

Rather than relying on a high-level CAN library, the firmware communicates directly with the MCP2515 CAN controllers through SPI register operations.

A Python-based hardware test framework was also developed to automate command transmission, verify received CAN frames, and validate the receiver's reported lighting states.

## 1. Project Overview

The objective was to develop a distributed lighting control system that could simulate communication between electronic control units (ECUs) in an automotive environment.

The system consists of two nodes:

* **ESP32 (Transmitter):** Receives commands over USB serial, constructs CAN messages, and transmits them using an MCP2515 controller.
* **Arduino Mega (Receiver):** Receives CAN messages, decodes lighting commands, and updates a finite state machine controlling LED outputs.

The implemented lighting modes include:

* Left turn signal
* Right turn signal
* Hazard lights

Headlight and brake-light commands are defined in the communication protocol but are not yet implemented in the receiver's lighting state machine.

## 2. System Architecture

```text
                  HOST COMPUTER
                        |
                   USB Serial
                        |
                        v
                 +-------------+
                 |    ESP32    |
                 | Transmitter |
                 +-------------+
                        |
                       SPI
                        |
                        v
                 +-------------+
                 |   MCP2515   |
                 | CAN TX Node |
                 +-------------+
                        |
                  CAN Transceiver
                        |
                   CANH / CANL
                        |
                  CAN Transceiver
                        |
                        v
                 +-------------+
                 |   MCP2515   |
                 | CAN RX Node |
                 +-------------+
                        |
                       SPI
                        |
                        v
                 +-------------+
                 | Arduino Mega|
                 |  Receiver   |
                 +-------------+
                        |
                  Lighting FSM
                        |
                        v
                   LED Outputs
```

Each MCP2515 module includes a physical CAN transceiver interface.

The microcontrollers communicate with their respective MCP2515 controllers using SPI, while the two CAN nodes communicate over CANH and CANL.

## 3. Hardware

### Components

| Component            | Function                                    |
| -------------------- | ------------------------------------------- |
| ESP32                | CAN command transmitter                     |
| Arduino Mega 2560    | CAN receiver and lighting controller        |
| MCP2515 modules (×2) | CAN controllers with transceiver interfaces |
| LEDs                 | Simulated automotive lighting               |
| Breadboard           | Circuit prototyping                         |
| Jumper wires         | Electrical connections                      |

### ESP32 SPI Pin Configuration

| SPI Signal | ESP32 GPIO |
| ---------- | ---------- |
| CS         | GPIO 5     |
| SCK        | GPIO 18    |
| MOSI       | GPIO 19    |
| MISO       | GPIO 21    |

### Arduino Mega SPI Pin Configuration

| SPI Signal | Arduino Mega Pin |
| ---------- | ---------------- |
| CS         | 53               |
| SCK        | 52               |
| MOSI       | 51               |
| MISO       | 50               |

Both MCP2515 modules use 8 MHz oscillators.

## 4. Firmware Implementation

### 4.1 Low-Level SPI Communication

The ESP32 firmware uses the Arduino SPI library to communicate with the MCP2515.

Custom functions were implemented for:

* SPI chip-select control
* MCP2515 reset
* Register reads
* Register writes
* Register bit modification

For example, the following function writes directly to an MCP2515 register:

```cpp
void MCP2515_write(uint8_t address, uint8_t data)
{
    cs_low();

    spi_transfer(0x02);
    spi_transfer(address);
    spi_transfer(data);

    cs_high();
}
```

The `0x02` instruction tells the MCP2515 to perform a register write.

The following bytes specify the register address and the data to be written.

This approach provided direct control over the CAN controller and made it possible to inspect internal registers during hardware debugging.

### 4.2 MCP2515 Initialization

The firmware initializes the MCP2515 by:

1. Issuing a hardware reset instruction.
2. Verifying that the controller enters configuration mode.
3. Configuring the CAN bit-timing registers.
4. Switching the controller into normal operating mode.
5. Reading back registers to verify the configuration.

The main configuration registers are:

| Register | Purpose                                               |
| -------- | ----------------------------------------------------- |
| CANCTRL  | Controls the MCP2515 operating mode                   |
| CANSTAT  | Reports the current operating mode                    |
| CNF1     | Baud-rate prescaler and synchronization configuration |
| CNF2     | Propagation and phase-segment timing                  |
| CNF3     | Additional phase-segment configuration                |
| CANINTF  | CAN interrupt flags                                   |
| EFLG     | CAN controller error flags                            |
| TEC      | Transmit error counter                                |
| REC      | Receive error counter                                 |

The final tested configuration used:

```cpp
MCP2515_write(0x2A, 0x03);  // CNF1
MCP2515_write(0x29, 0x90);  // CNF2
MCP2515_write(0x28, 0x02);  // CNF3
```

This reduced-speed configuration was used during successful communication testing.

**Note:** The project presentation describes the configuration as 125 kbit/s. However, the listed register values correspond to approximately 111.1 kbit/s for an 8 MHz MCP2515 oscillator. The nominal bit rate should be verified before further development.

### 4.3 CAN Frame Transmission

Lighting commands are transmitted using standard 11-bit CAN frames.

| Parameter        | Value                        |
| ---------------- | ---------------------------- |
| CAN Identifier   | `0x7FF`                      |
| Frame Format     | Standard 11-bit              |
| Data Length Code | 1                            |
| Payload          | Single-byte lighting command |

The transmitter loads the identifier and command into the MCP2515 transmit buffer.

```cpp
MCP2515_write(0x31, id >> 3);
MCP2515_write(0x32, (id & 0x07) << 5);

MCP2515_write(0x35, 1);
MCP2515_write(0x36, data);
```

Before requesting transmission, the firmware reads back the transmit-buffer registers to verify that the expected identifier, data length, and payload were loaded correctly.

Transmission is initiated using the MCP2515 Request-to-Send instruction:

```cpp
cs_low();
spi_transfer(0x81);
cs_high();
```

The firmware then monitors the transmit-buffer status and controller error registers.

### 4.4 CAN Command Protocol

All commands use CAN identifier `0x7FF`.

| Command        | Payload | Function                         |
| -------------- | ------- | -------------------------------- |
| LEFT_ON        | 9       | Enable left turn signal          |
| LEFT_OFF       | 10      | Disable left turn signal         |
| RIGHT_ON       | 11      | Enable right turn signal         |
| RIGHT_OFF      | 12      | Disable right turn signal        |
| HEADLIGHTS_ON  | 13      | Reserved for headlight control   |
| HEADLIGHTS_OFF | 14      | Reserved for headlight control   |
| HAZARDS_ON     | 15      | Enable hazard lights             |
| HAZARDS_OFF    | 16      | Disable hazard lights            |
| BRAKES_ON      | 17      | Reserved for brake-light control |
| BRAKES_OFF     | 18      | Reserved for brake-light control |

The receiver checks incoming CAN identifiers, data lengths, and payloads before processing lighting commands.

## 5. Lighting Finite State Machine

The Arduino Mega implements a finite state machine to manage lighting behavior.

The supported states are:

```text
ALL_OFF
LEFT_ON
RIGHT_ON
HAZARD_ON
```

Incoming CAN commands determine state transitions.

For example:

```text
                  LEFT_ON
        +------------------------+
        |                        v
    +---------+             +---------+
    | ALL_OFF |             | LEFT_ON |
    +---------+             +---------+
        ^                        |
        |                        |
        +------------------------+
                  LEFT_OFF
```

The receiver also supports switching between lighting modes.

An OFF command only disables its corresponding active mode, preventing unrelated commands from unintentionally changing the current lighting state.

### Non-Blocking LED Control

The Arduino Mega uses `millis()`-based timing to control LED blinking.

This allows the firmware to update the lighting outputs while continuing to process incoming CAN messages.

## 6. Hardware Bring-Up and Debugging

One of the main challenges was establishing reliable communication between the two MCP2515 CAN nodes.

Initial testing produced unsuccessful transmissions and missing receiver responses.

Rather than modifying the entire application, debugging was divided into separate stages.

### Step 1 — Verify SPI Communication

The first step was confirming that each microcontroller could communicate with its MCP2515.

This was accomplished by:

* Resetting the MCP2515.
* Reading the CANSTAT and CANCTRL registers.
* Writing configuration values.
* Reading the registers back.
* Comparing the returned values against expectations.

This helped establish that the microcontroller-to-CAN-controller SPI interfaces were functioning.

### Step 2 — Verify CAN Frame Construction

The next step was verifying that the ESP32 correctly loaded the CAN transmit buffer.

The firmware read back:

* CAN identifier registers
* Data Length Code
* Command payload
* Transmit-buffer control register

This helped distinguish frame-construction problems from CAN bus communication problems.

### Step 3 — Investigate CAN Communication Failures

After verifying SPI communication and transmit-buffer contents, attention shifted to the CAN bus.

Potential issues included:

* Incorrect bit timing
* CANH/CANL wiring
* Loose terminal connections
* Physical-layer signal integrity

Higher-speed communication was unreliable.

The CANH/CANL wires were also found to be insufficiently secured in the MCP2515 screw terminals.

This was considered a possible contributing factor, although the root cause was not conclusively established.

### Step 4 — Adjust Configuration and Retest

The CAN timing configuration was adjusted to operate at a lower bit rate.

After reconfiguration, the receiver successfully reported the expected CAN frames.

The automated test framework was then used to validate repeated command sequences.

## 7. Automated Hardware-in-the-Loop Testing

A Python script, `test_esp32.py`, was developed to automate communication testing using PySerial.

The script communicates with both microcontrollers through their USB serial interfaces.

### Test Architecture

```text
                  Python Test Script
                         |
                +--------+--------+
                |                 |
             COM6               COM5
                |                 |
                v                 v
              ESP32          Arduino Mega
                |                 ^
                v                 |
             MCP2515          MCP2515
                |                 ^
                +---- CAN BUS ----+
```

The Python script sends commands to the ESP32 and collects diagnostic output from both devices.

### Test Procedure

1. Open serial connections to the ESP32 and Arduino Mega.
2. Initialize the lighting controller to a known state.
3. Send a predefined sequence of lighting commands.
4. Collect transmission and reception logs.
5. Parse the received CAN frames.
6. Compare the received identifier, DLC, and payload against expected values.
7. Predict the expected lighting state.
8. Compare the predicted state against the Arduino Mega's reported state.
9. Inspect CAN error flags and counters.
10. Save the test results to a log file.

### Pass Criteria

A test passes when:

* The received CAN identifier equals `0x7FF`.
* The DLC equals 1.
* The payload matches the transmitted command.
* The transmitter reports `TX_COMPLETE`.
* The receiver reports the expected lighting state.

At the end of the test sequence, the script also checks that both controllers report zero values for the monitored CAN error flags and counters.

### Example Test Output

```text
ESP TX_COMPLETE

MEGA RX CAN ID: 0x7FF DLC: 1 DATA: 15
MEGA STATE: HAZARD_ON

TEST 1: PASS
command=1
frame=(2047, 1, ['15'])
state=HAZARD_ON
```

The test script repeats a predefined command sequence across multiple rounds and stops when a failure is detected.

**Testing limitation:** This framework verifies communication and the lighting states reported by the receiver. It does not independently measure the physical LED outputs.

## 8. Running the Hardware Tests

### Requirements

* Python 3
* PySerial
* ESP32 with transmitter firmware
* Arduino Mega with receiver firmware
* Two MCP2515 CAN modules
* Connected CAN bus

Install PySerial:

```bash
pip install pyserial
```

Connect both microcontrollers to the computer and close any open Arduino Serial Monitor sessions.

Run the automated test:

```bash
python test_esp32.py
```

The default configuration uses:

| Parameter                | Default        |
| ------------------------ | -------------- |
| ESP32 serial port        | COM6           |
| Arduino Mega serial port | COM5           |
| Test rounds              | 5              |
| Log file                 | `can_test.log` |

Custom settings can be provided:

```bash
python test_esp32.py --esp COM6 --mega COM5 --rounds 10
```

The test results are printed to the terminal and saved to the specified log file.

## 9. Results

The project demonstrated successful communication between an ESP32 and Arduino Mega through MCP2515 CAN controllers using the reduced-speed configuration.

The validation process demonstrated:

* Successful SPI register communication with both CAN controllers.
* Correct CAN identifier and payload loading.
* Successful CAN frame reception.
* Correct reported lighting state transitions.
* Automated testing of repeated command sequences.
* Zero CAN error flags and counters in the passing test run.

The results demonstrate working CAN communication and reported lighting-state behavior under the tested conditions.

They do not establish reliable operation across all bus speeds, physical configurations, or failure scenarios.

## 10. Technical Challenges and Lessons Learned

### Register-Level Debugging

Working directly with MCP2515 registers provided greater visibility into the controller's internal state.

Reading back configuration registers and transmit-buffer contents helped identify which parts of the system were functioning correctly.

### Hardware and Software Integration

The project demonstrated the importance of validating individual communication layers before testing an entire embedded system.

Successful SPI communication did not automatically guarantee successful CAN communication.

### Automated Testing

Developing a Python test framework made it possible to execute repeatable command sequences and compare actual receiver responses against expected behavior.

This reduced reliance on manually inspecting serial output for every command.

### Physical-Layer Reliability

The project also highlighted the importance of physical connections and CAN bit timing.

Even when firmware is configured correctly, unreliable wiring or incompatible timing can prevent communication.

## 11. Future Improvements

* Implement headlight and brake-light functionality.
* Verify the exact CAN bit rate and timing parameters.
* Improve CAN bus wiring and termination.
* Add automated physical LED-output verification.
* Implement CAN communication fault recovery.
* Add fault injection and negative testing.
* Test communication reliability at higher bit rates.
* Refactor the MCP2515 register operations into a reusable driver.
* Add sensor-driven automatic lighting functionality.

## 12. Technologies and Skills

| Category                | Technologies / Concepts                                |
| ----------------------- | ------------------------------------------------------ |
| Embedded Programming    | C/C++, Arduino framework                               |
| Microcontrollers        | ESP32, Arduino Mega 2560                               |
| Communication Protocols | SPI, CAN, UART/USB Serial                              |
| CAN Controller          | MCP2515                                                |
| Firmware Design         | Register-level programming, finite state machines      |
| Hardware Debugging      | Register readback, CAN error diagnostics, bus bring-up |
| Automated Testing       | Python, PySerial                                       |
| Verification            | Hardware-in-the-loop testing, regression testing       |
| Development Tools       | Arduino IDE, Python, serial diagnostics                |

## Project Status

**Functional prototype — CAN communication and reported lighting-state verification completed.**

The ESP32 and Arduino Mega successfully exchanged CAN lighting commands under the tested configuration.

Automated tests verified received frame contents and reported state transitions.

Further work is required to validate physical lighting outputs, improve communication robustness, and implement the remaining lighting features.

---

## Author

**Heysome Yonis**<img width="1218" height="696" alt="image" src="https://github.com/user-attachments/assets/711ecad6-db73-4b75-b5ca-303fc299be72" /> <img width="1137" height="646" alt="image" src="https://github.com/user-attachments/assets/03b78cbc-c0e1-416b-90d4-4742f1c37568" />

<img width="1120" height="422" alt="image" src="https://github.com/user-attachments/assets/df5cf05e-70f4-4a03-8cc2-e85ff533ca1b" />


Embedded Systems | Digital Hardware | Firmware Development
