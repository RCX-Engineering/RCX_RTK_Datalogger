# RCX RTK Datalogger & RCX RTK Base

The **RCX RTK Datalogger**, developed by **RCX Engineering**, is an open-source RTK GNSS data-acquisition system developed for motorsports. The project combines a 20 Hz quad-band omni-constellation GNSS receiver, 6-axis IMU, vehicle CAN data, local SD logging, an onboard display, Wi-Fi connectivity for RTCM (RTK) data, a web interface for device management, and Bluetooth (BLE) telemetry compatible with the RaceCapture protocol for connectivity to datalogging software like SoloStorm.

The repository contains firmware for two closely related devices:

- **RCX RTK Datalogger (rover)** — installed in the vehicle to acquire GNSS, IMU, and CAN data and stream/log the resulting telemetry.
- **RCX RTK Base** — a portable reference station that generates RTK correction data for use where a suitable nearby public base is unavailable.

The rover and base share identical hardware. Their primary difference is the firmware and the interfaces enabled for each role.

> **Project status:** The RCX RTK Datalogger and RCX RTK Base are actively developed engineering/prototype platforms. They have been tested in real autocross environments, but hardware, firmware, configuration, and data-processing methods continue to evolve. 

## What Is RTK GNSS?

**GNSS** (Global Navigation Satellite System) is the general term for satellite-positioning systems such as GPS, Galileo, GLONASS, and BeiDou. A conventional standalone GNSS receiver calculates its position from the timing and ranging information broadcast by the satellites. Even with a good modern multi-band receiver, the resulting position still contains errors from satellite clocks and orbits, atmospheric propagation, multipath, receiver noise, and satellite geometry.

**RTK — Real-Time Kinematic — GNSS adds a fixed reference base station at a known location.** The base observes many of the same satellites and many of the same error sources as the moving rover. It generates correction data and sends it to the rover, typically as an RTCM stream over a radio or an Internet-based NTRIP connection. The rover combines those corrections with multi-frequency carrier-phase measurements and resolves the carrier-cycle ambiguities required for an **RTK Fixed** solution.

The important difference is scale: ordinary standalone GNSS positioning is generally a **meter-scale** measurement, while a good RTK Fixed solution can provide **centimeter-scale relative positioning**. The exact accuracy is installation and environment-dependent, and absolute accuracy also depends on how accurately the base position is known.

| | Standalone GNSS | RTK GNSS |
|---|---|---|
| Reference station | Not required | Required, directly or through an RTK network |
| Correction data | None | Real-time base/network corrections |
| Primary measurements | Satellite ranging; receiver-dependent use of carrier data | Multi-frequency carrier-phase positioning with corrections |
| Typical positioning scale | Meter level | Centimeter level when RTK Fixed |
| Sensitivity to multipath/obstruction | Significant | Still significant; RTK cannot eliminate local multipath or blocked satellites |
| Best suited for | Navigation, timing, general track mapping | Precise trajectory comparison, surveying, machine control, and high-resolution motorsports analysis |

For motorsports, the benefit is not simply a prettier GPS trace. When two runs differ by only a few inches at an apex or slalom cone, meter-scale position uncertainty can be larger than the driving difference being studied. Centimeter-class RTK data makes those differences directly measurable and substantially improves confidence when correlating vehicle position with IMU and CAN data.

RTK also improves **repeatability**: data from different runs, vehicles, or days can be placed in the same coordinate frame when they use a properly established reference. This enables meaningful comparison of line choice, cone clearance, braking and acceleration locations, vehicle attitude, and derived quantities such as wheel trajectories and slip behavior.

RTK does **not** inherently mean a high update rate. Position accuracy and sample rate are separate characteristics. The RCX RTK Datalogger combines RTK positioning with **20 Hz** GNSS output specifically so that centimeter-scale position information is also sampled fast enough for highly transient autocross maneuvers.

## Why RTK Datalogging?

Conventional consumer GNSS logging is often adequate for lap timing and broad vehicle-path visualization, but its positional uncertainty can be large relative to the differences that matter in autocross: inches of line placement, distance from a cone, small changes in corner entry, and vehicle attitude during transient maneuvers.

The RCX RTK Datalogger was developed to combine high-rate RTK positioning with vehicle and inertial data in a single inexpensive, reproducible platform. The resulting data supports applications such as:

- high-resolution racing-line comparison;
- cone-relative path and clearance analysis;
- vehicle orientation and yaw analysis;
- oversteer/understeer and slip-angle analysis;
- correlation of GNSS, IMU, and CAN data;
- course reconstruction and 3D visualization;
- development of higher-fidelity empirical vehicle models.

## System Architecture

```text
                         ┌──────────────────────┐
                         │     RCX RTK Base     │
                         │                      │
                         │ ESP32-S3 + LG290P    │
                         │ survey / reference   │
                         └──────────┬───────────┘
                                    │ RTK correction stream
                                    ▼
                         ┌──────────────────────┐
                         │     NTRIP caster     │
                         │  Centipede / RTK2go  │
                         └──────────┬───────────┘
                                    │ Wi-Fi / Internet
                                    ▼
┌──────────────┐         ┌──────────────────────────────┐
│ Vehicle CAN  ├────────►│   RCX RTK Datalogger         │
└──────────────┘         │                              │
                         │ ESP32-S3-LCD-1.47B           │
                         │ LG290P RTK GNSS @ 20 Hz      │
                         │ onboard QMI8658 6-axis IMU   │
                         │ TWAI / CAN interface         │
                         │ SD logging                   │
                         │ LCD + web UI                 │
                         └─────────┬───────────┬────────┘
                                   │           │
                          Bluetooth│           │ local files/UI
                                   ▼           ▼
                            ┌────────────┐   SD / browser
                            │ SoloStorm  │
                            │RaceCapture │
                            │ protocol   │
                            └────────────┘
```

### RCX RTK Datalogger

The rover firmware is designed for motorsports data logging and currently integrates:

- **20 Hz RTK GNSS** using the Waveshare/Quectel LG290P;
- **6-axis inertial data** from the QMI8658 IMU;
- **vehicle CAN acquisition** through the ESP32-S3 TWAI peripheral and an external CAN transceiver;
- **DBC-based CAN decoding**;
- **RaceCapture-compatible Bluetooth telemetry** for SoloStorm;
- **MicroSD logging** for local high-rate data capture;
- **on-device status display**;
- **embedded web interface** for status and configuration;
- **Wi-Fi/NTRIP correction handling**;
- watchdog and resource-management logic for an aggressively loaded ESP32-S3 platform.

Current vehicle-specific CAN work has primarily been developed and validated around Porsche 987.2 S PDK and 718 PDK platforms. DBC support is intended to make the CAN layer extensible to other vehicles and signal definitions.

### RCX RTK Base

The RCX RTK Base firmware uses the same general ESP32-S3/LG290P hardware platform as the RCX RTK Datalogger but configures it as a stationary RTK reference source.

The base has been developed to:

- establish a reference position for the GNSS receiver;
- use PPP/Galileo E6-HAS information as part of the custom survey-in workflow;
- generate RTK correction data;
- publish corrections through NTRIP casters;
- provide local status information and logging for base health and stability.

Development testing has used both **Centipede** and **RTK2go** NTRIP services. A single correctly configured base can provide correction data to multiple compatible rovers within the practical RTK baseline and communications limits.

## Measured Performance

The RCX RTK Datalogger has produced centimeter-class dynamic GNSS solution quality in motorsports testing, with substantially tighter stationary solution scatter under favorable conditions.

Representative results reported during development include:

| Test | Reported result |
|---|---:|
| 30-minute stationary test | **1.6 mm 1σ positional scatter** |
| Bristol National Tour, on course | **~1.9 cm mean reported accuracy** |
| NCCAR autocross, on course | **1.6 cm median reported accuracy** |
| NCCAR autocross, on course | **2.4 cm 95th-percentile reported accuracy** |

These values are representative test results, not guaranteed performance. GNSS performance remains dependent on satellite geometry, correction quality, antenna installation, multipath, obstructions, RF environment, RTK fix state, and base-station quality. The reported solution metrics also should not be interpreted as an independent surveyed ground-truth error measurement.

## Hardware

The reference implementation is based on the following hardware.

### Core hardware

- **Waveshare ESP32-S3-LCD-1.47B**
  - ESP32-S3
  - onboard QMI8658 IMU
  - onboard LCD
  - PSRAM required by the current firmware
- **Waveshare LG290P RTK GNSS development board**
  - LG290P firmware **v2.02 or newer** required
  - tested package: `LG290P03AANR02A02S.pkg`
- multi-band GNSS antenna
- MicroSD card
- USB power

### RCX RTK Datalogger-specific hardware

- **SN65HVD230 CAN transceiver board** or equivalent compatible 3.3 V CAN interface
- vehicle CAN harness/tap
- appropriate termination configuration for the vehicle network

The reference SN65HVD230 development board is used with its onboard **120 Ω termination resistor removed** when attaching to an already terminated vehicle CAN bus.

### Recommended supporting hardware

- GNSS RTC backup battery for faster hot starts;
- small cooling fan for sustained high-load operation;
- 3D-printed enclosure and mounting hardware;
- suitable multi-band antenna with good multipath rejection.

A helical antenna has performed better than the original patch antenna in some real autocross environments with substantial multipath and dynamic obstruction, although antenna choice and mounting should be treated as installation-specific engineering decisions.

## Firmware Build Environment

The firmware is built with the **Arduino IDE**.

The rover build has been validated with:

- **ESP32 by Espressif Systems 3.3.8**
- board target: **ESP32S3 Dev Module**
- 16 MB flash
- QIO 80 MHz flash mode
- OPI PSRAM
- 240 MHz CPU
- `Huge APP (3MB No OTA/1MB SPIFFS)` partition scheme
- USB CDC on boot enabled

The current rover firmware depends on the ESP32 Arduino Core 3.x APIs and memory layout; earlier core releases are not supported.

### RCX RTK Datalogger libraries

The rover currently uses:

- `TFT_eSPI` — Bodmer
- `NimBLE-Arduino` 2.x — h2zero
- `TinyGPSPlus` — Mikal Hart
- `ESP Async WebServer` — ESP32Async
- `AsyncTCP` — ESP32Async

Wi-Fi, Wire/I2C, SD_MMC, Preferences, TWAI, and watchdog support are provided by the ESP32 Arduino core. The QMI8658 is driven directly over I2C.

## Engineering Knowledge Graphs Are Source Code

This repository also contains **OpenCypher knowledge-graph models developed by RCX Engineering for the RCX RTK Datalogger and RCX RTK Base wiring and system architecture**.

These graph models are considered part of the project source code, not merely supplemental documentation. They provide a machine-readable representation of the engineered system, including relationships among hardware components, interfaces, wiring, buses, data paths, and architectural elements.

The intent is for the knowledge graph to serve alongside the firmware as an authoritative engineering artifact that can support:

- architecture queries and design review;
- wiring/interface traceability;
- consistency checking between hardware and software;
- automated engineering analysis;
- impact analysis when components or interfaces change;
- future AI/agent-assisted engineering workflows.

Changes to the physical or logical architecture should therefore include corresponding updates to the OpenCypher model where applicable.

## Data and Interfaces

### GNSS

The rover is designed around 20 Hz LG290P position output and RTK corrections delivered over NTRIP. RTK fixed operation is the normal target state for high-precision motorsports analysis.

### IMU

The onboard QMI8658 provides six-axis inertial measurements. IMU data are logged alongside GNSS and CAN so that vehicle motion can be analyzed in a common time history.

### CAN

Vehicle CAN is acquired through the ESP32-S3 TWAI controller. DBC decoding is supported so that raw CAN traffic can be converted into engineering channels without hard-coding every signal into the application.

Connecting experimental hardware to a vehicle CAN network requires care: incorrect wiring, termination, shorts, or transceiver faults can disrupt vehicle communications.

Porsche 987.2 and 718 were used for development and their CAN addresses are hardcoded and automatically recognized. Other vehicles are supported by uploading a .dbc file with vehicle specific CAN data.

### Bluetooth / SoloStorm

The rover implements the published **RaceCapture** communication format over Bluetooth so that supported telemetry can be consumed by SoloStorm while the full data set is also retained locally.

### Local Logging

MicroSD logging provides a source independent of the live Bluetooth connection and is intended for engineering analysis, debugging, and preservation of higher-detail data.

## Base-Station and RTK Considerations

RTK quality depends on both ends of the system. A high-quality rover cannot compensate for a poor reference position, interrupted RTCM stream, excessive base-to-rover separation, or severe local multipath.

Development of the RCX RTK Base has therefore focused on both position establishment and operational stability. PPP-assisted survey-in was added to improve the quality of a temporary base position when a surveyed reference point is not available.

For repeat events, a previously established high-quality reference position is generally preferable to treating every setup as an unrelated new base location.

## Known Limitations / Active Development Areas

The RCX RTK Datalogger and RCX RTK Base have been deliberately pushed close to the resource and thermal limits of the ESP32-S3. Areas that have required active development include:

- thermal management under sustained load;
- task scheduling and prioritization;
- simultaneous base casting and SD logging;
- GNSS multipath and dynamic occlusion;
- transitions between RTK fixed and RTK float;
- antenna placement and phase-center effects;
- reliability of external NTRIP services;
- vehicle-specific CAN decoding;
- dead-reckoning and GNSS-outage handling.

These are important engineering characteristics of the project, not hidden failure modes. Contributions that improve robustness, portability, diagnostics, or validation are welcome.

## Building and Configuration

1. Install the Arduino IDE.
2. Install **ESP32 by Espressif Systems 3.3.8**.
3. Select **ESP32S3 Dev Module** and configure flash/PSRAM/partition settings as documented in the firmware-specific README.
4. Install the required libraries.
5. Build and flash the appropriate **rover** or **base-station** firmware.
6. Connect to the wifi SSID shown on the LCD 
7. Browse to the IP address shown on the LCD, http://192.168.4.1 or http://192.168.5.1 and add the desired wifi SSIDs annd passwords. *Note: Use http, not https; encryption is not yet implemented.*
8. If a private base is used, configure the base and datalogger for the same NTRIPS service and host name.
9. Configure the tablet datalogger to connect to your new datalogger via RaceCapture format.

## Contributing

Issues, test data, DBC definitions, hardware-porting improvements, GNSS/base-station validation, documentation fixes, and pull requests are welcome.

When modifying the system architecture, please treat the firmware, wiring definitions, and OpenCypher knowledge graph as coordinated source artifacts and update all affected representations together.

## Project History

The RCX RTK Datalogger and RCX RTK Base grew from a DIY motorsports data-logging experiment into a combined rover/base RTK platform through iterative static testing, autocross competition use, CAN reverse engineering, GNSS/antenna testing, and firmware optimization.

The development discussion, test results, and design evolution are documented in the RoadRaceAutoX thread:

**[DIY RTK GPS Datalogger: “Holy crap how is that possible??” level accuracy](https://www.roadraceautox.com/forum/general-discussion/fabrication-design/4178587-diy-rtk-gps-datalogger-holy-crap-how-is-that-possible-level-accuracy)**

## Acknowledgments

The RCX RTK Datalogger and RCX RTK Base build on open protocols, open-source libraries, community reverse engineering, and the work of the developers who make the ESP32, GNSS, Bluetooth, web-server, and motorsports data ecosystems accessible to hobbyist engineering projects.

In particular, the project uses the published **RaceCapture protocol from Autosport Labs** for motorsports telemetry interoperability.

## License

See [LICENSE](LICENSE) for the terms that apply to this repository.
