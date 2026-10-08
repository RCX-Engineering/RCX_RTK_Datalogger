# RCX RTK Datalogger Schematics & Knowledge Graph

This directory contains the interactive wiring and architecture model for the **RCX RTK Datalogger**, developed by **RCX Engineering**.

These files are not conventional static schematics. Each HTML file contains a self-contained **OpenCypher labeled property graph** describing the physical and logical wiring and firmware architectures, together with JavaScript that renders the graph into interactive engineering views. These are interactive viewers designed to aid design and troubleshooting and are historic as the the first published generative model-based engineering knowledge graph for electrical design using new methods proposeed by NASA and shared with NASA's Text-to-Spaceship team.

## Interactive Viewers

| File | Purpose |
|---|---|
| [`RCX_RTK_Datalogger-schematic_viewer.html`](https://rcx-engineering.github.io/RCX_RTK_Datalogger/schematics/RCX_RTK_Datalogger-schematic_viewer.html) | Engineering model viewer for the hardware and wiring configuration. |
| [`RCX_RTK_Datalogger - Firmware Schematic Viewer.html`](https://rcx-engineering.github.io/RCX_RTK_Datalogger/schematics/RCX_RTK_Datalogger%20-%20Firmware%20Schematic%20Viewer.html) | Engineering model viewer for the firmware architecture. |

## Knowledge Graph Model

The OpenCypher models in these files are considered part of the **RCX RTK Datalogger source code**, not merely documentation. The full graph models are embedded in the html viewers for convenience; both model and viewer must be updated simultanously.

The graphs are intended to be a machine-readable engineering definition of the system architecture. They captures relationships among devices, connectors, pins, harnesses, electrical nets, functions, and physical wiring in a form that can be queried, validated, transformed, or used to generate other engineering artifacts.

---

**RCX Engineering**  
RCX RTK Datalogger
