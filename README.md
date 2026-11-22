# iBus Sensor Bridge

A small STM32 bridge for forwarding ArduPilot iBus telemetry to a FlySky FS-iA6B receiver. It polls the flight controller, caches sensor replies, and answers the receiver from the cache to give a prompt response.

## Motivation

A direct ArduPilot iBus telemetry connection to the FS-iA6B receiver caused receiver/link problems during testing. The exact cause is still unconfirmed, but the receiver appeared to be ver sensitive to telemetry response timing. This bridge polls the flight controller and caches the latest sensor frames, then answers the receiver promptly from that cache. The goal is to decouple the receiver's request timing from delays on the flight-controller side.

## Board and wiring

This project targets the STM32F103C8Tx and was set up for a 'blue pill' board with LED on PB2.

| Blue Pill pin | MCU function | Connect to |
| --- | --- | --- |
| **PA9** | USART1, half-duplex, 115200 baud | Flight controller iBus telemetry / SENS line |
| **PA2** | USART2, half-duplex, 115200 baud | FS-iA6B receiver SENS line |
| **GND** | Ground | Flight controller and receiver ground |

PA9 and PA2 are separate one-wire connections; the bridge connects them in firmware. Configure the flight-controller UART connected to PA9 for ArduPilot iBus telemetry (SERIALx_PROTOCOL=49). Use a shared ground and signal levels compatible with 3.3V I/O.

The onboard status LED defaults to **PB2**. Its port, pin, and clock-enable macro are in the board settings block in Core/Inc/main.h; update those for a different LED pin.

## Build

Import the project into STM32CubeIDE.

The UARTs are initialized manually in Core/Src/ibus_bridge.c. This project builds from its CubeIDE metadata and source code; it does not use a CubeMX .ioc file.
