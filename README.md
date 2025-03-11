# Akita Advanced Location/Navigation Plugin (AALNP)

AALNP is a Meshtastic plugin that enhances location sharing and navigation capabilities. It provides features for broadcasting location data, logging location events, and handling incoming location messages.

## Features

* **Location Broadcast:** Periodically broadcasts location data over the Meshtastic network.
* **Location Logging:** Logs all received location messages to a JSON file.
* **Configurable Location Interval:** Allows users to adjust the frequency of location broadcasts via command-line arguments.
* **Configurable Log File:** Allows users to specify the log file name via command-line arguments.
* **Robust Error Handling:** Includes error handling for file I/O, network operations, and GPS data validation.
* **Respects TX Delay:** The plugin respects the TX delay of the LoRa configuration using a message queue.
* **Graceful Shutdown:** Handles keyboard interrupts for clean plugin termination.
* **Message Queueing:** Uses a message queue to buffer location data before transmission.
* **GPS Data Validation:** Validates GPS data before broadcasting.

## Installation

1.  Place `aalnp.py` in your Meshtastic plugins directory.
2.  Install the Meshtastic Python API if not already installed.
3.  Run Meshtastic with the plugin enabled.

## Usage

* Location data is automatically broadcast at the configured interval.
* Received location data is logged in the specified log file (default: `location_log.json`).
* Use Ctrl+C to stop the plugin gracefully.

## Command-Line Arguments

* `--log`: Specifies the log file name (default: `location_log.json`).
* `--interval`: Specifies the location broadcast interval in seconds (default: 60).

## Dependencies

* Meshtastic Python API

## Configuration

The plugin is configured via command-line arguments.

## Logging

The plugin uses the Python `logging` module for detailed logging.

## Location Log File

The location log file is a JSON file that stores received location data. If the file does not exist, it will be created. If the file is malformed, it will be reset to an empty array.

## Akita Engineering

This project is developed and maintained by Akita Engineering. We are dedicated to creating innovative solutions for LoRa and Meshtastic networks.
