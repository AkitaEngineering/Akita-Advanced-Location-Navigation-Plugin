# Akita Advanced Location/Navigation Plugin (AALNP)

AALNP is a Meshtastic plugin that enhances location sharing and navigation capabilities. It provides features for broadcasting location data, logging location events, and handling incoming location messages.

## Features

-   **Location Broadcast:** Periodically broadcasts location data over the Meshtastic network.
-   **Location Logging:** Logs all received location messages to a JSON file.
-   **Configurable Location Interval:** Allows users to adjust the frequency of location broadcasts via command-line arguments.
-   **Configurable Log File:** Allows users to specify the log file name via command-line arguments.
-   **Robust Error Handling:** Includes error handling for file I/O and network operations.
-   **Respects TX Delay:** The plugin will respect the TX delay of the LoRa configuration.

## Installation

1.  Place `aalnp.py` in your Meshtastic plugins directory.
2.  Run Meshtastic with the plugin enabled.

## Usage

-   Location data is automatically broadcast at the configured interval.
-   Received location data is logged in the specified log file (default: `location_log.json`).

## Command-Line Arguments

-   `--log`: Specifies the log file name (default: `location_log.json`).
-   `--interval`: Specifies the location broadcast interval in seconds (default: 60).

## Dependencies

-   Meshtastic Python API

## Akita Engineering
