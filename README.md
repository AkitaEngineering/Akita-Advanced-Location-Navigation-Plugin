# Akita Advanced Location/Navigation Plugin (AALNP) v2.1

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
**Repository:** AkitaEngineering/Akita-Advanced-Location-Navigation-Plugin

AALNP is a significantly enhanced Meshtastic plugin developed by Akita Engineering, focused on providing advanced, practical location sharing, tracking, and simple navigation features over LoRa. Version 2.1 adds interactive commands, display output, geo-fencing, and local track storage.

Version 2.1 also supports an optional local desktop console for live monitoring and configuration. The GUI uses a black, grey, silver, titanium, white, and green palette and can show local status, an activity map, nearby nodes, selected-node telemetry, navigation info, geofence state, display preview text, recent GeoJSON activity, and direct location-request controls. The activity map is interactive, so clicking a plotted nearby node focuses that node in the detail panel.

## Overview

Meshtastic is great for off-grid communication, but basic location sharing can be improved. The Akita Advanced Location/Navigation Plugin (AALNP) v2.1 transforms your Meshtastic node into a smarter location-aware device. It intelligently broadcasts its location, responds to direct requests, helps you navigate towards waypoints, logs detailed track data, alerts you to nearby teammates, reacts to text commands, shows status on device displays, warns on entering/exiting defined zones, and keeps a short history of received locations, all while respecting the low-bandwidth nature of LoRa.

## Key Features v2.1

* **Dynamic Location Broadcast:** Automatically adjusts broadcast frequency based on movement speed.
* **Location Request/Response:** Request the current location of a specific node via command; automatically respond if configured.
* **Simple Waypoint Navigation:** Define a target waypoint via command or config; continuously calculates bearing and distance.
* **GeoJSON Logging:** Logs sent and received location data to a `.geojson` file for easy visualization.
* **Node Metadata:** Attach a short status message to your broadcasts, configurable via command or config.
* **Proximity Alerts:** Get notified (via logs) when other AALNP nodes enter a defined proximity radius.
* **Text Message Command Interface:** Control AALNP using commands sent via standard Meshtastic text messages (e.g., set waypoints, request locations, check status). Uses a configurable prefix (default `/aalnp`).
* **Display Integration (Basic):** Outputs current status, navigation info (bearing/distance), or proximity alerts to compatible device OLED screens (if available and supported by firmware/API).
* **Polygon Geo-fencing:** Define named polygonal zones in the configuration; logs alerts upon entering or exiting these zones. Requires `shapely` library.
* **Local Track Storage:** Keeps the last N received location points for each node in memory (configurable). (Querying tracks over LoRa is *not* implemented due to bandwidth limits).
* **Configuration File:** Uses `aalnp_config.json` for persistent settings.
* **Robust & Efficient:** Designed with error handling, graceful shutdown, message queuing, and respect for LoRa TX delays.

## How it Works (New Features)

* **Text Commands:** Listens for standard Meshtastic text messages starting with the `command_prefix`. Parses the command and arguments, executes the action (e.g., updates config, queues a location request), and sends a confirmation or error message back via text.
* **Display Output:** Periodically checks if a screen is available via the API. If yes, formats a short status string (e.g., Nav info, speed, nearby nodes) and uses the API's `show_text` (or similar) function. Updates are throttled.
* **Geo-fencing:** Loads polygon definitions from config. Uses the `shapely` library to perform point-in-polygon tests with the node's current location. Tracks the 'inside'/'outside' state for each fence and logs changes.
* **Local Tracks:** Received AALNP location packets are stored in a dictionary where keys are node IDs and values are `collections.deque` objects with a maximum length defined in the config.

## Installation

1.  **Install Dependencies:**
    ```bash
    pip install meshtastic haversine shapely
    ```
    *(Requires `haversine` and `shapely`; `geomet` is optional and not used)*
    
  For the optional desktop GUI, Tkinter must also be available. On many Linux systems that means installing `python3-tk` from the OS package manager.
2.  **Place Files:** Copy `aalnp_v2_enhanced.py` (or rename) and create/update `aalnp_config.json` (see below) in your runtime directory.
3.  **Run:** Execute the script.
    ```bash
    python aalnp_v2_enhanced.py [--config PATH_TO_CONFIG] [--port SERIAL_PORT]
    ```

  To launch the desktop GUI:
  ```bash
  python aalnp_v2_enhanced.py --gui [--config PATH_TO_CONFIG] [--port SERIAL_PORT]
  ```

## Configuration (`aalnp_config.json`)

Update or create this file. New sections/keys added for v2.1.

```json
{
  "log_file": "location_log.geojson",
  "base_interval_s": 300,
  "fast_interval_s": 60,
  "speed_threshold_mps": 2.0,
  "enable_location_response": true,
  "node_metadata": "AALNP Node",
  "tx_delay_ms_override": null,
  "waypoint": null,
  "proximity_alert": {
    "enabled": true,
    "distance_m": 500,
    "alert_interval_s": 60
  },
  "commands": {
    "prefix": "/aalnp",
    "allow_remote_config": false
  },
  "display": {
    "enabled": true,
    "update_interval_s": 10,
    "mode": "auto"
  },
  "track_storage": {
    "max_points_per_node": 10
  },
  "geofences": [
    {
      "name": "HomeZone",
      "enabled": true,
      "polygon": [
        [43.7417, -79.3733],
        [43.7417, -79.3700],
        [43.7400, -79.3700],
        [43.7400, -79.3733]
      ]
    }
  ]
}
```
## Commands Configuration

```yaml
commands:
  prefix: The text prefix required for commands (e.g., /aalnp).
  allow_remote_config: Set to true to allow commands like setwp and setmeta to modify the running state (use with caution).
  display:
    enabled: Set to true to attempt using the device display.
    update_interval_s: How often to refresh the display.
    mode: What information to prioritize (future enhancement - currently 'auto').
  track_storage:
    max_points_per_node: How many recent locations to store in memory for each other node.
    geofences:
      - name: Unique name for the fence.
        enabled: Enable checking for this fence.
        polygon: A list of [latitude, longitude] coordinate pairs defining the polygon vertices in order. (Needs at least 3 points).
```

> Note: Example coordinates updated.

---

## Usage & Commands

Run the script. Monitor console output. Visualize the GeoJSON log.

If you start with `--gui`, the desktop console exposes:

- Live node and queue status
- Current GPS fix, speed, and last broadcast age
- Activity map showing your position, recent GeoJSON trail, waypoint, and nearby nodes
- Click-to-select nearby nodes directly from the activity map
- Waypoint distance and bearing
- Nearby node summaries with live filtering
- Selected-node telemetry with radio metrics and recent track points
- Direct `reqloc` actions for a selected or manually entered node ID
- Device display preview text
- Recent GeoJSON activity from the configured log file
- Local config controls for metadata and waypoint changes
- Recent AALNP activity logs

Send commands via Meshtastic text message (e.g., using the app or another device):

- `(prefix) help`: Show available commands.
- `(prefix) status`: Get current GPS, speed, and waypoint info.
- `(prefix) setwp <name> <lat> <lon>`: Set the active waypoint (if `allow_remote_config` is true).  
  **Example**: `/aalnp setwp BaseCamp 43.74 -79.37`
- `(prefix) clearwp`: Clear the active waypoint (if `allow_remote_config` is true).
- `(prefix) reqloc <node_id>`: Request the location of another node (use partial hex ID like `!aabbccdd` or `aabbccdd`).  
  **Example**: `/aalnp reqloc !1a2b3c4d`
- `(prefix) setmeta <new_metadata>`: Set your node's metadata string (if `allow_remote_config` is true). Max ~20 chars recommended.  
  **Example**: `/aalnp setmeta Hiking Team Lead`

*(Replace `(prefix)` with the configured command prefix, e.g., `/aalnp`.)*

---

## Dependencies

- Python 3.7+
- `meshtastic`
- `haversine`
- `shapely` (for geo-fencing)
- `geomet` (optional, not used in current implementation)

---

## About Akita Engineering

This project is developed and maintained by **Akita Engineering**.  
Visit us at [www.akitaengineering.com](https://www.akitaengineering.com)

---

## License

This project is licensed under the **GNU General Public License v3.0**.  
Copyright (c) 2025 **Akita Engineering**


