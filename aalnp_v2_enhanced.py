# -*- coding: utf-8 -*-
#
# Akita Advanced Location/Navigation Plugin (AALNP) v2.1 (Enhanced)
# Copyright (C) 2025 Akita Engineering <contact@akitaengineering.com>
# Website: https://www.akitaengineering.com
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
#

import meshtastic
import meshtastic.serial_interface
import meshtastic.util
# Enums/Constants - attempt robust import
try:
    # Try importing from the modern protobuf structure first
    from meshtastic.protobuf import portnums_pb2, mesh_pb2
    Ports = portnums_pb2.PortNum # Use the Enum directly
    DATA_APP_PORT = Ports.PRIVATE_APP # Use PRIVATE_APP for custom data payloads
    POSITION_APP_PORT = Ports.POSITION_APP # Use the enum value directly
    TEXT_MESSAGE_APP_PORT = Ports.TEXT_MESSAGE_APP
    Mesh = mesh_pb2 # For accessing enums like HopLimit
    print("INFO: Imported protobuf definitions (v2.x style).") # Use print for early messages before logging
except ImportError:
    try:
        # Try older style if above fails (check exact paths for older versions)
        from meshtastic import portnums_pb2 as Ports # Alias
        from meshtastic import mesh_pb2
        DATA_APP_PORT = Ports.PRIVATE_APP
        POSITION_APP_PORT = Ports.POSITION_APP
        TEXT_MESSAGE_APP_PORT = Ports.TEXT_MESSAGE_APP
        Mesh = mesh_pb2
        print("INFO: Imported protobuf definitions (older style).")
    except (ImportError, AttributeError):
        # Fallback if specific protobufs cannot be imported
        print("WARN: Could not import specific protobuf definitions. Using fallback integer values.")
        DATA_APP_PORT = 256 # PRIVATE_APP default
        POSITION_APP_PORT = 1 # POSITION_APP default
        TEXT_MESSAGE_APP_PORT = 4403 # TEXT_MESSAGE_APP default


import time
import json
import threading
import os
import argparse
import logging
import queue
import math
import sys
import collections # For deque (track storage)
from datetime import datetime, timezone # For display timestamp
from haversine import haversine, Unit # For distance calculations

try:
    import tkinter as tk
    from tkinter import messagebox
    from tkinter.scrolledtext import ScrolledText
    HAS_TKINTER = True
except ImportError:
    tk = None
    messagebox = None
    ScrolledText = None
    HAS_TKINTER = False

# Attempt to import Shapely for Geo-fencing
try:
    from shapely.geometry import Point as ShapelyPoint
    from shapely.geometry.polygon import Polygon as ShapelyPolygon
    from shapely.errors import ShapelyError
    HAS_SHAPELY = True
except ImportError:
    HAS_SHAPELY = False
    # Define dummy classes/exceptions if shapely is not installed to avoid NameErrors
    ShapelyPoint = None
    ShapelyPolygon = None
    ShapelyError = type('ShapelyError', (Exception,), {}) # Dummy exception class
    print("WARN: Shapely library not found. Geo-fencing will be disabled. Run `pip install shapely` to enable.")


# --- GeoJSON Helper Functions ---
# Define basic GeoJSON structure builders manually to avoid geomet dependency if desired
def create_geojson_point(coordinates):
    """Creates a GeoJSON Point geometry dictionary."""
    return {"type": "Point", "coordinates": coordinates}

def create_geojson_feature(geometry, properties):
    """Creates a GeoJSON Feature dictionary."""
    return {"type": "Feature", "geometry": geometry, "properties": properties}

def create_geojson_feature_collection(features):
    """Creates a GeoJSON FeatureCollection dictionary."""
    return {"type": "FeatureCollection", "features": features}


# --- Configuration ---
DEFAULT_CONFIG_PATH = "aalnp_config.json"
# Default configuration values
DEFAULT_CONFIG = {
    "log_file": "location_log.geojson", # Path for GeoJSON log file
    "base_interval_s": 300,           # Broadcast interval when stationary (seconds)
    "fast_interval_s": 60,            # Broadcast interval when moving (seconds)
    "speed_threshold_mps": 2.0,       # Speed (m/s) to trigger fast interval
    "enable_location_response": True, # Respond to location requests?
    "node_metadata": "AALNP Node",    # Default metadata broadcast with location
    "tx_delay_ms_override": None,     # Manually override LoRa TX delay (ms), null to use device config
    "waypoint": None,                 # Active navigation waypoint {name, latitude, longitude}
    "proximity_alert": {              # Proximity alert settings
        "enabled": True,
        "distance_m": 500,            # Alert radius (meters)
        "alert_interval_s": 60,       # Cooldown between alerts for the same node (seconds)
    },
    "commands": {                     # Text command settings
        "prefix": "/aalnp",           # Prefix for text commands
        "allow_remote_config": False, # Allow changing config (WP, Meta) via command? SECURITY RISK!
    },
    "display": {                      # Device display settings
        "enabled": True,              # Attempt to use OLED display?
        "update_interval_s": 10,      # How often to refresh display (seconds)
        "mode": "auto",               # Display mode (currently only 'auto')
    },
    "track_storage": {                # Local track storage settings
        "max_points_per_node": 10,    # Max recent locations stored per node
    },
    "geofences": []                   # List of geofence definitions
                                      # Example: [{"name": "Home", "enabled": true, "polygon": [[lat,lon],[lat,lon],...]}]
}

# --- Logging Setup ---
# Use a logger specific to this module for better control
logger = logging.getLogger("AALNPv2.1")
# Basic config, handler levels can be set later if needed
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)


# --- General Helper Functions ---
def calculate_bearing(lat1, lon1, lat2, lon2):
    """Calculates the initial bearing (degrees) from point 1 to point 2."""
    # Ensure inputs are valid floats before calculations
    try:
        lat1, lon1, lat2, lon2 = map(math.radians, [float(lat1), float(lon1), float(lat2), float(lon2)])
    except (TypeError, ValueError) as e:
        logger.error(f"Invalid input for bearing calculation: {lat1},{lon1} -> {lat2},{lon2}. Error: {e}")
        return 0.0 # Return default bearing on error

    dLon = lon2 - lon1
    # Formula for bearing calculation
    y = math.sin(dLon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dLon)
    bearing = math.atan2(y, x)
    bearing = math.degrees(bearing)
    bearing = (bearing + 360) % 360 # Normalize to 0-360 degrees
    return bearing

def safe_get(data, key, default=None):
    """Safely get a value from a dictionary or object attribute."""
    if isinstance(data, dict):
        return data.get(key, default)
    elif hasattr(data, key):
        # Guard against accessing methods if not intended
        attr = getattr(data, key, default)
        # Check if the attribute is callable (e.g., a method) and return default if it is
        if callable(attr):
             # logger.debug(f"safe_get: Attribute '{key}' is callable, returning default.")
             return default
        return attr
    else:
        return default

def format_node_id(node_num):
    """Formats node number consistently as hex string (0xdeadbeef)."""
    if node_num is None:
        return "Unknown"
    # Ensure node_num is an integer before formatting
    try:
        # Format as 8-digit hex with 0x prefix
        return f"0x{int(node_num):08x}"
    except (ValueError, TypeError):
        # Fallback if conversion fails
        return str(node_num)


class AALNPGuiLogHandler(logging.Handler):
    """Keeps a short in-memory log history for the optional desktop GUI."""

    def __init__(self, max_entries=250):
        super().__init__()
        self.records = collections.deque(maxlen=max_entries)
        self.records_lock = threading.Lock()

    def emit(self, record):
        try:
            message = self.format(record)
        except Exception:
            message = record.getMessage()
        with self.records_lock:
            self.records.append(message)

    def get_lines(self):
        with self.records_lock:
            return list(self.records)


# --- Main Plugin Class ---
class AALNPv2_Enhanced:
    """
    Akita Advanced Location/Navigation Plugin (AALNP) v2.1 (Enhanced).
    Developed by Akita Engineering (www.akitaengineering.com).
    Adds text commands, display output, geo-fencing, local track storage.
    """

    def __init__(self, interface, config_path=DEFAULT_CONFIG_PATH):
        """
        Initialize the AALNPv2 enhanced plugin.
        Args:
            interface: An initialized Meshtastic interface object.
            config_path (str): Path to the JSON configuration file.
        """
        self.interface = interface
        self.config_path = config_path
        self.config = self._load_config() # Load initial config
        self.node_num = None # Local node number, set on connection
        # Store last known position as a dictionary for easy access
        self.last_pos = {} # Keys: latitude, longitude, altitude, speed, heading, time, accuracy, satsInView, etc.
        self.current_speed_mps = 0.0 # Calculated/reported speed
        self.running = False # Flag indicating if plugin threads are active
        # Queue for outgoing messages {payload dict, type str, destination int, reason str}
        self.msg_queue = queue.Queue()
        self.threads = [] # List to keep track of running threads
        self.tx_delay_ms = 500 # Default LoRa TX delay, potentially overridden by config or device
        self.last_broadcast_time = 0 # Timestamp of the last location broadcast
        # Cache of other nodes' last known location and status
        # Key: node_num (int), Value: dict {lat, lon, ts, meta, alt, spd, hdg, acc, rx_ts, rssi, snr}
        self.other_nodes_location = {}
        # Timestamps for proximity alert cooldowns
        # Key: node_num (int), Value: timestamp (float)
        self.proximity_alert_timestamps = {}

        # --- New state variables for enhanced features ---
        # Store recent track points for each node
        # Key: node_num (int), Value: deque of track point dicts
        self.node_tracks = collections.defaultdict(
            lambda: collections.deque(maxlen=self.config.get('track_storage',{}).get('max_points_per_node', 10))
        )
        # Store compiled Shapely polygons for geofences
        # Key: fence name (str), Value: ShapelyPolygon object
        self.geofence_polygons = {}
        # Track current state relative to each geofence
        # Key: fence name (str), Value: 'unknown'/'inside'/'outside'/'disabled'
        self.geofence_states = {}
        self.last_display_update_time = 0 # Timestamp of last display refresh
        # Store command prefix for quick access
        self.command_prefix = self.config.get('commands', {}).get('prefix', '/aalnp')
        # Lock for potentially concurrent config access (e.g., saving from command thread)
        self.config_lock = threading.Lock()
        # Store last display content for inspection/debugging
        self._last_display = []

        # Initialize features based on loaded config
        self._init_geofences()
        self._update_lora_config() # Initial attempt to get TX delay

    def _deep_merge_dicts(self, base, update):
        """Recursively merge 'update' dictionary into 'base' dictionary."""
        for key, value in update.items():
            if isinstance(value, dict) and key in base and isinstance(base[key], dict):
                # If both base and update have a dict for the same key, recurse
                self._deep_merge_dicts(base[key], value)
            else:
                # Otherwise, overwrite or add the key/value from update to base
                base[key] = value
        return base

    def _load_config(self):
        """Loads configuration from JSON file, merging with defaults deeply."""
        logger.info(f"Loading configuration from: {self.config_path}")
        try:
            with open(self.config_path, 'r') as f:
                user_config = json.load(f)

            # Start with a deep copy of defaults to avoid modifying the original global
            conf = json.loads(json.dumps(DEFAULT_CONFIG)) # Simple deep copy for this structure

            # Deep merge user config into the defaults
            conf = self._deep_merge_dicts(conf, user_config)

            # --- Post-load Validation and Adjustments ---
            # Waypoint: Ensure lat/lon exist and are valid numbers if waypoint object exists
            wp = conf.get('waypoint')
            if isinstance(wp, dict):
                try:
                    lat = float(wp.get('latitude'))
                    lon = float(wp.get('longitude'))
                    if not (-90 <= lat <= 90 and -180 <= lon <= 180):
                         raise ValueError("Coordinates out of range")
                except (TypeError, ValueError, KeyError) as e:
                     logger.warning(f"Waypoint config invalid ({e}). Disabling waypoint.")
                     conf['waypoint'] = None
            elif wp is not None: # Handle cases where waypoint is defined but not a dict
                 logger.warning("Waypoint config is invalid (not a dictionary). Disabling waypoint.")
                 conf['waypoint'] = None


            # Track Storage: Validate max_points_per_node
            track_conf = conf.get('track_storage', {})
            max_len = track_conf.get('max_points_per_node', 10)
            if not isinstance(max_len, int) or max_len < 0:
                 logger.warning(f"Invalid max_track_points_per_node ({max_len}). Using default 10.")
                 # Ensure track_storage dict exists before modifying
                 if 'track_storage' not in conf: conf['track_storage'] = {}
                 conf['track_storage']['max_points_per_node'] = 10

            # Commands: Ensure prefix is a non-empty string
            cmd_conf = conf.get('commands', {})
            prefix = cmd_conf.get('prefix', '/aalnp')
            if not prefix or not isinstance(prefix, str):
                 logger.warning(f"Invalid command prefix '{prefix}'. Using default '/aalnp'.")
                 if 'commands' not in conf: conf['commands'] = {}
                 conf['commands']['prefix'] = '/aalnp'


            logger.info("Configuration loaded successfully.")
            logger.debug(f"Effective config: {json.dumps(conf, indent=2)}")
            return conf

        except FileNotFoundError:
            logger.warning(f"Config file not found at {self.config_path}. Using default settings.")
            try:
                # Attempt to create a default config file
                with open(self.config_path, 'w') as f:
                    json.dump(DEFAULT_CONFIG, f, indent=2)
                logger.info(f"Created default config file: {self.config_path}")
            except Exception as e:
                logger.error(f"Failed to create default config file: {e}")
            return DEFAULT_CONFIG.copy() # Return a copy of defaults
        except json.JSONDecodeError as e:
            logger.error(f"Error decoding config file {self.config_path}: {e}. Using default settings.")
            return DEFAULT_CONFIG.copy()
        except Exception as e:
            logger.error(f"Unexpected error loading config: {e}. Using default settings.")
            return DEFAULT_CONFIG.copy()

    def _save_config(self):
        """Saves current configuration back to file."""
        # Use lock to prevent race conditions if commands modify config while saving
        with self.config_lock:
            logger.debug(f"Saving configuration to: {self.config_path}")
            try:
                # Save a copy to avoid modifying the live config dict during serialization
                config_to_save = self.config.copy()
                with open(self.config_path, 'w') as f:
                    json.dump(config_to_save, f, indent=2)
                logger.info("Configuration saved.")
            except Exception as e:
                logger.error(f"Error saving configuration: {e}")

    def _init_geofences(self):
        """Initializes Shapely polygon objects from config, handling potential errors."""
        # Check if Shapely is available
        if not HAS_SHAPELY:
            # Log only if geofences were actually defined in config
            if self.config.get('geofences'):
                 logger.warning("Shapely library not found/installed. Geo-fencing feature disabled.")
            self.config['geofences'] = [] # Ensure fences list is empty if lib missing
            self.geofence_polygons = {}
            self.geofence_states = {}
            return

        logger.debug("Initializing geo-fences from config...")
        self.geofence_polygons = {}
        self.geofence_states = {}
        fences = self.config.get('geofences', []) # Get fence definitions from config

        # Ensure 'geofences' in config is actually a list
        if not isinstance(fences, list):
             logger.error("Config error: 'geofences' should be a list.")
             return

        # Process each fence definition
        for i, fence_def in enumerate(fences):
            # Basic validation of the definition structure
            if not isinstance(fence_def, dict):
                logger.warning(f"Skipping invalid geofence definition at index {i} (not a dictionary).")
                continue

            name = fence_def.get('name')
            enabled = fence_def.get('enabled', False) # Default to disabled if key missing
            polygon_coords = fence_def.get('polygon')

            # Validate required fields
            if not name:
                logger.warning(f"Skipping geofence definition at index {i} (missing 'name').")
                continue
            if name in self.geofence_polygons:
                 logger.warning(f"Skipping duplicate geo-fence name '{name}'. Ensure names are unique.")
                 continue

            # Handle disabled fences
            if not enabled:
                 logger.info(f"Geo-fence '{name}' is disabled in config.")
                 self.geofence_states[name] = 'disabled'
                 continue

            # Validate polygon coordinates
            if not polygon_coords or not isinstance(polygon_coords, list) or len(polygon_coords) < 3:
                logger.warning(f"Skipping geo-fence '{name}': 'polygon' must be a list of at least 3 coordinate pairs [[lat, lon],...]. Found: {polygon_coords}")
                continue

            # Attempt to create Shapely polygon
            try:
                # Convert [[lat, lon], ...] from config to Shapely's expected [(lon, lat), ...] format
                shapely_coords = []
                for coord_pair in polygon_coords:
                    if isinstance(coord_pair, list) and len(coord_pair) == 2:
                        # Assume [lat, lon] order from config, convert to (lon, lat) for Shapely
                        shapely_coords.append((float(coord_pair[1]), float(coord_pair[0])))
                    else:
                        raise ValueError("Invalid coordinate pair format within polygon list.")

                # Create the polygon object
                polygon = ShapelyPolygon(shapely_coords)

                # Check polygon validity (e.g., self-intersection)
                if not polygon.is_valid:
                     logger.warning(f"Geo-fence '{name}' polygon is invalid (e.g., self-intersecting). Attempting fix using buffer(0)...")
                     # Attempt to fix simple issues like self-touching points
                     polygon = polygon.buffer(0)
                     if not polygon.is_valid:
                          logger.error(f"Failed to fix invalid polygon for geo-fence '{name}'. Disabling this fence.")
                          continue # Skip this invalid fence

                # Store the valid polygon and initial state
                self.geofence_polygons[name] = polygon
                self.geofence_states[name] = 'unknown' # Initial state, determine on first check
                logger.info(f"Initialized geo-fence '{name}'.")

            except (ShapelyError, TypeError, ValueError) as e:
                # Catch errors during coordinate conversion or polygon creation
                logger.error(f"Error initializing geo-fence '{name}': {e}. Check coordinate format [[lat, lon],...].")
            except Exception as e:
                 # Catch any other unexpected errors during initialization
                 logger.error(f"Unexpected error initializing geo-fence '{name}': {e}")

    # --- Core Methods (_update_node_info, _update_lora_config, start, stop, _sleep_interruptible) ---
    def _update_node_info(self):
        """Fetches and stores the local node number, retrying if necessary."""
        if self.node_num is not None: return # Already have it

        logger.debug("Attempting to fetch local node info...")
        max_attempts = 3
        wait_interval = 1.5 # Seconds between attempts
        for attempt in range(max_attempts):
            try:
                # Check if localNode object and 'num' attribute exist and are populated
                if (hasattr(self.interface, 'localNode') and self.interface.localNode and
                    hasattr(self.interface.localNode, 'num') and self.interface.localNode.num is not None):
                     self.node_num = self.interface.localNode.num
                     logger.info(f"Fetched local node number: {self.node_num} ({format_node_id(self.node_num)})")
                     return # Success

                # If localNode not ready, log and wait before retrying
                elif attempt < max_attempts - 1:
                     logger.debug(f"Waiting for localNode info (attempt {attempt+1}/{max_attempts})...")
                     time.sleep(wait_interval)

            except Exception as e:
                 logger.error(f"Error accessing localNode info (attempt {attempt+1}): {e}")
                 # Wait before retrying after an error
                 if attempt < max_attempts - 1: time.sleep(wait_interval)

        # Fallback if localNode info failed after multiple attempts
        logger.warning("localNode info not available after multiple attempts. Trying getMyNodeInfo() as fallback.")
        try:
            my_info = self.interface.getMyNodeInfo()
            if my_info and 'num' in my_info:
                 self.node_num = my_info['num']
                 logger.info(f"Fetched local node number (getMyNodeInfo fallback): {self.node_num} ({format_node_id(self.node_num)})")
            else:
                 logger.error("Could not get local node number via localNode or getMyNodeInfo fallback.")
                 self.node_num = None # Ensure it's None if all attempts fail
        except Exception as e:
            logger.error(f"Error fetching node info via getMyNodeInfo(): {e}")
            self.node_num = None


    def _update_lora_config(self):
        """Fetches LoRa config to determine TX delay, using config override if set."""
        # Check for user override first
        override = self.config.get("tx_delay_ms_override")
        if override is not None and isinstance(override, int):
            self.tx_delay_ms = max(0, override) # Ensure non-negative
            logger.info(f"Using overridden TX delay: {self.tx_delay_ms} ms")
            return

        # Attempt to get from device config (requires connection and populated localNode)
        try:
            # Check modern API structure carefully, ensuring each object exists
            if (hasattr(self.interface, 'localNode') and self.interface.localNode and
                hasattr(self.interface.localNode, 'radioConfig') and self.interface.localNode.radioConfig and
                hasattr(self.interface.localNode.radioConfig, 'preferences') and self.interface.localNode.radioConfig.preferences and
                hasattr(self.interface.localNode.radioConfig.preferences, 'tx_delay')):

                device_delay = self.interface.localNode.radioConfig.preferences.tx_delay
                # Validate the fetched delay value
                if isinstance(device_delay, int) and device_delay >= 0:
                     self.tx_delay_ms = device_delay
                     logger.info(f"Fetched LoRa TX delay from device config: {self.tx_delay_ms} ms")
                else:
                     # Use default if device value is invalid
                     logger.warning(f"Device TX delay invalid ({device_delay}). Using default: {DEFAULT_CONFIG['tx_delay_ms_override'] or 500} ms")
                     self.tx_delay_ms = DEFAULT_CONFIG['tx_delay_ms_override'] or 500
            else:
                 # Use default if the structure isn't found (e.g., older API, node not fully initialized)
                 logger.warning(f"Could not find TX delay in localNode structure. Using default: {DEFAULT_CONFIG['tx_delay_ms_override'] or 500} ms")
                 self.tx_delay_ms = DEFAULT_CONFIG['tx_delay_ms_override'] or 500
        except Exception as e:
            # Catch any errors during access and fallback to default
            logger.error(f"Error fetching LoRa config: {e}. Using default TX delay: {DEFAULT_CONFIG['tx_delay_ms_override'] or 500} ms")
            self.tx_delay_ms = DEFAULT_CONFIG['tx_delay_ms_override'] or 500


    def start(self):
        """Starts the AALNPv2.1 plugin background threads."""
        # Prevent starting if already running or if essential info is missing
        if self.running:
            logger.warning("AALNPv2.1 already running.")
            return
        if self.node_num is None:
             logger.error("Cannot start AALNPv2.1: Local node number not identified.")
             # Attempt to get node number again before failing completely
             self._update_node_info()
             if self.node_num is None:
                  logger.critical("Failed to get node number on start. AALNP cannot run.")
                  return

        logger.info("Starting AALNPv2.1 background threads...")
        self.running = True # Set running flag
        self.threads = [] # Ensure thread list is clear before starting

        # Define and start each thread
        thread_definitions = [
            {"target": self._publish_loop, "name": "AALNP_Publish"},
            {"target": self._broadcast_loop, "name": "AALNP_Broadcast"},
            {"target": self._update_local_state_loop, "name": "AALNP_State"}
        ]

        for t_def in thread_definitions:
            thread = threading.Thread(target=t_def["target"], name=t_def["name"], daemon=True)
            self.threads.append(thread)
            thread.start()
            logger.debug(f"Thread '{t_def['name']}' started.")

        logger.info("AALNPv2.1 started successfully with {} threads.".format(len(self.threads)))


    def stop(self):
        """Stops the AALNPv2.1 plugin background threads gracefully."""
        # Prevent stopping if already stopped
        if not self.running:
            # logger.debug("AALNPv2.1 already stopped.") # Can be noisy during shutdown
            return

        logger.info("Stopping AALNPv2.1 background threads...")
        self.running = False # Signal threads to stop their loops

        # Wait for threads to finish with a timeout
        join_timeout_per_thread = 2.0 # Seconds to wait per thread
        total_timeout = join_timeout_per_thread * len(self.threads)
        start_time = time.time()

        # Filter for threads that are still alive
        active_threads = [t for t in self.threads if t.is_alive()]

        # Loop while there are active threads and timeout hasn't been reached
        while active_threads and (time.time() - start_time) < total_timeout:
            logger.debug(f"Waiting for {len(active_threads)} threads to stop...")
            # Attempt to join each active thread with a short timeout
            for thread in active_threads:
                thread.join(timeout=0.1)
            # Update the list of active threads
            active_threads = [t for t in self.threads if t.is_alive()]
            if not active_threads: break # Exit loop if all threads stopped
            time.sleep(0.1) # Small delay to prevent busy waiting

        # Check if any threads failed to stop within the timeout
        final_active_threads = [t for t in self.threads if t.is_alive()]
        if final_active_threads:
            for thread in final_active_threads:
                logger.warning(f"Thread '{thread.name}' did not stop gracefully within timeout.")
        else:
             logger.debug("All AALNP threads stopped.")

        self.threads = [] # Clear the list of threads
        logger.info("AALNPv2.1 stopped.")


    def _sleep_interruptible(self, duration_s):
        """
        Sleeps for a specified duration, but checks the self.running flag
        periodically to allow for faster shutdown if needed.
        """
        if duration_s <= 0: return # No sleep needed

        end_time = time.time() + duration_s
        while self.running and time.time() < end_time:
            # Calculate remaining sleep time
            remaining_time = end_time - time.time()
            # Sleep for a short interval (e.g., 1 second) or the remaining time, whichever is smaller
            # Use a minimum check interval (e.g., 0.1s) to avoid excessive checks for very short sleeps
            sleep_interval = max(0.1, min(1.0, remaining_time))
            time.sleep(sleep_interval)


    # --- Main Loops (_update_local_state_loop, _broadcast_loop, _publish_loop) ---
    def _update_local_state_loop(self):
        """Periodically updates local state: navigation, proximity, geofence, display."""
        logger.info("Local state update thread started.")
        while self.running:
            try:
                # Use a thread-safe copy of the last known position
                pos = self.last_pos.copy()
                now = time.time()

                # Initialize status strings for display
                nav_info_str = ""
                fence_status_str = ""
                prox_alert_node_id = None
                prox_alert_dist = float('inf')

                # --- Navigation Calculation ---
                wp = self.config.get("waypoint")
                # Check if waypoint is valid and we have a valid current position
                if wp and isinstance(wp, dict) and pos and 'latitude' in pos and 'longitude' in pos:
                    wp_lat = safe_get(wp, 'latitude'); wp_lon = safe_get(wp, 'longitude')
                    wp_name = safe_get(wp, 'name', 'WP')[:7] # Limit name length for display
                    my_lat = pos.get('latitude'); my_lon = pos.get('longitude')

                    # Ensure all coordinates are valid numbers
                    if all(isinstance(c, (int, float)) for c in [wp_lat, wp_lon, my_lat, my_lon]):
                         try:
                              distance_m = haversine((my_lat, my_lon), (wp_lat, wp_lon), unit=Unit.METERS)
                              bearing_deg = calculate_bearing(my_lat, my_lon, wp_lat, wp_lon)
                              logger.info(f"NAV -> '{wp_name}': Dist={distance_m:.0f}m, Bearing={bearing_deg:.1f} deg")
                              # Format distance string concisely (m or km)
                              dist_str = f"{distance_m:.0f}m" if distance_m < 10000 else f"{distance_m/1000.0:.1f}k"
                              # Format navigation info string for display
                              nav_info_str = f"NAV {wp_name}:{dist_str} {bearing_deg:.0f}d"
                         except Exception as calc_e:
                              logger.error(f"Navigation calculation error: {calc_e}")
                    else:
                         logger.debug("Navigation skipped: Invalid coordinates for calculation.")

                # --- Proximity Check ---
                if self.config.get("proximity_alert", {}).get("enabled") and pos and 'latitude' in pos and 'longitude' in pos:
                    # Returns dict {'id', 'dist', 'meta'} of closest node triggering alert, or None
                    closest_node_info = self._check_proximity(pos)
                    if closest_node_info:
                         prox_alert_node_id = closest_node_info['id']
                         prox_alert_dist = closest_node_info['dist']

                # --- Geo-fence Check ---
                # Check only if library loaded, fences exist, and we have a valid position
                if HAS_SHAPELY and self.geofence_polygons and pos and 'latitude' in pos and 'longitude' in pos:
                    fence_status_str = self._check_geofence(pos) # Returns short status like "IN:HomeZn" or ""

                # --- Update Device Display ---
                display_conf = self.config.get("display", {})
                update_interval = display_conf.get("update_interval_s", 10)
                # Check if display is enabled and enough time has passed since last update
                if display_conf.get("enabled") and (now - self.last_display_update_time >= update_interval):
                     # Pass formatted info to display function
                     self._update_display(nav_info_str, prox_alert_node_id, prox_alert_dist, fence_status_str, pos)
                     self.last_display_update_time = now # Update timestamp

                # --- Main loop sleep ---
                # Sleep for a short interval, checking the running flag
                self._sleep_interruptible(1.0) # Check state roughly once per second

            except Exception as e:
                # Catch unexpected errors in the loop
                logger.error(f"Error in local state update loop: {e}", exc_info=True)
                # Prevent busy-looping on error, wait before retrying
                if not self.running: break # Exit if stopped
                self._sleep_interruptible(10.0)


    def _broadcast_loop(self):
        """Periodically fetches GPS data and queues location broadcast message."""
        logger.info("Broadcast thread started.")
        # Initial delay to allow node connection and info to stabilize
        self._sleep_interruptible(2.0)

        while self.running:
            try:
                # --- Pre-broadcast Checks ---
                # Ensure local node number is known
                if self.node_num is None:
                     logger.warning("Broadcast loop waiting: Local node number unknown.")
                     self._update_node_info() # Attempt to get it again
                     self._sleep_interruptible(5.0); continue # Wait and retry

                # --- Determine Broadcast Interval ---
                # Get config values with defaults
                base_interval = self.config.get("base_interval_s", 300)
                fast_interval = self.config.get("fast_interval_s", 60)
                speed_threshold = self.config.get("speed_threshold_mps", 2.0)
                # Choose interval based on current speed
                current_interval = fast_interval if self.current_speed_mps > speed_threshold else base_interval
                # Ensure a minimum interval to prevent flooding
                current_interval = max(5, current_interval) # Minimum 5 seconds

                # --- Check Time Since Last Broadcast ---
                now = time.time()
                time_since_last = now - self.last_broadcast_time
                # If not enough time has passed, sleep until the next interval
                if time_since_last < current_interval:
                    sleep_duration = current_interval - time_since_last
                    logger.debug(f"Broadcast interval not met. Sleeping for {sleep_duration:.1f}s.")
                    self._sleep_interruptible(sleep_duration)
                    # Re-check running flag after sleep before continuing
                    if not self.running: break
                    continue

                # --- Fetch Current GPS Data ---
                position_data = None # Reset position data for this cycle
                try:
                     # Prefer accessing position data directly from localNode if available
                     if (hasattr(self.interface, 'localNode') and self.interface.localNode and
                         hasattr(self.interface.localNode, 'position')):
                          pos_obj = self.interface.localNode.position
                          # Check if position object is populated
                          if pos_obj and safe_get(pos_obj, 'latitude') is not None:
                               # Convert protobuf object to a dictionary for consistent handling
                               position_data = {
                                   k: safe_get(pos_obj, k) for k in [
                                       'latitude', 'longitude', 'altitude', 'speed', 'heading',
                                       'time', 'accuracy', 'horizontal_accuracy', 'groundTrack',
                                       'satsInView', 'pdop', 'hdop', 'vdop', 'precision'
                                   ] if safe_get(pos_obj, k) is not None # Include only non-None values
                               }
                               # Ensure essential fields are present after conversion
                               if 'latitude' not in position_data or 'longitude' not in position_data:
                                    position_data = None # Mark as invalid if core fields missing

                     # Fallback if localNode.position isn't available/populated
                     if position_data is None:
                          logger.debug("localNode.position not available, trying getMyNodeInfo()...")
                          my_info = self.interface.getMyNodeInfo()
                          position_data = my_info.get('position') # Should be a dict already

                except Exception as e:
                     # Catch errors during GPS data fetching
                     logger.error(f"Failed to get position data: {e}")
                     position_data = None # Ensure it's None if fetch fails

                # --- Validate GPS Data ---
                lat = safe_get(position_data, 'latitude')
                lon = safe_get(position_data, 'longitude')
                gps_time = safe_get(position_data, 'time', 0) # Check GPS timestamp

                # Check for essential lat/lon and non-zero coordinates (unless time is also zero, indicating no fix yet)
                is_valid_fix = (lat is not None and lon is not None and (lat != 0 or lon != 0 or gps_time != 0))

                if not is_valid_fix:
                     logger.warning("GPS data not available or invalid (lat/lon missing/zero). Skipping broadcast.")
                     # Clear last known position and speed if fix is lost
                     self.last_pos = {}
                     self.current_speed_mps = 0.0
                     # Wait a shorter time before retrying if GPS fix is lost
                     retry_wait = min(max(5, current_interval // 4), 15) # Wait 1/4 interval or 15s max
                     self._sleep_interruptible(retry_wait)
                     continue # Skip to next loop iteration

                # --- Update Internal State ---
                # Store a copy of the valid position data
                self.last_pos = position_data.copy() if position_data else {}
                # Update current speed (handle different possible field names)
                self.current_speed_mps = self.last_pos.get('speed', self.last_pos.get('groundSpeed', 0.0)) or 0.0

                # --- Prepare Broadcast Payload ---
                payload = {
                    "type": "aalnp_loc", # Message type identifier
                    "node": self.node_num, # Our node number
                    "lat": lat,
                    "lon": lon,
                    "alt": self.last_pos.get('altitude'), # Include altitude if available
                    "spd": self.current_speed_mps,        # Current speed
                    # Use heading, fallback to groundTrack if heading unavailable
                    "hdg": self.last_pos.get('heading', self.last_pos.get('groundTrack')),
                    # Use whichever accuracy field is available ('accuracy' or 'horizontal_accuracy')
                    "acc": self.last_pos.get('accuracy', self.last_pos.get('horizontal_accuracy')),
                    "ts": gps_time if gps_time != 0 else int(now), # Use GPS time if valid, else system time
                    "meta": self.config.get("node_metadata", "") # Include configured metadata
                }
                # Remove any keys with None values for a cleaner payload
                payload = {k: v for k, v in payload.items() if v is not None}

                # --- Queue Message and Log ---
                # Add the prepared payload to the outgoing message queue
                self.msg_queue.put({"payload": payload, "type": "broadcast"})
                self.last_broadcast_time = now # Update timestamp of last broadcast
                logger.info(f"Queued location broadcast (Spd:{self.current_speed_mps:.1f}m/s Int:{current_interval}s)")
                logger.debug(f"Broadcast Payload: {payload}")

                # Log the sent position to the GeoJSON file
                self._log_position_geojson(payload, log_type="SENT")

            except Exception as e:
                # Catch any unexpected errors in the broadcast loop
                logger.error(f"Error in broadcast loop: {e}", exc_info=True)
                # Prevent busy-looping on error
                if not self.running: break # Exit if stopped
                self._sleep_interruptible(10.0) # Wait longer after an error


    def _publish_loop(self):
        """Sends messages from the outgoing queue, respecting LoRa TX delay."""
        logger.info("Publish thread started.")
        while self.running:
            try:
                # Wait for an item from the queue with a timeout
                # Timeout allows checking the self.running flag periodically
                item = self.msg_queue.get(timeout=1.0)

                # If stopped while waiting, exit the loop
                if not self.running: break

                # --- Process Queue Item ---
                payload = item.get("payload")
                msg_type = item.get("type", "broadcast") # Default to broadcast
                # destination_node = item.get("destination") # For potential future direct messaging
                reason = item.get("reason", "periodic") # Reason for sending (e.g., periodic, response)

                # Validate the payload
                if not payload or not isinstance(payload, dict):
                    logger.warning(f"Invalid/empty payload found in publish queue: {item}")
                    self.msg_queue.task_done() # Mark task done even if invalid
                    continue

                # --- Attempt to Send ---
                try:
                    # Serialize payload dictionary to compact JSON bytes (UTF-8)
                    payload_bytes = bytes(json.dumps(payload, separators=(',', ':')), 'utf-8')

                    # Optional: Payload size check (LoRaWAN limits vary, ~51-222 bytes typical)
                    # max_safe_payload = 50 # Be conservative for wider compatibility
                    # if len(payload_bytes) > max_safe_payload:
                    #    logger.warning(f"Payload size ({len(payload_bytes)} bytes) exceeds safe limit ({max_safe_payload}). May fail.")

                    # Sending logic: support both broadcast and direct messaging
                    if msg_type == "direct":
                        destination_node = item.get('destination')
                        if destination_node is None:
                            logger.warning("Direct message requested but no 'destination' provided — sending as broadcast.")
                            self.interface.sendData(payload_bytes, portNum=DATA_APP_PORT, channelIndex=0)
                        else:
                            # Normalize destination to '!aabbccdd' string when possible
                            try:
                                if isinstance(destination_node, str):
                                    ds = destination_node.strip()
                                    if ds.startswith('!'):
                                        dest_id = ds
                                    elif ds.lower().startswith('0x'):
                                        dest_id = f"!{int(ds,16):08x}"
                                    else:
                                        # assume decimal string
                                        dest_id = f"!{int(ds):08x}"
                                elif isinstance(destination_node, int):
                                    dest_id = f"!{destination_node:08x}"
                                else:
                                    dest_id = str(destination_node)

                                # Request ACK for direct sends when supported
                                self.interface.sendData(payload_bytes, destinationId=dest_id, portNum=DATA_APP_PORT, wantAck=True, channelIndex=0)
                                logger.debug(f"Sent direct message to {dest_id} ({len(payload_bytes)} bytes)")
                            except Exception as e:
                                logger.error(f"Failed to send direct message to {destination_node}: {e}", exc_info=True)
                                # Fallback to broadcast if direct send fails
                                try:
                                    self.interface.sendData(payload_bytes, portNum=DATA_APP_PORT, channelIndex=0)
                                except Exception as e2:
                                    logger.error(f"Fallback broadcast also failed: {e2}")
                    else: # Default to broadcast
                        self.interface.sendData(payload_bytes, portNum=DATA_APP_PORT, channelIndex=0) # Send on primary channel
                        logger.debug(f"Published broadcast msg type:'{payload.get('type')}' reason:'{reason}' ({len(payload_bytes)} bytes)")

                    # --- Respect LoRa TX Delay ---
                    delay_s = self.tx_delay_ms / 1000.0
                    if delay_s > 0:
                        # Use interruptible sleep for the delay to allow faster shutdown
                        self._sleep_interruptible(delay_s)

                except json.JSONDecodeError as e:
                     # This should ideally not happen if payload is always a dict
                     logger.error(f"Internal Error: Failed to encode payload to JSON: {e}")
                except meshtastic.MeshtasticError as e:
                     # Catch errors specifically from the Meshtastic library during send
                     logger.error(f"Meshtastic error during sendData: {e}")
                     # Optional: Decide whether to requeue the message on failure
                     # Consider potential infinite loops if the error persists
                     # self.msg_queue.put(item)
                except Exception as e:
                    # Catch any other errors during the sending process
                    logger.error(f"Error sending message from queue: {e}", exc_info=True)

                finally:
                    # Crucial: Mark the task as done in the queue, regardless of success/failure
                    # Prevents queue from blocking indefinitely if errors occur
                    self.msg_queue.task_done()

            except queue.Empty:
                # Queue was empty during the timeout period, this is normal
                # Loop continues, checking self.running flag again
                continue
            except Exception as e:
                # Catch errors related to queue access itself (e.g., get timeout)
                logger.error(f"Error in publish loop (queue handling): {e}", exc_info=True)
                # Prevent busy-looping on unexpected queue errors
                if not self.running: break # Exit if stopped
                self._sleep_interruptible(1.0) # Wait briefly before retrying queue access


    # --- Incoming Packet Handling (on_receive, _handle_location_packet, _handle_location_request) ---
    def on_receive(self, packet, interface):
        """
        Primary callback handler for ALL received packets from Meshtastic.
        Filters and routes packets to appropriate handlers (AALNP data, Text Commands).
        """
        try:
            # --- Basic Packet Validation and Info Extraction ---
            # Use safe_get extensively as packet structure can vary
            decoded_info = safe_get(packet, 'decoded', {}) # Get the decoded part of the packet
            if not decoded_info:
                 logger.debug("Received packet with no 'decoded' field.")
                 return # Cannot process without decoded info

            payload_bytes = safe_get(decoded_info,'payload') # Raw payload bytes
            port_num = safe_get(decoded_info,'portNum', -1) # Port number (int)
            sender_id_raw = safe_get(packet,'from') # Sender node number (int)
            # Ensure sender_id_raw is integer if found
            if sender_id_raw is not None:
                 try: sender_id_raw = int(sender_id_raw)
                 except (ValueError, TypeError): sender_id_raw = None # Invalidate if not int

            # Get hex ID ('!aabbccdd'), prefer 'fromId', construct if missing
            sender_id_hex = safe_get(packet,'fromId')
            if sender_id_hex is None and sender_id_raw is not None:
                 sender_id_hex = f"!{sender_id_raw:08x}" # Construct standard hex format

            # --- Ignore Packets Sent By Ourselves ---
            if sender_id_raw is not None and sender_id_raw == self.node_num:
                # logger.debug("Ignoring packet from self.") # Can be noisy
                return

            # --- Extract Metadata ---
            rssi = safe_get(packet,'rxRssi', safe_get(decoded_info, 'rxRssi', 'N/A'))
            snr = safe_get(packet,'rxSnr', safe_get(decoded_info, 'rxSnr', 'N/A'))
            hop_limit = safe_get(decoded_info, 'hopLimit', 'N/A') # Max hops allowed
            channel = safe_get(packet, 'channel', 'N/A') # Channel index packet was received on

            # Log basic reception info at DEBUG level
            logger.debug(f"Rx Pkt from {sender_id_hex or 'Unknown'} Ch:{channel} Port:{port_num} RSSI:{rssi} SNR:{snr} Hop:{hop_limit}")


            # --- Route Packet Based on Port Number ---

            # 1. Handle Text Messages for Commands
            if port_num == TEXT_MESSAGE_APP_PORT and isinstance(payload_bytes, bytes):
                try:
                    # Decode text message (assuming UTF-8)
                    text_message = payload_bytes.decode('utf-8').strip()
                    # Log received message
                    logger.info(f"Received Text Msg from {sender_id_hex or 'Unknown'}: '{text_message}'")
                    # Check if it starts with our configured command prefix
                    prefix = self.command_prefix # Use stored prefix
                    if text_message.startswith(prefix):
                         # If it's a command, handle it (don't run in callback thread if complex)
                         # For simplicity now, handle directly. Consider threading for long tasks.
                         self._handle_text_command(text_message, sender_id_raw) # Pass raw ID for response
                except UnicodeDecodeError:
                    logger.debug(f"Received non-UTF8 text message from {sender_id_hex or 'Unknown'}.")
                except Exception as e:
                    # Catch errors specifically during text message handling
                    logger.error(f"Error handling text message from {sender_id_hex or 'Unknown'}: {e}", exc_info=True)
                return # Don't process text messages further (e.g., as AALNP data)

            # 2. Handle AALNP Data Packets
            elif port_num == DATA_APP_PORT and isinstance(payload_bytes, bytes):
                logger.debug(f"Processing DATA_APP payload ({len(payload_bytes)} bytes) from {sender_id_hex or 'Unknown'}")
                try:
                    # Decode payload as UTF-8 JSON string
                    payload_str = payload_bytes.decode('utf-8')
                    # Parse JSON string into Python dictionary
                    data = json.loads(payload_str)

                    # Basic validation of AALNP structure
                    if not isinstance(data, dict) or "type" not in data or not data["type"].startswith("aalnp_"):
                         logger.debug(f"DATA_APP JSON from {sender_id_hex or 'Unknown'} received, but not valid AALNP format/type.")
                         return

                    # Route based on AALNP message type field
                    message_type = data.get("type")
                    logger.debug(f"Received AALNP message type '{message_type}' from {sender_id_hex or 'Unknown'}")

                    if message_type == "aalnp_loc":
                        # Handle incoming location packet
                        self._handle_location_packet(data, sender_id_raw, rssi, snr)
                    elif message_type == "aalnp_req":
                        # Handle incoming location request packet
                        self._handle_location_request(data, sender_id_raw)
                    # Add handlers for other AALNP types here if needed (e.g., track response)
                    else:
                        # Log if an unknown AALNP type is received
                        logger.warning(f"Received unknown AALNP message type: {message_type} from {sender_id_hex or 'Unknown'}")

                except UnicodeDecodeError:
                    logger.debug(f"Received non-UTF8 DATA_APP payload from {sender_id_hex or 'Unknown'}.")
                except json.JSONDecodeError:
                    logger.debug(f"Received non-JSON DATA_APP payload from {sender_id_hex or 'Unknown'}.")
                except Exception as e:
                    # Catch errors during specific AALNP packet processing
                    logger.error(f"Error processing AALNP DATA_APP packet from {sender_id_hex or 'Unknown'}: {e}", exc_info=True)
                return # Processed DATA_APP packet

            # 3. Optionally Handle/Ignore Other Ports (like standard POSITION_APP)
            elif port_num == POSITION_APP_PORT:
                 # Attempt to parse standard Meshtastic position packet and treat it as an AALNP location
                 try:
                     pos_payload = None
                     # Check decoded.position dict
                     if isinstance(decoded_info.get('position'), dict):
                         pos_payload = decoded_info.get('position')
                     # Some versions expose latitude/longitude at top-level of decoded
                     elif 'latitude' in decoded_info and 'longitude' in decoded_info:
                         pos_payload = {k: decoded_info.get(k) for k in ('latitude','longitude','altitude','time','accuracy','speed','heading') if k in decoded_info}
                     # Fall back to trying to decode payload bytes as JSON
                     elif isinstance(payload_bytes, bytes):
                         try:
                             parsed = json.loads(payload_bytes.decode('utf-8'))
                             if isinstance(parsed, dict) and ('lat' in parsed or 'latitude' in parsed):
                                 pos_payload = parsed
                         except Exception:
                             pos_payload = None

                     if pos_payload and (('latitude' in pos_payload) or ('lat' in pos_payload)):
                         lat = pos_payload.get('latitude', pos_payload.get('lat'))
                         lon = pos_payload.get('longitude', pos_payload.get('lon'))
                         synthetic = {
                             'type': 'aalnp_loc',
                             'node': sender_id_raw,
                             'lat': lat,
                             'lon': lon,
                             'alt': pos_payload.get('altitude', pos_payload.get('alt')),
                             'spd': pos_payload.get('speed'),
                             'hdg': pos_payload.get('heading'),
                             'acc': pos_payload.get('accuracy'),
                             'ts': pos_payload.get('time')
                         }
                         # Reuse existing location packet handler for consistency
                         self._handle_location_packet(synthetic, sender_id_raw, rssi, snr)
                     else:
                         logger.debug(f"Ignoring standard Meshtastic position packet from {sender_id_hex or 'Unknown'} (no usable position).")
                 except Exception as e:
                     logger.error(f"Error processing standard POSITION_APP packet from {sender_id_hex or 'Unknown'}: {e}", exc_info=True)
                 return

            # 4. Ignore packets on other ports not handled above
            else:
                 # logger.debug(f"Ignoring packet on unhandled port {port_num} from {sender_id_hex or 'Unknown'}.")
                 pass
                 return

        except Exception as e:
            # Catch broad errors in the callback logic itself (e.g., accessing packet fields)
            logger.error(f"Critical error in on_receive handler: {e}", exc_info=True)


    def _handle_location_packet(self, data, sender_id_raw, rssi, snr):
        """Processes received 'aalnp_loc', updates cache, tracks, and logs."""
        # --- Determine the Node ID ---
        node_id_in_payload = data.get('node') # Node ID reported in the payload
        sender_id_hex = format_node_id(sender_id_raw) # Formatted hex ID from transport layer

        # Use transport sender ID if available, otherwise use payload ID
        # Log warning if they mismatch (potential spoofing or relay issue?)
        reported_node_id = sender_id_raw if sender_id_raw is not None else node_id_in_payload
        if reported_node_id is None:
             logger.error(f"Cannot process location packet: No identifiable node ID found. Data: {data}")
             return
        if node_id_in_payload is not None and sender_id_raw is not None and node_id_in_payload != sender_id_raw:
             logger.warning(f"Received location packet where sender ({sender_id_hex}) != payload node ID ({format_node_id(node_id_in_payload)}). Using sender ID ({sender_id_hex}) for tracking.")
             # Trust the transport layer ID (sender_id_raw) more

        # --- Validate Essential Data ---
        if not all(k in data for k in ('lat', 'lon')):
            logger.warning(f"Received invalid location packet (missing lat/lon) from {sender_id_hex}. Data: {data}")
            return

        # --- Log Reception ---
        meta = data.get('meta', '') # Node metadata from payload
        spd = data.get('spd', -1.0) # Speed from payload
        lat = data['lat']; lon = data['lon']
        logger.info(f"Location from {format_node_id(reported_node_id)} ({meta}): "
                    f"Lat={lat:.5f}, Lon={lon:.5f} "
                    f"(Spd={spd:.1f}m/s) RSSI:{rssi}, SNR:{snr}")

        # --- Update Node Location Cache ---
        # Use the determined node ID (prefer transport sender ID) as the key
        cache_key = reported_node_id
        # Store a copy of the received data along with reception metadata
        location_entry = data.copy()
        location_entry['rx_ts'] = time.time() # Timestamp when packet was received
        location_entry['rssi'] = rssi         # Received Signal Strength Indicator
        location_entry['snr'] = snr           # Signal-to-Noise Ratio
        self.other_nodes_location[cache_key] = location_entry

        # --- Update Local Track Storage ---
        track_conf = self.config.get('track_storage', {})
        max_points = track_conf.get('max_points_per_node', 0) # Default 0 if section missing
        if max_points > 0:
             # Store essential track point info (lat, lon, ts, alt, spd, acc)
             track_point = {
                 'lat': lat, 'lon': lon, 'ts': data.get('ts', time.time()), # Use packet timestamp or receive time
                 'alt': data.get('alt'), 'spd': data.get('spd'), 'acc': data.get('acc') }
             # Remove None values for cleaner storage
             track_point = {k:v for k,v in track_point.items() if v is not None}

             # Ensure deque exists with correct maxlen (handles config reloads)
             node_deque = self.node_tracks[cache_key] # Creates deque if not exists
             if node_deque.maxlen != max_points:
                  logger.info(f"Updating track deque maxlen for {format_node_id(cache_key)} from {node_deque.maxlen} to {max_points}")
                  # Recreate deque with new maxlen, preserving existing points
                  current_track = list(node_deque)
                  self.node_tracks[cache_key] = collections.deque(current_track, maxlen=max_points)

             # Append the new track point
             self.node_tracks[cache_key].append(track_point)
             logger.debug(f"Stored track point {len(self.node_tracks[cache_key])}/{max_points} for node {format_node_id(cache_key)}")

        # --- Log to GeoJSON File ---
        # Pass necessary info for logging
        self._log_position_geojson(data, log_type="RECEIVED", rssi=rssi, snr=snr, rx_from_id=sender_id_raw)


    def _handle_location_request(self, data, sender_id_raw):
        """Processes a received location request ('aalnp_req')."""
        # Extract information from the request payload
        target_node = data.get("req_node") # Node whose location is requested
        # Use 'from_node' field if present, otherwise fallback to transport sender ID
        requesting_node = data.get("from_node", sender_id_raw)

        # Format IDs for logging
        sender_id_hex = format_node_id(sender_id_raw)
        req_node_hex = format_node_id(requesting_node)
        target_node_hex = format_node_id(target_node)

        logger.info(f"Received location request for node {target_node_hex} from node {req_node_hex} (via {sender_id_hex})")

        # --- Check if the Request is for this Node ---
        if target_node is not None and target_node == self.node_num:
            # Check if responding is enabled in config
            if self.config.get("enable_location_response", True):
                # Check if we have a valid position to send
                pos_to_send = self.last_pos.copy() # Use last known position
                if pos_to_send and pos_to_send.get('latitude') is not None:
                     logger.info(f"Request is for this node ({target_node_hex})! Queuing location response.")
                     # --- Prepare Response Payload (Standard 'aalnp_loc' format) ---
                     payload = {
                         "type": "aalnp_loc", # Send a standard location packet
                         "node": self.node_num, # Our node number
                         "lat": pos_to_send.get('latitude'),
                         "lon": pos_to_send.get('longitude'),
                         "alt": pos_to_send.get('altitude'),
                         "spd": self.current_speed_mps, # Use current calculated speed
                         "hdg": pos_to_send.get('heading', pos_to_send.get('groundTrack')),
                         "acc": pos_to_send.get('accuracy', pos_to_send.get('horizontal_accuracy')),
                         "ts": pos_to_send.get('time', int(time.time())), # GPS time or system time
                         "meta": self.config.get("node_metadata", "") # Our metadata
                     }
                     # Remove None values for cleaner payload
                     payload = {k: v for k, v in payload.items() if v is not None}

                     # --- Queue the Response ---
                     # Add to outgoing queue, marked as a response
                     self.msg_queue.put({"payload": payload, "type": "broadcast", "reason": "response"})
                     # Log the sent response position
                     self._log_position_geojson(payload, log_type="SENT", reason=f"Response to {req_node_hex}")
                else:
                     # Log if we cannot respond due to lack of GPS fix
                     logger.warning(f"Cannot respond to location request from {req_node_hex}: No valid GPS position available.")
                     # Optionally send a text message back indicating no fix? (Could be noisy)
                     # self._send_text_response("Cannot respond: No GPS fix.", requesting_node)
            else:
                # Log if responding is disabled
                logger.info(f"Ignoring location request from {req_node_hex} as responding is disabled in config.")
        else:
             # Log if the request was for a different node
             logger.debug(f"Location request was for {target_node_hex}, not this node ({format_node_id(self.node_num)}). Ignoring.")


    def _check_proximity(self, pos):
        """Return closest node within proximity alert radius (and not on cooldown).
        Returns {'id': node_id, 'dist': meters, 'meta': meta} or None if no alert triggered.
        """
        try:
            prox_conf = self.config.get('proximity_alert', {})
            if not prox_conf.get('enabled', False):
                return None

            threshold_m = float(prox_conf.get('distance_m', 500))
            cooldown_s = float(prox_conf.get('alert_interval_s', 60))

            lat = safe_get(pos, 'latitude', safe_get(pos, 'lat'))
            lon = safe_get(pos, 'longitude', safe_get(pos, 'lon'))
            if lat is None or lon is None:
                return None

            closest_node = None
            closest_dist = float('inf')
            for node_id, node_data in self.other_nodes_location.items():
                node_lat = safe_get(node_data, 'lat', safe_get(node_data, 'latitude'))
                node_lon = safe_get(node_data, 'lon', safe_get(node_data, 'longitude'))
                if node_lat is None or node_lon is None:
                    continue
                try:
                    d = haversine((float(lat), float(lon)), (float(node_lat), float(node_lon)), unit=Unit.METERS)
                except Exception:
                    continue
                if d < closest_dist:
                    closest_dist = d
                    closest_node = node_id

            if closest_node is None or closest_dist > threshold_m:
                return None

            now = time.time()
            last_ts = self.proximity_alert_timestamps.get(closest_node, 0)
            if (now - last_ts) < cooldown_s:
                # Suppress repeated alerts for the same node during cooldown
                logger.debug(f"Proximity alert for {format_node_id(closest_node)} suppressed by cooldown ({now-last_ts:.0f}s<{cooldown_s}s).")
                return None

            # Record the alert timestamp and return info
            self.proximity_alert_timestamps[closest_node] = now
            meta = self.other_nodes_location.get(closest_node, {}).get('meta', '')
            logger.info(f"Proximity alert: Node {format_node_id(closest_node)} is {closest_dist:.0f}m away (threshold {threshold_m}m).")
            return {'id': closest_node, 'dist': closest_dist, 'meta': meta}
        except Exception as e:
            logger.error(f"Error in _check_proximity: {e}", exc_info=True)
            return None


    def _check_geofence(self, pos):
        """Check configured geofences and update states.
        Returns a short status string for display (e.g. 'IN:Home') or empty string if none.
        """
        if not HAS_SHAPELY or not self.geofence_polygons:
            return ""
        lat = safe_get(pos, 'latitude', safe_get(pos, 'lat'))
        lon = safe_get(pos, 'longitude', safe_get(pos, 'lon'))
        if lat is None or lon is None:
            return ""

        try:
            pt = ShapelyPoint(float(lon), float(lat))
            inside_names = []
            for name, polygon in self.geofence_polygons.items():
                if polygon is None:
                    continue
                prev_state = self.geofence_states.get(name, 'unknown')
                try:
                    inside = polygon.contains(pt) or polygon.touches(pt) or polygon.covers(pt)
                except Exception as e:
                    logger.error(f"Error evaluating geo-fence '{name}': {e}")
                    continue

                if inside:
                    if prev_state != 'inside':
                        logger.info(f"Entered geo-fence '{name}'.")
                    self.geofence_states[name] = 'inside'
                    inside_names.append(name)
                else:
                    if prev_state == 'inside':
                        logger.info(f"Exited geo-fence '{name}'.")
                    self.geofence_states[name] = 'outside'

            # Return a compact display string for the first matching fence
            return f"IN:{inside_names[0][:12]}" if inside_names else ""
        except Exception as e:
            logger.error(f"Error in _check_geofence: {e}", exc_info=True)
            return ""


    def _update_display(self, nav_info_str, prox_alert_node_id, prox_alert_dist, fence_status_str, pos):
        """Update an attached display (best-effort) and store last display lines for inspection.
        If no physical display is available, this simply logs the intended content.
        """
        try:
            lines = []
            if nav_info_str:
                lines.append(nav_info_str)
            else:
                lat = safe_get(pos, 'latitude', safe_get(pos, 'lat'))
                lon = safe_get(pos, 'longitude', safe_get(pos, 'lon'))
                if lat is not None and lon is not None:
                    lines.append(f"{lat:.5f},{lon:.5f}")
                else:
                    lines.append("No GPS Fix")

            if prox_alert_node_id:
                lines.append(f"Near {format_node_id(prox_alert_node_id)} {prox_alert_dist:.0f}m")
            elif fence_status_str:
                lines.append(fence_status_str)
            else:
                lines.append(self.config.get('node_metadata','')[:16])

            # Save last display content (useful for tests/inspection)
            self._last_display = lines

            # Best-effort: if the meshtastic interface exposes a display API, try to use it
            try:
                if hasattr(self.interface, 'setDisplay'):
                    # Some custom interfaces may implement a setDisplay(text) convenience method
                    self.interface.setDisplay('\n'.join(lines))
                elif hasattr(self.interface, 'displayUpdate'):
                    self.interface.displayUpdate(lines)
            except Exception:
                # Do not raise if display update fails on devices with no display
                logger.debug("Physical display update failed or not supported (ignored).")

            logger.debug("Display content: %s", " | ".join(lines))
        except Exception as e:
            logger.error(f"Error in _update_display: {e}", exc_info=True)

    def get_runtime_snapshot(self):
        """Return a GUI-friendly snapshot of the current plugin state."""
        with self.config_lock:
            config_copy = json.loads(json.dumps(self.config))

        now = time.time()
        pos = self.last_pos.copy() if isinstance(self.last_pos, dict) else {}
        lat = safe_get(pos, 'latitude', safe_get(pos, 'lat'))
        lon = safe_get(pos, 'longitude', safe_get(pos, 'lon'))

        navigation = None
        waypoint = config_copy.get('waypoint') if isinstance(config_copy.get('waypoint'), dict) else None
        if waypoint and lat is not None and lon is not None:
            wp_lat = safe_get(waypoint, 'latitude')
            wp_lon = safe_get(waypoint, 'longitude')
            if wp_lat is not None and wp_lon is not None:
                try:
                    distance_m = haversine((float(lat), float(lon)), (float(wp_lat), float(wp_lon)), unit=Unit.METERS)
                    bearing_deg = calculate_bearing(float(lat), float(lon), float(wp_lat), float(wp_lon))
                    navigation = {
                        "distance_m": distance_m,
                        "bearing_deg": bearing_deg,
                        "waypoint_name": safe_get(waypoint, 'name', 'WP'),
                    }
                except Exception:
                    navigation = None

        other_nodes = []
        for node_id, node_data in list(self.other_nodes_location.items()):
            entry = node_data.copy() if isinstance(node_data, dict) else {}
            entry['node_id'] = node_id
            entry['node_id_hex'] = format_node_id(node_id)
            rx_ts = safe_get(entry, 'rx_ts')
            entry['age_s'] = max(0.0, now - float(rx_ts)) if rx_ts else None

            node_lat = safe_get(entry, 'lat', safe_get(entry, 'latitude'))
            node_lon = safe_get(entry, 'lon', safe_get(entry, 'longitude'))
            if lat is not None and lon is not None and node_lat is not None and node_lon is not None:
                try:
                    entry['distance_m'] = haversine((float(lat), float(lon)), (float(node_lat), float(node_lon)), unit=Unit.METERS)
                except Exception:
                    entry['distance_m'] = None
            else:
                entry['distance_m'] = None
            other_nodes.append(entry)

        other_nodes.sort(key=lambda item: safe_get(item, 'rx_ts', 0) or 0, reverse=True)

        try:
            queue_depth = self.msg_queue.qsize()
        except Exception:
            queue_depth = 0

        return {
            "running": self.running,
            "connected": self.running and self.node_num is not None,
            "node_num": self.node_num,
            "node_id_hex": format_node_id(self.node_num),
            "tx_delay_ms": self.tx_delay_ms,
            "queue_depth": queue_depth,
            "speed_mps": self.current_speed_mps,
            "position": pos,
            "position_fix": lat is not None and lon is not None,
            "metadata": config_copy.get('node_metadata', ''),
            "command_prefix": config_copy.get('commands', {}).get('prefix', '/aalnp'),
            "waypoint": waypoint,
            "navigation": navigation,
            "other_nodes": other_nodes,
            "heard_count": len(other_nodes),
            "display_lines": list(self._last_display),
            "geofence_states": dict(self.geofence_states),
            "last_broadcast_age_s": (now - self.last_broadcast_time) if self.last_broadcast_time else None,
            "log_file": config_copy.get('log_file'),
            "config_path": self.config_path,
        }

    def set_waypoint(self, name, latitude, longitude):
        """Public helper used by the desktop GUI to set waypoint config."""
        return self._cmd_set_wp([name, str(latitude), str(longitude)])

    def clear_waypoint(self):
        """Public helper used by the desktop GUI to clear waypoint config."""
        return self._cmd_clear_wp()

    def set_metadata(self, metadata):
        """Public helper used by the desktop GUI to update node metadata."""
        return self._cmd_set_meta([metadata])


    # --- Text Command Handling ---
    def _handle_text_command(self, text, sender_id_raw):
        """Parses and executes commands received via text message."""
        # Get command configuration
        commands_conf = self.config.get('commands', {})
        prefix = commands_conf.get('prefix', '/aalnp')
        allow_remote_cfg = commands_conf.get('allow_remote_config', False)

        # Basic validation (should already be checked by caller)
        if not text.strip().startswith(prefix): return

        # Parse command and arguments
        parts = text.strip().split()
        if len(parts) < 2: # Needs at least prefix + command
            self._send_text_response("Usage: " + prefix + " <command> [args]", sender_id_raw)
            return

        command = parts[1].lower() # Command is case-insensitive
        args = parts[2:] # Arguments are the rest of the parts
        sender_id_hex = format_node_id(sender_id_raw) # For logging
        logger.info(f"Processing command '{command}' with args {args} from {sender_id_hex}")

        # --- Command Routing ---
        response = "" # Initialize response string/list
        try:
            # Route to appropriate command handler method
            if command == "help":
                response = "Cmds: help, status, setwp, clearwp, reqloc, setmeta"
            elif command == "status":
                response = self._cmd_status() # Returns status string or list
            elif command == "setwp":
                # Check permission before executing config-changing command
                if not allow_remote_cfg: response = "Error: Remote config disabled."
                else: response = self._cmd_set_wp(args)
            elif command == "clearwp":
                if not allow_remote_cfg: response = "Error: Remote config disabled."
                else: response = self._cmd_clear_wp()
            elif command == "reqloc":
                response = self._cmd_req_loc(args)
            elif command == "setmeta":
                if not allow_remote_cfg: response = "Error: Remote config disabled."
                else: response = self._cmd_set_meta(args)
            else:
                # Handle unknown commands
                response = f"Unknown command: {command}. Try '{prefix} help'"

        except Exception as cmd_e:
             # Catch unexpected errors during command execution
             logger.error(f"Error executing command '{command}' from {sender_id_hex}: {cmd_e}", exc_info=True)
             response = "Error executing command."

        # --- Send Response(s) ---
        # Handle single string or list of strings for multi-line responses
        if isinstance(response, list):
            for i, line in enumerate(response):
                self._send_text_response(line, sender_id_raw)
                # Add a small delay between multi-line messages to avoid flooding
                if i < len(response) - 1: time.sleep(0.3)
        elif response: # Non-empty single string response
            self._send_text_response(response, sender_id_raw)
        # else: No response needed (e.g., for commands that don't produce output)


    def _send_text_response(self, message, destination_node_id):
        """Sends a response message back to the command sender via Meshtastic text message."""
        # Validate destination ID
        if destination_node_id is None:
            logger.warning("Cannot send text response: Destination Node ID is None.")
            return
        try:
             # Ensure message is not excessively long for text payload
             max_len = 200 # Conservative estimate for typical text message limits
             if len(message) > max_len:
                  logger.warning(f"Truncating text response exceeding {max_len} chars.")
                  message = message[:max_len-3] + "..." # Truncate and add ellipsis

             # Ensure destination ID is integer for sendText API call
             dest_id_int = int(destination_node_id)

             logger.info(f"Sending text response to {format_node_id(dest_id_int)}: '{message}'")
             # Use sendText API with destination node number
             # Set wantAck=False for simple responses to avoid needing ack handling/timeouts
             # Send on primary channel (channelIndex=0)
             self.interface.sendText(message, destinationId=dest_id_int, wantAck=False, channelIndex=0)

             # Add a small delay after sending text, accounting for TX delay,
             # to prevent potential issues if another message is queued immediately.
             time.sleep(0.2 + (self.tx_delay_ms / 1000.0))

        except ValueError:
             # Handle error if destination_node_id cannot be converted to int
             logger.error(f"Cannot send text response: Invalid destination Node ID '{destination_node_id}'. Must be integer.")
        except meshtastic.MeshtasticError as e:
             # Catch errors from the Meshtastic library during sendText
             logger.error(f"Failed to send text response to {format_node_id(destination_node_id)}: {e}")
        except Exception as e:
             # Catch any other unexpected errors
             logger.error(f"Unexpected error sending text response: {e}", exc_info=True)


    # --- Command Implementation Methods (return string or list of strings for response) ---
    def _cmd_status(self):
        """Handles the 'status' command. Returns list of status strings."""
        pos = self.last_pos.copy(); lines = [] # Use thread-safe copy
        lat = pos.get('latitude', None); lon = pos.get('longitude', None)
        spd = self.current_speed_mps
        fix_status = "OK" if (lat is not None and lon is not None) else "No"
        sats = pos.get('satsInView', '--') # Satellites in view
        # Accuracy (use available field, default 0)
        acc = pos.get('accuracy', pos.get('horizontal_accuracy', 0)) or 0
        # Line 1: GPS Fix, Satellites, Accuracy, Speed
        lines.append(f"GPS:{fix_status} S:{sats} A:{acc:.0f}m S:{spd:.1f}m/s")

        # Line 2: Waypoint, Metadata, Nodes Heard
        wp = self.config.get('waypoint')
        wp_name = wp.get('name', 'None') if wp else 'None'
        meta = self.config.get('node_metadata', '')[:15] # Truncate metadata
        nodes_heard = len(self.other_nodes_location) # Number of other nodes heard from
        lines.append(f"WP:{wp_name[:8]} M:{meta} N:{nodes_heard}") # Truncate WP name

        return lines


    def _cmd_set_wp(self, args):
        """Handles the 'setwp <name> <lat> <lon>' command. Returns confirmation/error string."""
        # Validate number of arguments
        if len(args) < 3:
            return f"Usage: {self.command_prefix} setwp <name> <latitude> <longitude>"

        name = args[0]
        lat_str = args[1]
        lon_str = args[2]

        try:
            # Convert coordinates to float and validate range
            lat = float(lat_str)
            lon = float(lon_str)
            if not (-90 <= lat <= 90 and -180 <= lon <= 180):
                 raise ValueError("Coordinates out of valid range (-90/90, -180/180)")

            # Update waypoint in config (use lock for thread safety)
            with self.config_lock:
                 self.config['waypoint'] = {"name": name, "latitude": lat, "longitude": lon}
                 logger.info(f"Waypoint set via command: {name} ({lat}, {lon})")
                 self._save_config() # Persist the change immediately

            return f"Waypoint '{name}' set."

        except ValueError as e:
             # Handle errors during float conversion or validation
             return f"Error: Invalid coordinates '{lat_str}', '{lon_str}'. {e}"
        except Exception as e:
             # Catch other unexpected errors
             logger.error(f"Error processing setwp command: {e}", exc_info=True)
             return "Error setting waypoint."


    def _cmd_clear_wp(self):
        """Handles the 'clearwp' command. Returns confirmation/error string."""
        # Check if a waypoint is currently set
        if self.config.get('waypoint') is not None:
            # Clear waypoint in config (use lock)
            with self.config_lock:
                 self.config['waypoint'] = None
                 logger.info("Waypoint cleared via command.")
                 self._save_config() # Persist
            return "Waypoint cleared."
        else:
            # Inform user if no waypoint was active
            return "No active waypoint to clear."


    def _cmd_req_loc(self, args):
        """Handles the 'reqloc <node_id_hex>' command. Returns confirmation/error string."""
        # Validate argument count
        if len(args) < 1:
            return f"Usage: {self.command_prefix} reqloc <node_id_hex>"

        target_id_str = args[0].replace("!", "").replace("0x","") # Clean up input hex string

        try:
            # Convert hex string to integer node number
            target_node_num = int(target_id_str, 16)

            # Prepare the request payload
            payload = {
                "type": "aalnp_req",        # AALNP request type
                "req_node": target_node_num, # Node ID whose location is requested
                "from_node": self.node_num  # Include our node ID so receiver knows who asked
            }

            # Add the request message to the outgoing queue
            self.msg_queue.put({"payload": payload, "type": "broadcast", "reason": "user_request"})
            logger.info(f"Queued location request for node {target_id_str}")

            return f"Location request sent for {target_id_str}."

        except ValueError:
            # Handle error if hex string is invalid
            return f"Error: Invalid Node ID hex format '{args[0]}'."
        except Exception as e:
            # Catch other unexpected errors
            logger.error(f"Error processing reqloc command: {e}", exc_info=True)
            return "Error sending location request."


    def _cmd_set_meta(self, args):
        """Handles the 'setmeta <metadata text>' command. Returns confirmation/error string."""
        # Validate arguments
        if not args:
            return f"Usage: {self.command_prefix} setmeta <new_metadata_text>"

        # Join arguments into a single metadata string
        new_meta = " ".join(args)
        max_meta_len = 30 # Define a reasonable maximum length
        response = "" # Initialize response string

        # Truncate if necessary and inform user
        if len(new_meta) > max_meta_len:
             original_meta = new_meta
             new_meta = new_meta[:max_meta_len]
             logger.warning(f"Metadata truncated from '{original_meta}' to '{new_meta}' (max {max_meta_len} chars).")
             response = f"WARN: Meta truncated to {max_meta_len} chars.\n" # Add warning to response

        # Update metadata in config (use lock)
        with self.config_lock:
             self.config['node_metadata'] = new_meta
             logger.info(f"Node metadata set via command: '{new_meta}'")
             self._save_config() # Persist

        # Append confirmation to response
        response += f"Metadata set to: {new_meta}"
        return response


    # --- Logging and Connection Callbacks ---
    def _log_position_geojson(self, pos_data, log_type="UNKNOWN", reason=None, **kwargs):
        """Logs position data as a GeoJSON Feature to the configured log file."""
        # Get log file path from config
        log_file = self.config.get("log_file")
        # Silently return if logging is disabled (path is None or empty)
        if not log_file: return

        try:
            # --- Extract Coordinates ---
            # Use safe_get for robustness against missing keys
            lon = safe_get(pos_data, 'lon', safe_get(pos_data, 'longitude'))
            lat = safe_get(pos_data, 'lat', safe_get(pos_data, 'latitude'))
            # Altitude is optional, keep as None if missing
            alt = safe_get(pos_data, 'alt', safe_get(pos_data, 'altitude'))

            # --- Validate Coordinates ---
            # Ensure latitude and longitude are present and valid numbers
            if lon is None or lat is None or not isinstance(lat, (int, float)) or not isinstance(lon, (int, float)):
                 node_id_str = format_node_id(pos_data.get('node', 'Unknown'))
                 logger.error(f"GeoJSON log skipped for node {node_id_str}: Invalid coordinates (lat={lat}, lon={lon})")
                 return

            # --- Create Geometry ---
            # Create coordinate tuple (handle optional altitude)
            coordinates = (lon, lat, alt) if alt is not None and isinstance(alt, (int, float)) else (lon, lat)
            geometry = create_geojson_point(coordinates) # Use helper function

            # --- Prepare Properties ---
            properties = {
                "log_type": log_type, # SENT or RECEIVED
                "node_id": safe_get(pos_data, 'node'), # Node ID from payload
                "latitude": lat,
                "longitude": lon,
                "altitude_m": alt, # Altitude in meters
                "speed_mps": safe_get(pos_data, 'spd'), # Speed in m/s
                "heading_deg": safe_get(pos_data, 'hdg'), # Heading in degrees
                "h_accuracy_m": safe_get(pos_data, 'acc'), # Horizontal accuracy in meters
                "gps_timestamp": safe_get(pos_data, 'ts'), # Unix timestamp from GPS
                "log_timestamp": time.time(), # System timestamp when logged
                "metadata": safe_get(pos_data, 'meta'), # Node metadata string
                "reason": reason, # Optional reason for the log entry (e.g., response)
            }

            # Add receiver-specific info if it was a received packet
            if log_type == "RECEIVED":
                properties["rssi"] = kwargs.get('rssi') # Received Signal Strength Indicator
                properties["snr"] = kwargs.get('snr')   # Signal-to-Noise Ratio
                # Use the sender ID from the transport layer if available, fallback to payload node ID
                properties["rx_from_id"] = kwargs.get('rx_from_id', properties["node_id"])

            # Ensure node_id is our node_num if it was a SENT packet
            if log_type == "SENT":
                 properties["node_id"] = self.node_num

            # Remove properties with None values for cleaner output
            properties = {k: v for k, v in properties.items() if v is not None}

            # --- Create Feature ---
            feature = create_geojson_feature(geometry, properties) # Use helper function

            # --- Read/Write FeatureCollection (Atomic Write) ---
            feature_collection = None
            # Check if log file exists and is not empty
            if os.path.exists(log_file) and os.path.getsize(log_file) > 0:
                try:
                    # Attempt to read existing FeatureCollection
                    with open(log_file, 'r') as f:
                         feature_collection = json.load(f)
                         # Basic validation of the loaded structure
                         if not isinstance(feature_collection, dict) or \
                            feature_collection.get("type") != "FeatureCollection" or \
                            not isinstance(feature_collection.get("features"), list):
                             logger.warning(f"Log file '{log_file}' is not a valid GeoJSON FeatureCollection. Resetting.")
                             feature_collection = None # Reset if invalid
                except json.JSONDecodeError:
                    # Handle case where file contains invalid JSON
                    logger.warning(f"Log file '{log_file}' contains invalid JSON. Resetting.")
                    feature_collection = None
                except Exception as e:
                    # Handle other potential read errors
                    logger.error(f"Error reading log file '{log_file}': {e}. Resetting.")
                    feature_collection = None

            # If reading failed or file was empty/invalid, create a new empty collection
            if feature_collection is None:
                 feature_collection = create_geojson_feature_collection([]) # Use helper

            # --- Append new feature ---
            feature_collection["features"].append(feature)

            # --- Write back atomically (Write to temp file, then rename) ---
            # This prevents data loss if the script crashes during write
            temp_log_file = log_file + ".tmp"
            try:
                 with open(temp_log_file, 'w') as f:
                      # Use compact JSON format (no indentation) for smaller file size
                      json.dump(feature_collection, f, indent=None, separators=(',', ':'))
                 # Atomic rename (on most OSes) - replaces original file instantly
                 os.replace(temp_log_file, log_file)
                 # Log success at DEBUG level to avoid excessive console output
                 node_id_str = format_node_id(properties.get('node_id', 'Unknown'))
                 logger.debug(f"Logged {log_type} position for node {node_id_str} to {log_file}")
            except Exception as write_e:
                 # Handle errors during write/rename
                 logger.error(f"Failed to write log file '{log_file}': {write_e}")
                 # Attempt to remove temporary file if it exists to avoid clutter
                 if os.path.exists(temp_log_file):
                      try: os.remove(temp_log_file)
                      except Exception as rm_e: logger.error(f"Failed to remove temporary log file '{temp_log_file}': {rm_e}")

        except Exception as e:
            # Catch errors in the logging logic itself
            logger.error(f"Failed to log GeoJSON position: {e}", exc_info=True)


    def on_connection(self, interface, connected):
        """Callback handler for Meshtastic connection status changes."""
        if connected:
            logger.info("Meshtastic connection established.")
            # Attempt to get node info immediately after connection
            self._update_node_info()
            # Proceed only if node ID was successfully obtained
            if self.node_num is not None:
                # Get device-specific config (like TX delay) now that we are connected
                self._update_lora_config()
                # Start the main plugin operation threads
                self.start()
            else:
                 # Log error and do not start threads if node ID is missing
                 logger.error("Meshtastic connected, but failed to get local node number. AALNPv2.1 cannot start.")
                 # Consider scheduling a retry or notifying user?
        else:
            # Handle disconnection
            logger.warning("Meshtastic connection lost.")
            self.stop() # Stop plugin operation threads
            self.node_num = None # Reset node ID as we are disconnected


class AALNPDesktopGUI:
    """Optional local desktop console for monitoring and editing AALNP state."""

    def __init__(self, plugin, log_handler):
        if not HAS_TKINTER:
            raise RuntimeError("Tkinter is not available in this Python environment.")

        self.plugin = plugin
        self.log_handler = log_handler
        self.root = tk.Tk()
        self.root.title("Akita AALNP // Titanium Console")
        self.root.geometry("1360x920")
        self.root.minsize(1220, 820)

        self.colors = {
            "bg": "#07090b",
            "panel": "#101318",
            "surface": "#191e24",
            "surface_2": "#252c34",
            "border": "#59616a",
            "silver": "#c2cad2",
            "white": "#f5f7fa",
            "accent": "#39e67d",
            "accent_2": "#1f8b50",
            "muted": "#8a929a",
            "titanium": "#7a838d",
        }
        self.root.configure(bg=self.colors["bg"])

        self.font_family = "DejaVu Sans"
        self.mono_family = "DejaVu Sans Mono"
        self.status_widgets = {}
        self.metric_widgets = {}
        self.last_logs_text = None
        self.last_nodes_text = None
        self.last_display_text = None
        self._closed = False

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def f(self, size, weight="normal"):
        return (self.font_family, size, weight)

    def fm(self, size, weight="bold"):
        return (self.mono_family, size, weight)

    def _badge_palette(self, tone):
        tones = {
            "neutral": (self.colors["surface"], self.colors["silver"]),
            "muted": (self.colors["surface"], self.colors["muted"]),
            "active": (self.colors["surface"], self.colors["accent"]),
            "bright": (self.colors["white"], self.colors["bg"]),
        }
        return tones.get(tone, tones["neutral"])

    def _make_badge(self, parent, text, tone="neutral"):
        bg, fg = self._badge_palette(tone)
        return tk.Label(parent, text=text, font=self.f(10, "bold"), bg=bg, fg=fg, padx=12, pady=6)

    def _set_badge(self, widget, text, tone="neutral"):
        bg, fg = self._badge_palette(tone)
        widget.config(text=text, bg=bg, fg=fg)

    def _make_card(self, parent, title, subtitle=None):
        card = tk.Frame(
            parent,
            bg=self.colors["panel"],
            highlightbackground=self.colors["border"],
            highlightcolor=self.colors["border"],
            highlightthickness=1,
            bd=0,
        )
        tk.Frame(card, bg=self.colors["accent"], height=3).pack(fill=tk.X)
        header = tk.Frame(card, bg=self.colors["panel"])
        header.pack(fill=tk.X, padx=16, pady=(14, 10))
        tk.Label(header, text=title.upper(), font=self.f(11, "bold"), fg=self.colors["white"], bg=self.colors["panel"]).pack(anchor="w")
        if subtitle:
            tk.Label(header, text=subtitle, font=self.f(9), fg=self.colors["silver"], bg=self.colors["panel"]).pack(anchor="w", pady=(4, 0))
        body = tk.Frame(card, bg=self.colors["panel"])
        body.pack(fill=tk.BOTH, expand=True, padx=16, pady=(0, 16))
        return card, body

    def _make_metric_tile(self, parent, title, row, col):
        tile = tk.Frame(
            parent,
            bg=self.colors["surface"],
            highlightbackground=self.colors["border"],
            highlightcolor=self.colors["border"],
            highlightthickness=1,
            bd=0,
        )
        tile.grid(row=row, column=col, sticky="nsew", padx=6, pady=6)
        tk.Label(tile, text=title.upper(), font=self.f(8, "bold"), fg=self.colors["silver"], bg=self.colors["surface"]).pack(anchor="w", padx=12, pady=(10, 4))
        value = tk.Label(tile, text="--", font=self.fm(14), fg=self.colors["white"], bg=self.colors["surface"])
        value.pack(anchor="w", padx=12, pady=(0, 10))
        return value

    def _make_button(self, parent, text, command, tone="secondary"):
        palettes = {
            "secondary": (self.colors["surface_2"], self.colors["white"], self.colors["titanium"]),
            "primary": (self.colors["accent"], self.colors["bg"], "#6ff0a3"),
            "bright": (self.colors["white"], self.colors["bg"], self.colors["silver"]),
        }
        bg, fg, hover = palettes.get(tone, palettes["secondary"])
        button = tk.Button(
            parent,
            text=text,
            command=command,
            font=self.f(9, "bold"),
            bg=bg,
            fg=fg,
            activebackground=hover,
            activeforeground=fg,
            relief=tk.FLAT,
            bd=0,
            highlightthickness=0,
            padx=14,
            pady=10,
            cursor="hand2",
        )
        button.bind("<Enter>", lambda _event, btn=button, color=hover: btn.config(bg=color))
        button.bind("<Leave>", lambda _event, btn=button, color=bg: btn.config(bg=color))
        return button

    def _style_entry(self, entry):
        entry.config(
            bg=self.colors["surface"],
            fg=self.colors["white"],
            insertbackground=self.colors["white"],
            relief=tk.FLAT,
            highlightthickness=1,
            highlightbackground=self.colors["border"],
            highlightcolor=self.colors["accent"],
            font=self.f(10),
        )

    def _set_text_widget(self, widget, content):
        widget.config(state=tk.NORMAL)
        widget.delete("1.0", tk.END)
        widget.insert("1.0", content)
        widget.config(state=tk.DISABLED)

    def _build_ui(self):
        shell = tk.Frame(self.root, bg=self.colors["bg"])
        shell.pack(fill=tk.BOTH, expand=True, padx=18, pady=18)

        header_card, header_body = self._make_card(
            shell,
            "Akita AALNP",
            "Advanced location, navigation, geofence, and mesh activity console.",
        )
        header_card.pack(fill=tk.X, pady=(0, 14))
        header_body.grid_columnconfigure(0, weight=1)

        left_header = tk.Frame(header_body, bg=self.colors["panel"])
        left_header.grid(row=0, column=0, sticky="w")
        tk.Label(left_header, text="TITANIUM MESH CONSOLE", font=self.f(24, "bold"), fg=self.colors["white"], bg=self.colors["panel"]).pack(anchor="w")
        tk.Label(left_header, text="AALNP desktop view for local status, nearby nodes, navigation, and device display output.", font=self.f(10), fg=self.colors["silver"], bg=self.colors["panel"]).pack(anchor="w", pady=(4, 10))
        badge_row = tk.Frame(left_header, bg=self.colors["panel"])
        badge_row.pack(anchor="w")
        self.status_widgets["connection"] = self._make_badge(badge_row, "WAITING FOR MESH", "muted")
        self.status_widgets["connection"].pack(side=tk.LEFT, padx=(0, 8))
        self.status_widgets["node"] = self._make_badge(badge_row, "NODE: UNKNOWN", "neutral")
        self.status_widgets["node"].pack(side=tk.LEFT, padx=(0, 8))
        self.status_widgets["prefix"] = self._make_badge(badge_row, "PREFIX: /aalnp", "neutral")
        self.status_widgets["prefix"].pack(side=tk.LEFT)

        right_header = tk.Frame(header_body, bg=self.colors["panel"])
        right_header.grid(row=0, column=1, sticky="e")
        self.status_widgets["running"] = self._make_badge(right_header, "STATE: IDLE", "muted")
        self.status_widgets["running"].pack(anchor="e", pady=(0, 8))
        self.status_widgets["feedback"] = self._make_badge(right_header, "CONFIG PATH READY", "neutral")
        self.status_widgets["feedback"].pack(anchor="e")

        content = tk.Frame(shell, bg=self.colors["bg"])
        content.pack(fill=tk.BOTH, expand=True)
        content.grid_columnconfigure(0, weight=11)
        content.grid_columnconfigure(1, weight=9)
        content.grid_rowconfigure(1, weight=1)

        left_col = tk.Frame(content, bg=self.colors["bg"])
        left_col.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        right_col = tk.Frame(content, bg=self.colors["bg"])
        right_col.grid(row=0, column=1, sticky="nsew", padx=(8, 0))

        summary_card, summary_body = self._make_card(left_col, "Runtime Summary", "Live node, queue, position, and broadcast metrics.")
        summary_card.pack(fill=tk.X, pady=(0, 14))
        for row in range(2):
            summary_body.grid_rowconfigure(row, weight=1)
        for col in range(4):
            summary_body.grid_columnconfigure(col, weight=1)
        summary_titles = [
            ("Local Node", 0, 0),
            ("Queue Depth", 0, 1),
            ("Nodes Heard", 0, 2),
            ("TX Delay", 0, 3),
            ("Latitude", 1, 0),
            ("Longitude", 1, 1),
            ("Speed", 1, 2),
            ("Last Broadcast", 1, 3),
        ]
        for title, row, col in summary_titles:
            self.metric_widgets[title] = self._make_metric_tile(summary_body, title, row, col)

        controls_card, controls_body = self._make_card(left_col, "Config Controls", "Update metadata and waypoint settings stored in aalnp_config.json.")
        controls_card.pack(fill=tk.BOTH, expand=True)
        controls_body.grid_columnconfigure(1, weight=1)
        tk.Label(controls_body, text="NODE METADATA", font=self.f(8, "bold"), fg=self.colors["silver"], bg=self.colors["panel"]).grid(row=0, column=0, sticky="w", pady=(0, 6))
        self.metadata_entry = tk.Entry(controls_body)
        self._style_entry(self.metadata_entry)
        self.metadata_entry.grid(row=0, column=1, sticky="ew", padx=(0, 10), pady=(0, 6))
        self._make_button(controls_body, "APPLY METADATA", self._apply_metadata, tone="primary").grid(row=0, column=2, sticky="ew", pady=(0, 6))

        tk.Label(controls_body, text="WAYPOINT NAME", font=self.f(8, "bold"), fg=self.colors["silver"], bg=self.colors["panel"]).grid(row=1, column=0, sticky="w", pady=6)
        self.wp_name_entry = tk.Entry(controls_body)
        self._style_entry(self.wp_name_entry)
        self.wp_name_entry.grid(row=1, column=1, sticky="ew", padx=(0, 10), pady=6)

        tk.Label(controls_body, text="LATITUDE", font=self.f(8, "bold"), fg=self.colors["silver"], bg=self.colors["panel"]).grid(row=2, column=0, sticky="w", pady=6)
        self.wp_lat_entry = tk.Entry(controls_body)
        self._style_entry(self.wp_lat_entry)
        self.wp_lat_entry.grid(row=2, column=1, sticky="ew", padx=(0, 10), pady=6)

        tk.Label(controls_body, text="LONGITUDE", font=self.f(8, "bold"), fg=self.colors["silver"], bg=self.colors["panel"]).grid(row=3, column=0, sticky="w", pady=6)
        self.wp_lon_entry = tk.Entry(controls_body)
        self._style_entry(self.wp_lon_entry)
        self.wp_lon_entry.grid(row=3, column=1, sticky="ew", padx=(0, 10), pady=6)

        button_row = tk.Frame(controls_body, bg=self.colors["panel"])
        button_row.grid(row=4, column=0, columnspan=3, sticky="ew", pady=(10, 0))
        self._make_button(button_row, "SET WAYPOINT", self._apply_waypoint, tone="primary").pack(side=tk.LEFT, padx=(0, 8))
        self._make_button(button_row, "CLEAR WAYPOINT", self._clear_waypoint, tone="secondary").pack(side=tk.LEFT)

        display_card, display_body = self._make_card(right_col, "Navigation & Display", "Waypoint solution, geofence state, metadata, and OLED preview text.")
        display_card.pack(fill=tk.X, pady=(0, 14))
        self.metric_widgets["Navigation"] = self._make_metric_tile(display_body, "Navigation", 0, 0)
        self.metric_widgets["Geofences"] = self._make_metric_tile(display_body, "Geofences", 0, 1)
        display_body.grid_columnconfigure(0, weight=1)
        display_body.grid_columnconfigure(1, weight=1)
        tk.Label(display_body, text="DISPLAY PREVIEW", font=self.f(8, "bold"), fg=self.colors["silver"], bg=self.colors["panel"]).grid(row=1, column=0, columnspan=2, sticky="w", pady=(10, 6))
        self.display_preview = tk.Label(display_body, text="No display output yet.", justify=tk.LEFT, anchor="w", font=self.fm(11, "normal"), fg=self.colors["white"], bg=self.colors["surface"], padx=12, pady=12)
        self.display_preview.grid(row=2, column=0, columnspan=2, sticky="ew")

        nodes_card, nodes_body = self._make_card(right_col, "Nearby Nodes", "Recently heard nodes sorted by most recent reception.")
        nodes_card.pack(fill=tk.BOTH, expand=True)
        self.nodes_text = ScrolledText(
            nodes_body,
            wrap=tk.NONE,
            height=16,
            bg=self.colors["surface"],
            fg=self.colors["white"],
            insertbackground=self.colors["white"],
            relief=tk.FLAT,
            highlightthickness=1,
            highlightbackground=self.colors["border"],
            highlightcolor=self.colors["accent"],
            font=self.fm(10, "normal"),
        )
        self.nodes_text.pack(fill=tk.BOTH, expand=True)
        self.nodes_text.config(state=tk.DISABLED)

        log_card, log_body = self._make_card(shell, "Recent Activity", "Live AALNP and Meshtastic log stream for local inspection.")
        log_card.pack(fill=tk.BOTH, expand=True, pady=(14, 0))
        self.log_text = ScrolledText(
            log_body,
            wrap=tk.WORD,
            height=12,
            bg=self.colors["surface"],
            fg=self.colors["white"],
            insertbackground=self.colors["white"],
            relief=tk.FLAT,
            highlightthickness=1,
            highlightbackground=self.colors["border"],
            highlightcolor=self.colors["accent"],
            font=self.fm(10, "normal"),
        )
        self.log_text.pack(fill=tk.BOTH, expand=True)
        self.log_text.config(state=tk.DISABLED)

    def _set_feedback(self, message, tone="neutral"):
        message = (message or "Ready").replace("\n", " | ")
        self._set_badge(self.status_widgets["feedback"], message[:72], tone)

    def _apply_metadata(self):
        metadata = self.metadata_entry.get().strip()
        response = self.plugin.set_metadata(metadata)
        tone = "active" if response.startswith("Metadata set") or response.startswith("WARN") else "bright"
        self._set_feedback(response, tone)
        self.refresh(force_sync_inputs=True)

    def _apply_waypoint(self):
        name = self.wp_name_entry.get().strip()
        lat = self.wp_lat_entry.get().strip()
        lon = self.wp_lon_entry.get().strip()
        if not name:
            self._set_feedback("Waypoint name is required.", "bright")
            return
        response = self.plugin.set_waypoint(name, lat, lon)
        tone = "active" if response.endswith("set.") else "bright"
        self._set_feedback(response, tone)
        self.refresh(force_sync_inputs=True)

    def _clear_waypoint(self):
        response = self.plugin.clear_waypoint()
        tone = "muted" if response == "Waypoint cleared." else "bright"
        self._set_feedback(response, tone)
        self.refresh(force_sync_inputs=True)

    def _sync_inputs_from_snapshot(self, snapshot):
        metadata = snapshot.get("metadata", "")
        if self.metadata_entry.get().strip() != metadata and self.root.focus_get() is not self.metadata_entry:
            self.metadata_entry.delete(0, tk.END)
            self.metadata_entry.insert(0, metadata)

        waypoint = snapshot.get("waypoint") or {}
        if self.root.focus_get() is not self.wp_name_entry:
            self.wp_name_entry.delete(0, tk.END)
            self.wp_name_entry.insert(0, safe_get(waypoint, 'name', ''))
        if self.root.focus_get() is not self.wp_lat_entry:
            self.wp_lat_entry.delete(0, tk.END)
            lat = safe_get(waypoint, 'latitude', '')
            self.wp_lat_entry.insert(0, "" if lat == '' else str(lat))
        if self.root.focus_get() is not self.wp_lon_entry:
            self.wp_lon_entry.delete(0, tk.END)
            lon = safe_get(waypoint, 'longitude', '')
            self.wp_lon_entry.insert(0, "" if lon == '' else str(lon))

    def refresh(self, force_sync_inputs=False):
        snapshot = self.plugin.get_runtime_snapshot()
        connected = snapshot.get("connected", False)
        running = snapshot.get("running", False)

        self._set_badge(self.status_widgets["connection"], "LINKED TO MESH" if connected else "WAITING FOR MESH", "active" if connected else "muted")
        self._set_badge(self.status_widgets["node"], f"NODE: {snapshot.get('node_id_hex', 'Unknown')}", "bright" if connected else "neutral")
        self._set_badge(self.status_widgets["prefix"], f"PREFIX: {snapshot.get('command_prefix', '/aalnp')}", "neutral")
        self._set_badge(self.status_widgets["running"], f"STATE: {'RUNNING' if running else 'IDLE'}", "active" if running else "muted")

        pos = snapshot.get("position", {}) or {}
        lat = safe_get(pos, 'latitude', safe_get(pos, 'lat'))
        lon = safe_get(pos, 'longitude', safe_get(pos, 'lon'))
        speed = snapshot.get("speed_mps", 0.0) or 0.0
        last_broadcast_age = snapshot.get("last_broadcast_age_s")
        self.metric_widgets["Local Node"].config(text=snapshot.get("node_id_hex", "Unknown"))
        self.metric_widgets["Queue Depth"].config(text=str(snapshot.get("queue_depth", 0)))
        self.metric_widgets["Nodes Heard"].config(text=str(snapshot.get("heard_count", 0)))
        self.metric_widgets["TX Delay"].config(text=f"{snapshot.get('tx_delay_ms', 0)} ms")
        self.metric_widgets["Latitude"].config(text="--" if lat is None else f"{float(lat):.6f}")
        self.metric_widgets["Longitude"].config(text="--" if lon is None else f"{float(lon):.6f}")
        self.metric_widgets["Speed"].config(text=f"{speed:.2f} m/s")
        self.metric_widgets["Last Broadcast"].config(text="--" if last_broadcast_age is None else f"{last_broadcast_age:.0f}s ago")

        nav = snapshot.get("navigation")
        if nav:
            self.metric_widgets["Navigation"].config(text=f"{nav.get('waypoint_name', 'WP')}  {nav.get('distance_m', 0):.0f}m  {nav.get('bearing_deg', 0):.0f}d", fg=self.colors["accent"])
        else:
            self.metric_widgets["Navigation"].config(text="No active waypoint", fg=self.colors["silver"])

        geofences = snapshot.get("geofence_states", {})
        if geofences:
            summary = " | ".join(f"{name}:{state}" for name, state in sorted(geofences.items()))
            self.metric_widgets["Geofences"].config(text=summary[:48], fg=self.colors["white"])
        else:
            self.metric_widgets["Geofences"].config(text="No geofences configured", fg=self.colors["silver"])

        display_lines = snapshot.get("display_lines", []) or ["No display output yet."]
        display_text = "\n".join(display_lines)
        self.display_preview.config(text=display_text, fg=self.colors["accent"] if snapshot.get("position_fix") else self.colors["silver"])

        node_lines = ["NODE ID        AGE    DIST    META"]
        for entry in snapshot.get("other_nodes", [])[:12]:
            age = entry.get("age_s")
            dist = entry.get("distance_m")
            meta = (safe_get(entry, 'meta', '') or '').strip()[:22]
            node_lines.append(
                f"{entry.get('node_id_hex', 'Unknown'):<13}"
                f" {('--' if age is None else f'{age:.0f}s'):>5}"
                f" {('--' if dist is None else f'{dist:.0f}m'):>7}"
                f"  {meta or '-'}"
            )
        nodes_text = "\n".join(node_lines)
        if nodes_text != self.last_nodes_text:
            self._set_text_widget(self.nodes_text, nodes_text)
            self.last_nodes_text = nodes_text

        log_text = "\n".join(self.log_handler.get_lines()[-180:])
        if log_text != self.last_logs_text:
            self._set_text_widget(self.log_text, log_text)
            self.log_text.see(tk.END)
            self.last_logs_text = log_text

        if force_sync_inputs:
            self._sync_inputs_from_snapshot(snapshot)

        if not self._closed:
            self.root.after(1000, self.refresh)

    def _on_close(self):
        self._closed = True
        self.root.quit()
        self.root.destroy()

    def run(self):
        self.refresh(force_sync_inputs=True)
        self.root.mainloop()


# --- Main Execution Guard ---
if __name__ == "__main__":
    # --- Main Function Definition ---
    def main():
        """Main function to parse arguments, set up, and run the plugin."""
        # Setup argument parser
        parser = argparse.ArgumentParser(
            description="AALNPv2.1 - Akita Advanced Location/Navigation Plugin (Akita Engineering)",
            formatter_class=argparse.ArgumentDefaultsHelpFormatter # Show defaults in help message
        )
        # Define command-line arguments
        parser.add_argument("--config", default=DEFAULT_CONFIG_PATH, help="Path to the configuration JSON file.")
        parser.add_argument("--port", default=None, help="Specify the serial port or device address (e.g., /dev/ttyUSB0, COM3, 192.168.x.x)")
        parser.add_argument("--debug", action="store_true", help="Enable DEBUG level logging for AALNPv2.1 and Meshtastic library.")
        parser.add_argument("--no-log", action="store_true", help="Disable GeoJSON logging (overrides config file setting).")
        parser.add_argument("--gui", action="store_true", help="Launch the optional local desktop console for AALNP state and config.")
        args = parser.parse_args() # Parse arguments from command line

        # --- Configure Logging Level ---
        log_level = logging.DEBUG if args.debug else logging.INFO
        logger.setLevel(log_level) # Set level for AALNP logger
        # Configure root logger level only if debugging, otherwise INFO/DEBUG from library can be verbose
        if args.debug:
             logging.getLogger().setLevel(logging.DEBUG) # Set root logger level
             # Ensure handlers also respect the level (basicConfig usually handles this)
             for handler in logging.getLogger().handlers: handler.setLevel(log_level)
             logger.debug("Debug logging enabled for all loggers.")
        else:
             # Keep Meshtastic library logs at INFO or WARNING if not debugging AALNP
             logging.getLogger('meshtastic').setLevel(logging.INFO)


        # --- Initialization ---
        interface = None       # Meshtastic interface object
        aalnp_instance = None  # Plugin instance object
        gui_log_handler = None

        try:
            logger.info("--- Akita Advanced Location/Navigation Plugin (AALNP) v2.1 Starting ---")
            # Log warning if Shapely is missing (affects geo-fencing)
            if not HAS_SHAPELY:
                 logger.warning("Shapely library not installed. Geo-fencing functionality will be disabled.")

            # --- Connect to Meshtastic Device ---
            logger.info(f"Connecting to Meshtastic device ({args.port or 'Auto-detect'})...")
            # Pass debugOut stream to SerialInterface only if debugging is enabled
            debug_out_stream = sys.stdout if args.debug else None
            if args.port:
                # Connect using specified port/address
                interface = meshtastic.serial_interface.SerialInterface(devPath=args.port, debugOut=debug_out_stream)
            else:
                # Attempt auto-detection (will raise error if no device found)
                interface = meshtastic.serial_interface.SerialInterface(debugOut=debug_out_stream)

            logger.info("Meshtastic interface initialized successfully.")

            # --- Create Plugin Instance ---
            aalnp_instance = AALNPv2_Enhanced(interface, config_path=args.config)

            if args.gui:
                if not HAS_TKINTER:
                    raise RuntimeError("Tkinter is not installed. Install the Python Tk package or run without --gui.")
                gui_log_handler = AALNPGuiLogHandler(max_entries=300)
                gui_log_handler.setLevel(logging.DEBUG if args.debug else logging.INFO)
                gui_log_handler.setFormatter(logging.Formatter("%(asctime)s | %(levelname)s | %(message)s", "%H:%M:%S"))
                logging.getLogger().addHandler(gui_log_handler)

            # --- Apply Command-Line Overrides ---
            # Handle --no-log override *after* instance creation and config loading
            if args.no_log:
                 logger.warning("GeoJSON logging explicitly disabled via --no-log argument.")
                 aalnp_instance.config['log_file'] = None # Disable logging path in loaded config

            # --- Register Callbacks ---
            # Register instance methods to handle Meshtastic events
            interface.addReceiveCallback(aalnp_instance.on_receive)
            interface.addConnectionCallback(aalnp_instance.on_connection)
            # Note: The on_connection callback will trigger aalnp_instance.start()

            logger.info("AALNPv2.1 ready. Waiting for Meshtastic connection to establish...")
            logger.info("Press Ctrl+C to exit.")

            # --- Main Loop ---
            if args.gui:
                gui = AALNPDesktopGUI(aalnp_instance, gui_log_handler)
                gui.run()
            else:
                # Keep the main thread alive. Background threads handle operations.
                while True:
                    # Optional: Add checks here if needed (e.g., check interface health)
                    # if not interface or not interface.is_connected:
                    #      logger.warning("Main loop: Interface detected disconnection.")
                         # Rely on on_connection callback to handle shutdown/restart logic
                    time.sleep(5) # Sleep to reduce CPU usage, wake up periodically

        except meshtastic.MeshtasticError as e:
            # Catch specific Meshtastic errors during initialization/connection
            logger.critical(f"Meshtastic initialization/connection error: {e}")
            logger.critical("Ensure device is connected, powered on, not in use by another program, and the port (if specified) is correct.")
            # Provide specific hint if auto-detection failed
            if isinstance(e, meshtastic.error.NoSerialPortFoundError) and not args.port:
                logger.critical("Auto-detection failed. Try specifying the port explicitly using --port.")
            sys.exit(1) # Exit with error code on critical failure
        except KeyboardInterrupt:
            # Handle graceful shutdown on Ctrl+C
            logger.info("\nCtrl+C detected. Shutting down...")
        except Exception as e:
            # Catch any other unexpected errors during setup or the main loop
            logger.critical(f"An unexpected critical error occurred in main: {e}", exc_info=True)
        finally:
            # --- Cleanup ---
            logger.info("--- AALNPv2.1 Shutting Down ---")
            # Stop plugin threads first
            if aalnp_instance:
                aalnp_instance.stop()
            # Close Meshtastic interface connection
            if interface:
                logger.info("Closing Meshtastic interface...")
                interface.close()
            if gui_log_handler:
                logging.getLogger().removeHandler(gui_log_handler)
            logger.info("Shutdown complete.")
            # Ensure all logs are flushed if using file handlers etc.
            logging.shutdown()

    # Execute the main function when the script is run directly
    main()
