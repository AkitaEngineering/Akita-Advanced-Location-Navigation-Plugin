import meshtastic
import time
import json
import threading
import os
import argparse
from meshtastic.util import get_lora_config

class AALNP:
    def __init__(self, interface, log_file="location_log.json", location_interval=60):
        self.interface = interface
        self.location_data = {}
        self.location_thread = None
        self.user_id = interface.meshtastic.getMyNodeInfo()['num']
        self.log_file = log_file
        self.location_interval = location_interval
        self.lora_config = get_lora_config(interface.meshtastic)

    def start_location_broadcast(self):
        self.location_thread = threading.Thread(target=self._send_location_broadcast)
        self.location_thread.start()
        print("Location broadcast started.")

    def stop_location_broadcast(self):
        if self.location_thread:
            self.location_thread.join(timeout=2)
            print("Location broadcast stopped.")

    def _send_location_broadcast(self):
        while True:
            try:
                gps = self.interface.meshtastic.getGps()
                if gps:
                    self.location_data = {
                        "type": "location",
                        "user_id": self.user_id,
                        "gps_location": gps,
                        "timestamp": time.time()
                    }
                    self.interface.sendData(self.location_data, portNum=meshtastic.constants.DATA_APP)
                    time.sleep(self.location_interval)
                else:
                    time.sleep(10)
            except Exception as e:
                print(f"Error sending location broadcast: {e}")

    def handle_incoming(self, packet, interface):
        if packet['decoded']['portNum'] == meshtastic.constants.DATA_APP:
            decoded = packet['decoded']['payload']
            if decoded.get("type") == "location":
                print(f"Location received: {decoded}")
                self.log_location(decoded) #UI integration point.

    def log_location(self, data):
        try:
            if not os.path.exists(self.log_file):
                with open(self.log_file, 'w') as f:
                    f.write('[]')

            with open(self.log_file, 'r+') as f:
                file_data = json.load(f)
                file_data.append(data)
                f.seek(0)
                json.dump(file_data, f, indent=4)
        except Exception as e:
            print(f"Error logging location data: {e}")

    def onConnection(self, interface, connected):
        if connected:
            print("AALNP: Meshtastic connected.")
            self.start_location_broadcast()
