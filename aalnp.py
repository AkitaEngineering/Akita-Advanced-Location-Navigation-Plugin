import meshtastic
import time
import json
import threading
import os
import argparse
import logging
from meshtastic.util import get_lora_config
import queue

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class AALNP:
    def __init__(self, interface, log_file="location_log.json", location_interval=60):
        self.interface = interface
        self.location_data = {}
        self.location_thread = None
        self.user_id = interface.meshtastic.getMyNodeInfo()['num']
        self.log_file = log_file
        self.location_interval = location_interval
        self.lora_config = get_lora_config(interface.meshtastic)
        self.running = True
        self.message_queue = queue.Queue()
        self.publish_thread = threading.Thread(target=self._publish_from_queue)
        self.publish_thread.daemon = True
        self.publish_thread.start()

    def start_location_broadcast(self):
        self.location_thread = threading.Thread(target=self._send_location_broadcast)
        self.location_thread.start()
        logging.info("Location broadcast started.")

    def stop_location_broadcast(self):
        self.running = False
        if self.location_thread:
            self.location_thread.join()
            logging.info("Location broadcast stopped.")

    def _send_location_broadcast(self):
        while self.running:
            try:
                gps = self.interface.meshtastic.getGps()
                if gps and gps.get('latitude') and gps.get('longitude'):
                    self.location_data = {
                        "type": "location",
                        "user_id": self.user_id,
                        "gps_location": gps,
                        "timestamp": time.time()
                    }
                    self.message_queue.put(self.location_data)
                    time.sleep(self.location_interval)
                else:
                    logging.warning("GPS data invalid, waiting 10 seconds.")
                    time.sleep(10)
            except Exception as e:
                logging.error(f"Error sending location broadcast: {e}")
                time.sleep(10)

    def _publish_from_queue(self):
        while self.running:
            try:
                message = self.message_queue.get(timeout=1)
                self.interface.sendData(message, portNum=meshtastic.constants.DATA_APP)
                time.sleep(self.lora_config.tx_delay / 1000)  # Respect TX delay
            except queue.Empty:
                pass
            except Exception as e:
                logging.error(f"Error in publish thread: {e}")

    def handle_incoming(self, packet, interface):
        if packet['decoded']['portNum'] == meshtastic.constants.DATA_APP:
            decoded = packet['decoded']['payload']
            if decoded.get("type") == "location":
                logging.info(f"Location received: {decoded}")
                self.log_location(decoded)

    def log_location(self, data):
        try:
            if not os.path.exists(self.log_file):
                with open(self.log_file, 'w') as f:
                    f.write('[]')

            with open(self.log_file, 'r+') as f:
                try:
                    file_data = json.load(f)
                except json.JSONDecodeError:
                    file_data = []

                file_data.append(data)
                f.seek(0)
                json.dump(file_data, f, indent=4)
        except Exception as e:
            logging.error(f"Error logging location data: {e}")

    def onConnection(self, interface, connected):
        if connected:
            logging.info("AALNP: Meshtastic connected.")
            self.start_location_broadcast()
        else:
            logging.info("AALNP: Meshtastic disconnected.")
            self.stop_location_broadcast()

def onReceive(packet, interface):
    aalnp.handle_incoming(packet, interface)

def onConnection(interface, connected):
    aalnp.onConnection(interface, connected)

def main():
    parser = argparse.ArgumentParser(description="Akita Advanced Location/Navigation Plugin")
    parser.add_argument("--log", default="location_log.json", help="Log file name")
    parser.add_argument("--interval", type=int, default=60, help="Location broadcast interval in seconds")
    args = parser.parse_args()

    interface = meshtastic.SerialInterface()
    global aalnp
    aalnp = AALNP(interface, args.log, args.interval)
    interface.addReceiveCallback(onReceive)
    interface.addConnectionCallback(onConnection)

    try:
        while aalnp.running:
            time.sleep(1)
    except KeyboardInterrupt:
        logging.info("AALNP: Stopping...")
        aalnp.stop_location_broadcast()
        logging.info("AALNP: Stopped")

if __name__ == '__main__':
    main()
