import serial
import threading
import time

PORT = "/dev/serial0"
BAUD = 115200


class ESP32Serial:
    def __init__(self, port=PORT, baudrate=BAUD, timeout=0.1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.running = False
        self.latest_message = ""
        self.lock = threading.Lock()
        self.thread = None

    def connect(self):
        if self.ser and self.ser.is_open:
            return

        self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        time.sleep(2)
        self.running = True

    def start_reader(self):
        if self.thread and self.thread.is_alive():
            return

        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def _read_loop(self):
        while self.running:
            try:
                if self.ser and self.ser.in_waiting:
                    msg = self.ser.readline().decode("utf-8", errors="ignore").strip()
                    if msg:
                        with self.lock:
                            self.latest_message = msg
                        print(f"[ESP32] {msg}")
                else:
                    time.sleep(0.02)
            except Exception as e:
                print(f"[Serial Read Error] {e}")
                time.sleep(0.2)

    def send(self, message: str):
        if not self.ser or not self.ser.is_open:
            print("[Serial] Not connected")
            return False

        self.ser.write((message.strip() + "\n").encode("utf-8"))
        return True

    def get_latest_message(self):
        with self.lock:
            return self.latest_message

    def close(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()