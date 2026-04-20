import serial
import threading
import time


class ESP32Serial:
    def __init__(self, port="/dev/serial0", baudrate=115200, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.running = False
        self.latest_message = ""
        self.lock = threading.Lock()

    def connect(self):
        self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        time.sleep(2)
        self.running = True

    def start_reader(self):
        thread = threading.Thread(target=self._read_loop, daemon=True)
        thread.start()

    def _read_loop(self):
        while self.running and self.ser:
            try:
                if self.ser.in_waiting:
                    msg = self.ser.readline().decode("utf-8", errors="ignore").strip()
                    with self.lock:
                        self.latest_message = msg
                    print(f"[ESP32] {msg}")
            except Exception as e:
                print(f"[Serial Read Error] {e}")
                time.sleep(0.2)

    def send(self, message: str):
        if self.ser:
            self.ser.write((message.strip() + "\n").encode("utf-8"))

    def get_latest_message(self):
        with self.lock:
            return self.latest_message

    def close(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()