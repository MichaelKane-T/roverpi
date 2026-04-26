import serial
import threading
import time
from collections import deque

PORT = "/dev/serial0"
BAUD = 115200

class ESP32Serial:
    def __init__(self, port=PORT, baudrate=BAUD, timeout=0.1):
        self.port     = port
        self.baudrate = baudrate
        self.timeout  = timeout
        self.ser      = None
        self.running  = False
        self._lock    = threading.Lock()
        self.thread   = None
        self._history = deque(maxlen=50)

    def connect(self):
        if self.ser and self.ser.is_open:
            return
        self.ser     = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        self.running = True
        time.sleep(2)

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
                        with self._lock:
                            self._history.append(msg)
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

    def get_latest_message(self) -> str:
        with self._lock:
            return self._history[-1] if self._history else ""

    def get_history(self) -> list:
        with self._lock:
            return list(self._history)

    def has_seen(self, text: str) -> bool:
        with self._lock:
            return any(text in msg for msg in self._history)

    def clear_history(self):
        with self._lock:
            self._history.clear()

    def close(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()