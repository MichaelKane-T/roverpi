import serial
import threading
import time
from collections import deque

PORT = "/dev/serial0"
BAUD = 115200


class ESP32Serial:
    def __init__(self, port=PORT, baudrate=BAUD, timeout=0.1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

        self.ser = None
        self.running = False
        self.thread = None

        # One lock for serial writes
        self._tx_lock = threading.Lock()

        # One lock for message history
        self._history_lock = threading.Lock()
        self._history = deque(maxlen=50)

    def connect(self):
        if self.ser and self.ser.is_open:
            return

        self.ser = serial.Serial(
            self.port,
            self.baudrate,
            timeout=self.timeout
        )

        self.running = True
        time.sleep(2)  # ESP32 may reset when serial opens

    def start_reader(self):
        if self.thread and self.thread.is_alive():
            return

        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def _read_loop(self):
        while self.running:
            try:
                if self.ser and self.ser.is_open and self.ser.in_waiting:
                    msg = self.ser.readline().decode(
                        "utf-8",
                        errors="ignore"
                    ).strip()

                    if msg:
                        with self._history_lock:
                            self._history.append(msg)

                        print(f"[ESP32] {msg}")
                else:
                    time.sleep(0.02)

            except Exception as e:
                print(f"[Serial Read Error] {e}")
                time.sleep(0.2)

    def send(self, message: str) -> bool:
        if not message:
            return False

        if not self.ser or not self.ser.is_open:
            print("[Serial] Not connected")
            return False

        packet = (message.strip() + "\n").encode("utf-8")

        try:
            # This is the important part:
            # only one thread can write to UART at a time.
            with self._tx_lock:
                self.ser.write(packet)
                self.ser.flush()

            return True

        except Exception as e:
            print(f"[Serial Write Error] {e}")
            return False

    def get_latest_message(self) -> str:
        with self._history_lock:
            return self._history[-1] if self._history else ""

    def get_history(self) -> list:
        with self._history_lock:
            return list(self._history)

    def has_seen(self, text: str) -> bool:
        with self._history_lock:
            return any(text in msg for msg in self._history)

    def clear_history(self):
        with self._history_lock:
            self._history.clear()

    def close(self):
        self.running = False

        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)

        with self._tx_lock:
            if self.ser and self.ser.is_open:
                self.ser.close()