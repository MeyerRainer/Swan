""" Class for serial communication. Partly AI-generated.

Author: Rainer Meyer, r.meyer494@gmail.com
"""

from PyQt6.QtCore import QObject, pyqtSignal, QTimer
import serial
import serial.tools.list_ports
import time
import threading
from collections import deque

# self.app_context.serial.line_received.connect(self.process_line)

class GCSerial(QObject):

    connected = pyqtSignal(int, str)  # Baud, port
    disconnected = pyqtSignal()
    serial_connect = pyqtSignal(bool)
    ports_refreshed = pyqtSignal(list)
    write_terminal = pyqtSignal(str)
    status_received = pyqtSignal(str)
    # error_received = pyqtSignal(str)
    # alarm_received = pyqtSignal(str)
    # line_received = pyqtSignal(str)

    def __init__(self):

        super().__init__()

        self._ser = None
        self._running = False

        self._tx_thread = None
        self._rx_thread = None

        self._queue = deque()
        self._pending_lengths = deque()
        self._bytes_in_flight = 0

        # Defaults before user has changed the combo box.
        self.baud: int = 115200
        self.port: str = "Auto"

        self._queue_lock = threading.Lock()
        self._serial_lock = threading.Lock()

        self._state_cv = threading.Condition()

        self._poll_timer = QTimer()

        self._poll_timer.timeout.connect(self.poll_status)

    @property
    def is_connected(self):
        if self._ser:
            return self._ser.is_open
        return False

    def connect(self) -> bool:

        # bauds = [9600, 19200, 28800, 38400, 57600, 76800, 115200, 230400, 460800, 576000, 921600]

        if self.port == "Auto":
            ports = self.scan_ports()
        else:
            ports = [self.port]

        for port in ports:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.5, write_timeout=0.5)
            if self._ser:
                self.connected.emit(self.baud, port)
                self.write_terminal.emit(f"Connected to {port} at {self.baud} baud.")
                break

        if not self._ser:
            self.write_terminal.emit("Failed to connect.")
            return False

        # Successfully connected
        time.sleep(2)
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()
        self._running = True

        self._tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)

        self._tx_thread.start()
        self._rx_thread.start()

        self._poll_timer.start(100)

        return True

    def disconnect(self):
        self._running = False
        self._poll_timer.stop()

        # Wake up the TX thread if it's waiting on the Condition Variable
        with self._state_cv:
            self._state_cv.notify_all()

        # Close the port first! This forces the blocking rx read to immediately crash and exit
        if self._ser and self._ser.is_open:
            with self._serial_lock:
                self._ser.close()

        if self._tx_thread:
            self._tx_thread.join()
        if self._rx_thread:
            self._rx_thread.join()

        self.write_terminal.emit(f"Disconnected.")
        self.disconnected.emit()

    def toggle_connection(self):
        if self.is_connected:
            self.disconnect()
        else:
            self.connect()

    def scan_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.ports_refreshed.emit(ports)
        return ports

    def refresh_ports(self):
        ports: list = self.scan_ports()
        if not ports:
            self.write_terminal.emit("No ports found")
            return
        else:
            self.ports_refreshed.emit(ports)
            self.write_terminal.emit("Ports refreshed.")

    def send(self, cmd):
        with self._queue_lock:
            self._queue.append(cmd.strip())

        with self._state_cv:
            self._state_cv.notify()

    def send_realtime(self, cmd):
        with self._serial_lock:
            n = self._ser.write(cmd.encode("ascii"))
            #self._ser.flush()

    def _tx_loop(self):
        while self._running:
            with self._state_cv:
                self._state_cv.wait_for(lambda: (self._queue and self._can_send()) or not self._running)

            if not self._running:
                break

            with self._queue_lock:
                cmd = self._queue.popleft()

            self._send(cmd)

    def _can_send(self) -> bool:
        """ Checks if next command in buffer can be sent to serial.
        :return: True if next command in deque can be sent.
        """
        # Buffer empty
        if not self._queue:
            return False

        cmd = self._queue[0]
        n = len(cmd) + 1  # Newline counts
        can_send = self._bytes_in_flight + n < 117
        return can_send

    def _send(self, cmd):
        """ Send to serial
        :param cmd: command, string
        :return:
        """
        n = len(cmd) + 1

        with self._serial_lock:
            self._ser.write((cmd + '\n').encode("ascii"))
            # self._ser.flush()
            #print(f"Sent: {cmd}")

        with self._state_cv:
            self._bytes_in_flight += n
            self._pending_lengths.append(n)

            # TODO: Check this
            self._state_cv.notify()

    def _rx_loop(self):
        while self._running:
            try:
                line = self._ser.readline()
                if not line:
                    continue
                line = line.decode('ascii').strip()

            except (serial.SerialException, OSError) as e:
                self.write_terminal.emit(f"Serial connection lost: {e}")
                break

            self._process_line(line)

    def _process_line(self, line):

        if line.startswith('ok'):
            self._ack()
            return

        if line.startswith('error'):
            self._ack()
            # self.error_received.emit(line)
            return

        if line.startswith('<'):
            self.status_received.emit(line)
            return

        if line.startswith("ALARM"):
            # self.alarm_received.emit(line)
            return

    def _ack(self):
        with self._state_cv:
            if self._pending_lengths:
                n = self._pending_lengths.popleft()
                self._bytes_in_flight -= n
            self._state_cv.notify()

    def poll_status(self):
        if not self._running or not self._ser or not self._ser.is_open:
            return
        self.send_realtime("?")

    def on_baud_receive(self, baud: str):
        self.baud = int(baud)

    def on_port_receive(self, port: str):
        self.port = port
