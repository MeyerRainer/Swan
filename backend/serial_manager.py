from PyQt6.QtCore import QObject, pyqtSignal, QTimer
import serial.tools.list_ports


class SerialManager(QObject):

    _connected = pyqtSignal()
    _disconnected = pyqtSignal()

    line_received = pyqtSignal(str)
    status_received_signal = pyqtSignal(str)

    error_signal = pyqtSignal(str)

    ports_changed = pyqtSignal(list)


    def __init__(self):

        super().__init__()

        self._serial = None
        self.timer = QTimer()
        self.timer.timeout.connect(self.poll_status)

    @property
    def connected(self):
        return self._serial and self._serial.connected

    def scan_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.ports_changed.emit(ports)
        return ports

    def connect(self, port, baud):
        #self._disconnected()

        self._serial = GCodeSerial(desired_port=port, baud=baud)
        self._serial.set_response_callback(self.on_line_received)

        if self._serial.connect():
            self.timer.start(100)  # 10Hz
            self._connected.emit()
        else:
            self.error_signal.emit("Could not connect")

    def disconnect(self):
        if self._serial:
            self.timer.stop()
            self._serial.disconnect()
            self._serial = None
            self._disconnected.emit()

    def send(self, command):
        if self._serial:
            self._serial.send_command(command)

    def poll_status(self):
        if not self._serial.connected:
            return
        self._serial.send_immediate("?")

    # def on_line_received(self, line):
    #     self.line_received.emit(line)
    #     # Status
    #     if line.startswith("<"):
    #         status, m_pos, w_pos = parse_status(line)
    #         self.status_received_signal.emit(status, m_pos, w_pos)
    #
    #     elif line.startswith("ok"):
    #         pass
    #
    #     elif line.startswith("error"):
    #         pass
    #
    #     elif line.startswith("ALARM"):
    #         pass
    def on_line_received(self, line):
        self.status_received_signal.emit(line)


# class SerialManager(QObject):
#
#     connected = pyqtSignal()
#     disconnected = pyqtSignal()
#     line_received = pyqtSignal(str)
#     status_received = pyqtSignal(str)
#     error = pyqtSignal(str)
#     ports_changed=pyqtSignal(list)
#
#     def __init__(self):
#
#         super().__init__()
#
#         self._serial=None
#         self._poll_timer=QTimer()
#         self._poll_timer.timeout.connect(self.poll_status)
#
#     @property
#     def connected(self):
#         return self._serial is not None and self._serial.connected
#
#     def connect(self, port, baud):
#         self.disconnect()
#         self._serial = GCodeSerial(port, baud)
#
#         self._serial.set_response_callback(self._on_line)
#         if self._serial.connect():
#
#             self._poll_timer.start(100)
#
#             self.connected.emit()
#
#
#
#         else:
#
#             self.error.emit(
#
#                 "Connection failed"
#
#             )