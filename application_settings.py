from PyQt6.QtCore import QSettings


class ApplicationSettings:
    def __init__(self):

        self._settings = QSettings("MyCompany", "MyRobot")

    @property
    def program_directory(self) -> str:
        return self._settings.value("program/last_directory", "")

    @program_directory.setter
    def program_directory(self, value: str):
        self._settings.setValue("program/last_directory", value)

    @property
    def calibration_image_directory(self) -> str:
        return self._settings.value("vision/last_directory", "")

    @calibration_image_directory.setter
    def calibration_image_directory(self, value: str):
        self._settings.setValue("vision/last_directory", value)
