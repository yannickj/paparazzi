from PyQt5 import QtCore, QtWidgets, QtGui
from ui.strip_ui import Ui_Strip
from enum import Enum
from collections import namedtuple

MAX_RETRY = 9
RESEND = 3

class ButtonState(Enum):
    DISABLE_ACKWAIT = 0
    ENABLE_ACKWAIT = 1
    ENABLED = 2


class State(Enum):
    OK = 0
    UNKNOWN = 1
    NOK = 2


class Strip(QtWidgets.QWidget):

    States = namedtuple('States', ['rc', 'power'])

    def __init__(self, ac_id, name, hue, setting_index):
        QtWidgets.QWidget.__init__(self)
        self.ui = Ui_Strip()
        self.ui.setupUi(self)
        self.ac_id = ac_id
        self.setting_index = setting_index
        self.ui.name_label.setText(name)
        self.button_state = ButtonState.ENABLED
        self.rc_state = State.UNKNOWN
        self.power_state = State.UNKNOWN
        self.rc_desired_state = State.UNKNOWN
        self.power_desired_state = State.UNKNOWN
        style = "QLabel {{ background-color : hsv({}, 100, 255); color : black; font-weight: bold;}}".format(hue)
        self.ui.name_label.setStyleSheet(style)
        self.set_RC_state(self.rc_state)
        self.set_POWER_state(self.power_state)
        self.default_button_stylesheet = self.ui.select_button.styleSheet()
        self.default_frame_stylesheet = self.ui.frame.styleSheet()
        self.ui.ignore_checkBox.stateChanged.connect(self.ignore)
        self.retry_counter = 0

    def set_RC_state(self, state):
        self.rc_state = state
        if state == State.OK:
            self.ui.rc_label.setText("RC OK")
            self.ui.rc_label.setStyleSheet("QLabel { background-color : green; color : white; font-weight: bold;}")
        elif state == State.UNKNOWN:
            self.ui.rc_label.setText("RC Unknown")
            self.ui.rc_label.setStyleSheet("QLabel { background-color : grey; color : black; font-weight: bold;}")
        elif state == State.NOK:
            self.ui.rc_label.setText("RC LOST")
            self.ui.rc_label.setStyleSheet("QLabel { background-color : red; color : white; font-weight: bold;}")

    def set_POWER_state(self, state):
        self.power_state = state
        if state == State.OK:
            self.ui.power_label.setText("POWER ON")
            self.ui.power_label.setStyleSheet("QLabel { background-color : green; color : white; font-weight: bold;}")
        elif state == State.UNKNOWN:
            self.ui.power_label.setText("POWER Unknown")
            self.ui.power_label.setStyleSheet("QLabel { background-color : grey; color : black; font-weight: bold;}")
        elif state == State.NOK:
            self.ui.power_label.setText("POWER OFF")
            self.ui.power_label.setStyleSheet("QLabel { background-color : red; color : white; font-weight: bold;}")

    def set_button_state(self, state):
        self.button_state = state
        if state == ButtonState.DISABLE_ACKWAIT:    # wait for power off ack
            self.ui.select_button.setEnabled(False)
            self.ui.select_button.setStyleSheet(
                "QPushButton { background-color : hsv(0, 50, 200); color : white; font-weight: bold;}")
        elif state == ButtonState.ENABLE_ACKWAIT:   # wait for power on ack
            self.ui.select_button.setEnabled(False)
            self.ui.select_button.setStyleSheet(
                "QPushButton { background-color : hsv(200, 100, 200); color : white; font-weight: bold;}")
        elif state == ButtonState.ENABLED:          # enable button (ack received)
            self.ui.select_button.setEnabled(True)
            self.ui.select_button.setStyleSheet(self.default_button_stylesheet)

    def set_selected(self, selected):
        if selected:
            self.ui.frame.setStyleSheet("QFrame#frame {border: 4px solid green; border-radius: 10px; padding: 2px;}")
        else:
            self.ui.frame.setStyleSheet(self.default_frame_stylesheet)

    def set_desired_power_state(self, state):
        self.power_desired_state = state

    def is_ignored(self):
        return self.ui.ignore_checkBox.checkState() == QtCore.Qt.Checked

    def ignore(self, state):
        if state == QtCore.Qt.Checked:
            self.ui.select_button.setEnabled(False)
        else:
            self.set_button_state(self.button_state)


