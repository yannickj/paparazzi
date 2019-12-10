#!/usr/bin/python3
import sys
import argparse
from typing import List
from PyQt5 import QtCore, QtWidgets, QtGui
from strip import Strip, ButtonState, State, MAX_RETRY, RESEND
from ui.selector_ui import Ui_RCSelector
import traceback
import colorsys
from enum import Enum
from os import getenv, path
from lxml import etree
from urllib.parse import urlparse, urlencode
import threading
import time

PPRZ_HOME = getenv("PAPARAZZI_HOME")
if PPRZ_HOME is None:
    raise Exception("PAPARAZZI_HOME env var is not set !")
sys.path.append(PPRZ_HOME + "/var/lib/python")
sys.path.append(PPRZ_HOME + "/sw/lib/python")
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage
from pprz_connect import PprzConnect

HUES = {"blue": 240,
        "red": 0,
        "green": 120,
        "pink": 310,
        "purple": 280,
        "yellow": 60,
        "orange": 30}


class ConfigError(Exception):
    def __init__(self, message):
        self.message = message


class SelectState(Enum):
    IDLE = 0
    TURNING_OFF = 1
    TURNING_ON = 2


class Selector(Ui_RCSelector, QtCore.QObject):

    new_ac_signal = QtCore.pyqtSignal(tuple)
    ac_states_updated = QtCore.pyqtSignal()
    reset_signal = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        Ui_RCSelector.__init__(self)
        QtCore.QObject.__init__(self)
        self.strips = {}    # type: Dict[int, Strip]
        self.select_state = SelectState.IDLE
        self.connect = PprzConnect(notify=self.new_ac)
        self.connect.ivy.subscribe(self.dl_values_cb, PprzMessage('ground', 'DL_VALUES'))
        self.connect.ivy.subscribe(self.fbw_cb, PprzMessage('ground', 'FLY_BY_WIRE'))
        self.operation_completed = True
        self.running = True
        self.setting_request_thread = threading.Thread(target=self.poll_switch_settings)
        self.setting_request_thread.start()

    def built(self):
        self.none_button.clicked.connect(self.select_none)
        self.force_button.clicked.connect(self.force_select)
        self.new_ac_signal.connect(lambda infos: self.add_strip(*infos))
        self.ac_states_updated.connect(self.update_states)
        self.reset_signal.connect(self.reset_buttons)
        # self.add_strip(1, "Pelican", 40)
        # self.add_strip(2, "Pierre", 210)
        # self.add_strip(3, "Sloubi", 150)
        pass

    def get_strips(self):
        return filter(lambda s: not s.is_ignored(), self.strips.values())

    @staticmethod
    def get_color(color):
        if color[0] == "#":
            if len(color) == 13:
                r, g, b = int(color[1:5], 16)/0xffff, int(color[5:9], 16)/0xffff, int(color[9:13], 16)/0xffff
                h, s, v = colorsys.rgb_to_hsv(r, g, b)
                h, s, v = h*360, s*255, v*255
                return h, s, v
            elif len(color) == 7:
                r, g, b = int(color[1:3], 16)/0xff, int(color[3:5], 16)/0xff, int(color[5:7], 16)/0xff
                h, s, v = colorsys.rgb_to_hsv(r, g, b)
                h, s, v = h*360, s*255, v*255
                return h, s, v
            else:
                return HUES.get("white", 0), 255, 255
        else:
            return HUES.get(color, 0), 255, 255

    def new_ac(self, config):
        h, s, v = self.get_color(config.color)
        self.new_ac_signal.emit((h, config))
        pass

    def add_strip(self, hue, config):
        try:
            setting_index = self.get_power_switch_setting_index(config.settings)
            strip = Strip(config.id, config.name, hue, setting_index)
            strip.ui.select_button.clicked.connect(lambda: self.select_aircraft(config.id))
            self.strips[config.id] = strip
            self.scroll_layout.insertWidget(self.scroll_layout.count() - 1, strip)
        except ConfigError as e:
            print("ConfigError for aircraft {} : {}".format(config.name, e.message))

    @staticmethod
    def get_power_switch_setting_index(settings_url):
        def get_settings_rec(node):
            settings = node.findall("dl_setting")
            for dl_settings in node.findall("dl_settings"):
                settings.extend(get_settings_rec(dl_settings))
            return settings

        p = urlparse(settings_url)
        settings_path = path.abspath(path.join(p.netloc, p.path))
        root = etree.parse(settings_path).getroot()

        dls = list(map(lambda s: s.get("var"), get_settings_rec(root)))
        #print("nb settings : ", len(dls))
        try:
            i = dls.index("autopilot.power_switch")
            return i
        except ValueError:
            raise ConfigError("No setting named autopilot.power_switch !")

    def select_aircraft(self, ac_id):
        self.operation_completed = False
        self.select_state = SelectState.TURNING_OFF
        for strip in self.get_strips():
            if ac_id == strip.ac_id:
                strip.set_button_state(ButtonState.ENABLE_ACKWAIT)
                strip.set_desired_power_state(State.OK)
                strip.set_selected(True)
                # do not enable it now : wait for others to be disabled
            else:
                #strip.set_desired_power_state(State.NOK)
                self.change_switch_state(strip, State.NOK)
                strip.set_button_state(ButtonState.DISABLE_ACKWAIT)
                strip.set_selected(False)

    def dl_values_cb(self, sender, msg):
        if msg.ac_id in self.strips.keys():
            ps_index = self.strips[msg.ac_id].setting_index
            try:
                ps_value = msg.values[ps_index]
                #print(ps_value)
                if ps_value == '?':
                    self.strips[msg.ac_id].set_POWER_state(State.UNKNOWN)
                elif int(float(ps_value)) == 0:
                    self.strips[msg.ac_id].set_POWER_state(State.NOK)
                else:
                    self.strips[msg.ac_id].set_POWER_state(State.OK)
                self.ac_states_updated.emit()
            except IndexError:
                #print("power switch index out of range of DL_VALUES...")
                pass

    def fbw_cb(self, sender, msg):
        if msg.ac_id in self.strips.keys():
            if msg.rc_status == "OK":
                self.strips[msg.ac_id].set_RC_state(State.OK)
            elif msg.rc_status == "LOST" or msg.rc_status == "NONE":
                self.strips[msg.ac_id].set_RC_state(State.NOK)
            self.ac_states_updated.emit()

    def update_states(self):
        if self.select_state == SelectState.TURNING_OFF:
            turning_off_finished = True
            for strip in self.get_strips():
                if strip.power_desired_state == State.NOK and strip.power_state != State.NOK:
                    turning_off_finished = False

            if turning_off_finished:
                # everything is now turned off. Lets turn some stuff on, and wait for it in the next state !
                self.select_state = SelectState.TURNING_ON
                for strip in self.get_strips():
                    if strip.power_desired_state == State.OK and strip.power_state != State.OK:
                        self.change_switch_state(strip, State.OK)
                        pass

        elif self.select_state == SelectState.TURNING_ON:
            turning_on_finished = True
            for strip in self.get_strips():
                if strip.power_desired_state == State.OK and strip.power_state != State.OK:
                    turning_on_finished = False
            if turning_on_finished:
                self.select_state = SelectState.IDLE
                for strip in self.get_strips():
                    strip.set_button_state(ButtonState.ENABLED)     # enable the buttons back
                self.operation_completed = True
            else:
                pass    # resend the order if you want to !

    def force_select(self):
        # send every orders, not matter if multiples are on at the same time
        self.operation_completed = False
        for strip in self.get_strips():
            if strip.power_desired_state == State.OK:
                self.change_switch_state(strip, State.OK)
                strip.set_button_state(ButtonState.ENABLED)
            elif strip.power_desired_state == State.NOK:
                self.change_switch_state(strip, State.NOK)
                strip.set_button_state(ButtonState.ENABLED)

    def select_none(self):
        self.operation_completed = False
        for strip in self.get_strips():
            self.change_switch_state(strip, State.NOK)
            strip.set_button_state(ButtonState.DISABLE_ACKWAIT)
            strip.set_selected(False)
        self.select_state = SelectState.TURNING_OFF

    def reset_buttons(self):
        for strip in self.get_strips():
            strip.set_button_state(ButtonState.ENABLED)

    def poll_switch_settings(self):
        while self.running:
            if not self.operation_completed:
                self.operation_completed = True
                for strip in self.get_strips():
                    if strip.power_desired_state != strip.power_state:
                        if strip.retry_counter == MAX_RETRY:
                            # End retry sequence
                            strip.set_POWER_state(State.UNKNOWN)
                        else:
                            # keep polling
                            self.operation_completed = False
                            msg = PprzMessage("ground", "GET_DL_SETTING")
                            msg['ac_id'] = strip.ac_id
                            msg['index'] = strip.setting_index
                            self.connect.ivy.send(msg)
                            strip.retry_counter += 1
                            if strip.retry_counter % RESEND == 0:
                                # Resend command
                                self.change_switch_state(strip, strip.power_desired_state)
                if self.operation_completed:
                    for strip in self.strips.values():
                        strip.retry_counter = 0
                    self.reset_signal.emit()
            time.sleep(1)

    def change_switch_state(self, strip, state):
        msg = PprzMessage("ground", "DL_SETTING")
        msg['ac_id'] = strip.ac_id
        msg['index'] = strip.setting_index
        if state == State.NOK:
            strip.set_desired_power_state(State.NOK)
            msg['value'] = 0
        elif state == State.OK:
            strip.set_desired_power_state(State.OK)
            msg['value'] = 1
        self.connect.ivy.send(msg)

    def closing(self):
        self.running = False
        self.setting_request_thread.join()
        self.connect.shutdown()
        #print("bye !")


def main():
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    rcselector = Selector()
    app.aboutToQuit.connect(rcselector.closing)
    rcselector.setupUi(MainWindow)
    rcselector.built()
    MainWindow.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
