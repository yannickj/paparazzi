from PyQt5 import QtCore, QtWidgets, QtGui
from collections import namedtuple
from ui.mainwindow import Ui_MainWindow
import traceback
from time import sleep

import sys
from os import path, getenv

# if PAPARAZZI_SRC or PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_HOME + "/var/lib/python") # pprzlink

from pprzlink.message import PprzMessage
from pprz_connect import PprzConnect, PprzConfig

class Mark():
    '''
    Store mark information
    '''
    def __init__(self, name):
        self.name = name
        self.wp_id = {}
        self.lat = 0.
        self.lon = 0.
        self.alt = 0.

    def set_pos(self, lat, lon, alt):
        ''' set pos, TODO filtering '''
        self.lat = lat
        self.lon = lon
        self.alt = alt

class Tracker(Ui_MainWindow):
  
    def __init__(self, parent=None):
        Ui_MainWindow.__init__(self)

    def built(self):

        ''' get aircraft config '''
        def connect_cb(conf):
            self.comboBox.addItem(conf.name)
            print(conf)

        ''' create connect object, it will start Ivy interface '''
        self.connect = PprzConnect(notify=connect_cb)

        ''' bind to MARK message '''
        def mark_cb(ac_id, msg):
            print('from ',ac_id,':',msg)
        self.connect.ivy.subscribe(mark_cb,PprzMessage("telemetry", "MARK"))

    def closing(self):
        ''' shutdown Ivy and window '''
        self.connect.shutdown()

    def move_wp(self, ac_id, mark):
        ''' move waypoint corresponding to a selected aircraft and mark '''
        msg = PprzMessage("ground", "MOVE_WAYPOINT")
        msg['ac_id'] = ac_id
        msg['wp_id'] = mark.wp_id[ac_id]
        msg['lat'] = mark.lat
        msg['long'] = mark.lon
        mag['alt'] = mark.alt
        self.connect.ivy.send(msg)

