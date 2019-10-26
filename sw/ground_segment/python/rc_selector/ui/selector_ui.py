# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui/selector.ui'
#
# Created by: PyQt5 UI code generator 5.11.3
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_RCSelector(object):
    def setupUi(self, RCSelector):
        RCSelector.setObjectName("RCSelector")
        RCSelector.resize(429, 600)
        self.centralwidget = QtWidgets.QWidget(RCSelector)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName("verticalLayout")
        self.scrollArea = QtWidgets.QScrollArea(self.centralwidget)
        self.scrollArea.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)
        self.scrollArea.setWidgetResizable(True)
        self.scrollArea.setObjectName("scrollArea")
        self.scrollAreaWidgetContents = QtWidgets.QWidget()
        self.scrollAreaWidgetContents.setGeometry(QtCore.QRect(0, 0, 409, 503))
        self.scrollAreaWidgetContents.setObjectName("scrollAreaWidgetContents")
        self.scroll_layout = QtWidgets.QVBoxLayout(self.scrollAreaWidgetContents)
        self.scroll_layout.setObjectName("scroll_layout")
        spacerItem = QtWidgets.QSpacerItem(20, 482, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.scroll_layout.addItem(spacerItem)
        self.scrollArea.setWidget(self.scrollAreaWidgetContents)
        self.verticalLayout.addWidget(self.scrollArea)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.force_button = QtWidgets.QPushButton(self.centralwidget)
        self.force_button.setObjectName("force_button")
        self.horizontalLayout.addWidget(self.force_button)
        self.none_button = QtWidgets.QPushButton(self.centralwidget)
        self.none_button.setObjectName("none_button")
        self.horizontalLayout.addWidget(self.none_button)
        self.verticalLayout.addLayout(self.horizontalLayout)
        RCSelector.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(RCSelector)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 429, 22))
        self.menubar.setObjectName("menubar")
        RCSelector.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(RCSelector)
        self.statusbar.setObjectName("statusbar")
        RCSelector.setStatusBar(self.statusbar)

        self.retranslateUi(RCSelector)
        QtCore.QMetaObject.connectSlotsByName(RCSelector)

    def retranslateUi(self, RCSelector):
        _translate = QtCore.QCoreApplication.translate
        RCSelector.setWindowTitle(_translate("RCSelector", "RC Selector"))
        self.force_button.setText(_translate("RCSelector", "Force"))
        self.none_button.setText(_translate("RCSelector", "None"))

