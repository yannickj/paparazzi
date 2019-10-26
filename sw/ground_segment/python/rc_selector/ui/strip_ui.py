# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui/strip.ui'
#
# Created by: PyQt5 UI code generator 5.11.3
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_Strip(object):
    def setupUi(self, Strip):
        Strip.setObjectName("Strip")
        Strip.resize(637, 152)
        self.horizontalLayout = QtWidgets.QHBoxLayout(Strip)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.frame = QtWidgets.QFrame(Strip)
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.frame)
        self.verticalLayout.setObjectName("verticalLayout")
        self.name_label = QtWidgets.QLabel(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.name_label.sizePolicy().hasHeightForWidth())
        self.name_label.setSizePolicy(sizePolicy)
        self.name_label.setObjectName("name_label")
        self.verticalLayout.addWidget(self.name_label)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.select_button = QtWidgets.QPushButton(self.frame)
        self.select_button.setObjectName("select_button")
        self.horizontalLayout_2.addWidget(self.select_button)
        self.rc_label = QtWidgets.QLabel(self.frame)
        self.rc_label.setAutoFillBackground(True)
        self.rc_label.setAlignment(QtCore.Qt.AlignCenter)
        self.rc_label.setObjectName("rc_label")
        self.horizontalLayout_2.addWidget(self.rc_label)
        self.power_label = QtWidgets.QLabel(self.frame)
        self.power_label.setAutoFillBackground(True)
        self.power_label.setAlignment(QtCore.Qt.AlignCenter)
        self.power_label.setObjectName("power_label")
        self.horizontalLayout_2.addWidget(self.power_label)
        self.ignore_checkBox = QtWidgets.QCheckBox(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.ignore_checkBox.sizePolicy().hasHeightForWidth())
        self.ignore_checkBox.setSizePolicy(sizePolicy)
        self.ignore_checkBox.setObjectName("ignore_checkBox")
        self.horizontalLayout_2.addWidget(self.ignore_checkBox)
        self.verticalLayout.addLayout(self.horizontalLayout_2)
        self.horizontalLayout.addWidget(self.frame)

        self.retranslateUi(Strip)
        QtCore.QMetaObject.connectSlotsByName(Strip)

    def retranslateUi(self, Strip):
        _translate = QtCore.QCoreApplication.translate
        Strip.setWindowTitle(_translate("Strip", "Form"))
        self.name_label.setText(_translate("Strip", "AC NAME"))
        self.select_button.setText(_translate("Strip", "SELECT"))
        self.rc_label.setText(_translate("Strip", "RC STATUS"))
        self.power_label.setText(_translate("Strip", "POWER"))
        self.ignore_checkBox.setText(_translate("Strip", "Ignore"))

