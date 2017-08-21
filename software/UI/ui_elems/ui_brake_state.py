# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'resources/brake_state.ui'
#
# Created by: PyQt4 UI code generator 4.11.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_BrakeStateWidget(object):
    def setupUi(self, BrakeStateWidget):
        BrakeStateWidget.setObjectName(_fromUtf8("BrakeStateWidget"))
        BrakeStateWidget.resize(400, 300)
        self.label = QtGui.QLabel(BrakeStateWidget)
        self.label.setGeometry(QtCore.QRect(40, 30, 311, 211))
        self.label.setText(_fromUtf8(""))
        self.label.setPixmap(QtGui.QPixmap(_fromUtf8(":/images/slothstronaut.jpg")))
        self.label.setScaledContents(True)
        self.label.setObjectName(_fromUtf8("label"))
        self.lcdNumber_2 = QtGui.QLCDNumber(BrakeStateWidget)
        self.lcdNumber_2.setGeometry(QtCore.QRect(191, 100, 20, 23))
        self.lcdNumber_2.setNumDigits(1)
        self.lcdNumber_2.setObjectName(_fromUtf8("lcdNumber_2"))
        self.lcdNumber_3 = QtGui.QLCDNumber(BrakeStateWidget)
        self.lcdNumber_3.setGeometry(QtCore.QRect(140, 100, 20, 23))
        self.lcdNumber_3.setNumDigits(1)
        self.lcdNumber_3.setObjectName(_fromUtf8("lcdNumber_3"))

        self.retranslateUi(BrakeStateWidget)
        QtCore.QMetaObject.connectSlotsByName(BrakeStateWidget)

    def retranslateUi(self, BrakeStateWidget):
        BrakeStateWidget.setWindowTitle(_translate("BrakeStateWidget", "Form", None))

import resources_rc
