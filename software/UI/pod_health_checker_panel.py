# the python stuff
import sys
import math
import signal
import time
import yaml
from collections import namedtuple
# numerics
import numpy as np

# the interface stuff
from PyQt4 import QtCore, QtGui
from pod_health_monitor import RateCheckInfo, NetCheckInfo, ControlCheckInfo, SensorCheckInfo

# the messaging stuff
import lcm
from mithl import *
from lcm_utils import *

LABEL_DEFAULT_STYLE_SHEET = "font: 12pt; border-style: outset; border-width: 0px; border-color: black;"

LABEL_DISPLAY_GOOD_STYLE_SHEET = "font: 12pt; border-style: outset; border-width: 2px; border-color: black; background-color: green; color: white"
LABEL_DISPLAY_WARN_STYLE_SHEET = "font: 12pt; border-style: outset; border-width: 2px; border-color: black; background-color: blue; color: white"
LABEL_DISPLAY_STOP_STYLE_SHEET = "font: 12pt; border-style: outset; border-width: 2px; border-color: black; background-color: orange; color: white"
LABEL_DISPLAY_PANIC_STYLE_SHEET = "font: 12pt; border-style: outset; border-width: 2px; border-color: black; background-color: red; color: white"
LAYOUT_PANIC_STYLE_SHEET = "background-color: red"
LAYOUT_GOOD_STYLE_SHEET = "background-color: none"

LABEL_TITLE_GOOD_STYLE_SHEET = "font: 24pt; border-style: outset; border-width: 2px; border-color: black; background-color: green; color: white"
LABEL_TITLE_WARN_STYLE_SHEET = "font: 24pt; border-style: outset; border-width: 2px; border-color: black; background-color: blue; color: white"
LABEL_TITLE_STOP_STYLE_SHEET = "font: 24pt; border-style: outset; border-width: 2px; border-color: black; background-color: orange; color: white"
LABEL_TITLE_PANIC_STYLE_SHEET = "font: 24pt; border-style: outset; border-width: 2px; border-color: black; background-color: red; color: white"

def makeValPretty(val):
    if type(val) == str:
        return val
    elif type(val) == float:
        return "%.4f" % val
    else:
        return str(val)

def return_higher_resp(resp1, resp2):
    if (resp1 == 0):
        return resp2;
    if (resp2 == 0):
        return resp1;
    return max(resp1, resp2)

def setAppropriateStylesheet(widget, resp):
    if (resp == 0):
        widget.setStyleSheet(LABEL_DISPLAY_GOOD_STYLE_SHEET)
    elif (resp == -1):
        widget.setStyleSheet(LABEL_DISPLAY_WARN_STYLE_SHEET)
    elif (resp == 1):
        widget.setStyleSheet(LABEL_DISPLAY_STOP_STYLE_SHEET)
    elif (resp == 2):
        widget.setStyleSheet(LABEL_DISPLAY_PANIC_STYLE_SHEET)
    else:
        print "setAppropStylesheet handed value ", resp

def setAppropriateTitleStylesheet(widget, resp):
    if (resp == 0):
        widget.setStyleSheet(LABEL_TITLE_GOOD_STYLE_SHEET)
    elif (resp == -1):
        widget.setStyleSheet(LABEL_TITLE_WARN_STYLE_SHEET)
    elif (resp == 1):
        widget.setStyleSheet(LABEL_TITLE_STOP_STYLE_SHEET)
    elif (resp == 2):
        widget.setStyleSheet(LABEL_TITLE_PANIC_STYLE_SHEET)
    else:
        print "setAppropTitleStylesheet handed value ", resp

def wrapInVTitledItem(name, items):
  box = QtGui.QGroupBox(name)
  box.setAlignment(QtCore.Qt.AlignCenter) 
  boxLayout = QtGui.QVBoxLayout(box)
  for item in items:
    boxLayout.addWidget(item)
  return box
def wrapInHTitledItem(name, items):
  box = QtGui.QGroupBox(name)
  box.setAlignment(QtCore.Qt.AlignCenter) 
  boxLayout = QtGui.QHBoxLayout(box)
  for item in items:
    boxLayout.addWidget(item)
  return box
def spawnBasicLabel(txt = "<>"):
  qs = QtGui.QLabel(txt)
  qs.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
  qs.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET)
  qs.setAlignment(QtCore.Qt.AlignCenter)
  return qs
def spawnBasicLabelDisplay(txt = "<>"):
  qs = QtGui.QLabel(txt)
  qs.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
  qs.setStyleSheet(LABEL_DISPLAY_PANIC_STYLE_SHEET)
  qs.setAlignment(QtCore.Qt.AlignCenter)
  return qs

def spawnBasicButton(txt = "<>"):
  qs = QtGui.QPushButton(txt)
  qs.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
  qs.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET)
  return qs
def spawnBasicTextEntry():
  qs = QtGui.QLineEdit('')
  qs.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
  qs.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET)
  return qs
def spawnProgressBar(max, min):
  ql = QtGui.QProgressBar()
  ql.setMaximum(max)
  ql.setMinimum(min)
  ql.setOrientation(QtCore.Qt.Vertical)
  return ql

def wrapWithTitleOnLeft(name, items):
  box = QtGui.QWidget()
  boxLayout = QtGui.QHBoxLayout(box)
  boxLayout.addWidget(spawnBasicLabel(name))
  for item in items:
    boxLayout.addWidget(item)
  return box

class PodHealthCheckerPanel(QtGui.QWidget):
    ''' Summarizes some high-level pod health information. '''
    def __init__(self, configFile, lc=None, parent=None, name=None):
        super(PodHealthCheckerPanel, self).__init__(parent)

        self.lc = lc
        if name:
            self.setObjectName(name)
        overallLayout  = QtGui.QGridLayout()

        config = yaml.load(open(configFile, 'r'))

        # NET CHECKS
        self.max_total_bandwidth = config["net_checks"]["max_total_bandwidth"]
        self.total_bandwidth = 0
        self.netChecker = NetCheckInfo(config["net_checks"], lc)
        self.net_check_last_update = time.time()

        self.netCheckLabel = QtGui.QLabel("Net Rate Checks")
        self.netCheckLabel.setStyleSheet(LABEL_TITLE_GOOD_STYLE_SHEET)
        overallLayout.addWidget(self.netCheckLabel, 0, 0, 1, 1)
        self.netCheckBoxes = []
        self.NetCheckBox = namedtuple('NetCheckBox', ('est_rtt_box', 'est_drop_rate_box'))
        layout = QtGui.QGridLayout()
        i = 0
        for item in self.netChecker.required_performances:
            name_box = QtGui.QLabel(str(item.name_1) + " <-> " + str(item.name_2))
            name_box.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
            layout.addWidget(name_box, i*2, 0, 2, 1)
            rtt_name_box = QtGui.QLabel("RTT")
            rtt_name_box.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
            est_rtt_box = QtGui.QLabel("???")
            est_rtt_box.setStyleSheet(LABEL_DISPLAY_PANIC_STYLE_SHEET)
            max_rtt_box = QtGui.QLabel(" < " + makeValPretty(item.max_rtt))
            layout.addWidget(rtt_name_box, i*2, 1, 1, 1)
            layout.addWidget(est_rtt_box, i*2, 2, 1, 1)
            layout.addWidget(max_rtt_box, i*2, 3, 1, 1)
            drop_rate_name_box = QtGui.QLabel("Drop Rate")
            drop_rate_name_box.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
            est_drop_rate_box = QtGui.QLabel("???")
            est_drop_rate_box.setStyleSheet(LABEL_DISPLAY_PANIC_STYLE_SHEET)
            max_drop_rate_box = QtGui.QLabel(" < " + makeValPretty(item.max_drop_rate))
            layout.addWidget(drop_rate_name_box, i*2+1, 1, 1, 1)
            layout.addWidget(est_drop_rate_box, i*2+1, 2, 1, 1)
            layout.addWidget(max_drop_rate_box, i*2+1, 3, 1, 1)
            self.netCheckBoxes.append(self.NetCheckBox(est_rtt_box = est_rtt_box, est_drop_rate_box = est_drop_rate_box))
            i += 1            
        overallLayout.addLayout(layout, 1, 0, 1, 1)

        # CONTROL CHECKS
        self.controlChecker = ControlCheckInfo(config["control_checks"], lc)
        self.control_check_last_update = time.time()

        self.controlCheckLabel = QtGui.QLabel("Control Checks")
        self.controlCheckLabel.setStyleSheet(LABEL_TITLE_GOOD_STYLE_SHEET)
        overallLayout.addWidget(self.controlCheckLabel, 0, 1, 1, 1)

        layout = QtGui.QGridLayout()

        est_oscillations_name = QtGui.QLabel("Max Oscillations")
        est_oscillations_name.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        self.est_oscillations_box = QtGui.QLabel("???")
        est_oscillations_max = QtGui.QLabel(" < " + str(self.controlChecker.max_imu_oscillations))
        layout.addWidget(est_oscillations_name, 0, 0, 1, 1)
        layout.addWidget(self.est_oscillations_box, 0, 1, 1, 1)
        layout.addWidget(est_oscillations_max, 0, 2, 1, 1)

        watchdogs_state = QtGui.QLabel("Auto/Teleop WDs")
        watchdogs_state.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        self.watchdogs_state_box = QtGui.QLabel("???")
        watchdogs_state_des = QtGui.QLabel(" on unless teleop")
        layout.addWidget(watchdogs_state, 1, 0, 1, 1)
        layout.addWidget(self.watchdogs_state_box, 1, 1, 1, 1)
        layout.addWidget(watchdogs_state_des, 1, 2, 1, 1)
        overallLayout.addLayout(layout, 1, 1, 1, 1)

        rate_and_sensor_layout = QtGui.QVBoxLayout()
        # RATE CHECKS
        self.frequency_droop_allowed = config["message_rate_checks"]["frequency_droop_allowed"]
        self.rate_check_info_by_channel = {}
        for message_rate_check in config["message_rate_checks"]["items"]:
            self.rate_check_info_by_channel[message_rate_check["channel"]] = \
                RateCheckInfo(message_rate_check, lc)
        self.rate_check_last_update = time.time()

        self.rateCheckLabel = QtGui.QLabel("Message Rate Checks")
        self.rateCheckLabel.setStyleSheet(LABEL_TITLE_GOOD_STYLE_SHEET)
        overallLayout.addWidget(self.rateCheckLabel, 2, 0, 1, 1)
        self.rateCheckBoxes = {}
        layout = QtGui.QGridLayout()
        i = 0
        msg_names_in_alpha = sorted(self.rate_check_info_by_channel.keys(), key=str.lower)
        for priority in [2, 1, -1, 0]:
            for key in msg_names_in_alpha:
                value = self.rate_check_info_by_channel[key]
                if value.resp_level == priority:
                    msg_name_box = QtGui.QLabel(key)
                    msg_name_box.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
                    rate_box = QtGui.QLabel("???")
                    rate_box.setStyleSheet(LABEL_DISPLAY_PANIC_STYLE_SHEET)
                    rate_bound_box = QtGui.QLabel(" > " + str(value.target_frequency) + "*" + str(self.frequency_droop_allowed))
                    layout.addWidget(msg_name_box, i, 0, 1, 1)
                    layout.addWidget(rate_box, i, 1, 1, 1)
                    layout.addWidget(rate_bound_box, i, 2, 1, 1)
                    self.rateCheckBoxes[key] = rate_box
                    i += 1          

        checksWidget = QtGui.QWidget()
        checksWidget.setLayout(layout)
        scroll = QtGui.QScrollArea()
        scroll.setWidget(checksWidget)
        overallLayout.addWidget(scroll, 3, 0, 1, 1)

        # SENSOR CHECKS
        self.sensor_check_groups = {}
        for group in config["sensor_checks"]["items"].keys():
            sensor_check_info_by_name = {}
            for sensor_check in config["sensor_checks"]["items"][group]:
                sensor_check_info_by_name[sensor_check["name"]] = \
                    SensorCheckInfo(sensor_check, lc)
            self.sensor_check_groups[group] = sensor_check_info_by_name
        self.sensor_check_last_update = time.time()

        self.sensorCheckLabel = QtGui.QLabel("Sensor Range Checks")
        self.sensorCheckLabel.setStyleSheet(LABEL_TITLE_GOOD_STYLE_SHEET)
        overallLayout.addWidget(self.sensorCheckLabel, 2, 1, 1, 1)
        self.sensorCheckBoxes = {}
        self.sensorGroupLabels = {}
        self.SensorCheckBox = namedtuple('SensorCheckBox', ('val_box', 'var_box'))
        layout = QtGui.QGridLayout()
        i = 0
        sensorGroupsAlpha = sorted(self.sensor_check_groups.keys(), key=str.lower)
        for group in sensorGroupsAlpha:
            header = QtGui.QLabel(group)
            header.setStyleSheet(LABEL_TITLE_GOOD_STYLE_SHEET)
            header.setAlignment(QtCore.Qt.AlignCenter | QtCore.Qt.AlignVCenter)
            self.sensorGroupLabels[group] = header
            layout.addWidget(header, i, 0, 1, 5)
            i += 1

            sensorCheckBoxGroup = {}
            sensorNamesAlpha = sorted(self.sensor_check_groups[group].keys(), key=str.lower)

            for priority in [2, 1, -1, 0]:
                for item in sensorNamesAlpha:
                    val = self.sensor_check_groups[group][item]
                    if (val.resp_level == priority):
                        name_box = QtGui.QLabel(item)
                        name_box.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
                        layout.addWidget(name_box, i, 1, 2, 1)
                        min_box = QtGui.QLabel(str(val.min) + " < ")
                        min_box.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
                        layout.addWidget(min_box, i, 2, 1, 1)
                        val_box = QtGui.QLabel("???")
                        val_box.setStyleSheet(LABEL_DISPLAY_PANIC_STYLE_SHEET)
                        layout.addWidget(val_box, i, 3, 1, 1)
                        max_box = QtGui.QLabel(" < " + str(val.max))
                        layout.addWidget(max_box, i, 4, 1, 1)

                        min_var_box = QtGui.QLabel(str(val.noise_lower) + " < ")
                        min_var_box.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
                        layout.addWidget(min_var_box, i+1, 2, 1, 1)
                        var_box = QtGui.QLabel("???")
                        var_box.setStyleSheet(LABEL_DISPLAY_PANIC_STYLE_SHEET)
                        layout.addWidget(var_box, i+1, 3, 1, 1)
                        max_var_box = QtGui.QLabel(" < " + str(val.noise_upper))
                        layout.addWidget(max_var_box, i+1, 4, 1, 1)
                        sensorCheckBoxGroup[item] = self.SensorCheckBox(val_box = val_box, var_box = var_box)
                        i += 2           
            self.sensorCheckBoxes[group] = sensorCheckBoxGroup

        checksWidget = QtGui.QWidget()
        checksWidget.setLayout(layout)
        scroll = QtGui.QScrollArea()
        scroll.setWidget(checksWidget)
        overallLayout.addWidget(scroll, 3, 1, 1, 1)


        self.setLayout(overallLayout)

        self.lastRateCheckUpdates = time.time()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(100)



    def update(self):
        if time.time() - self.lastRateCheckUpdates > 3.0:
            elapsed = time.time() - self.lastRateCheckUpdates
            self.lastRateCheckUpdates = time.time()
            worst = 0
            for key, value in self.rate_check_info_by_channel.iteritems():
                value.update(elapsed)
                self.rateCheckBoxes[key].setText(makeValPretty(value.frequency))
                resp, expl = value.get_violation(self.frequency_droop_allowed)
                setAppropriateStylesheet(self.rateCheckBoxes[key], resp)
                worst = return_higher_resp(resp, worst)
            setAppropriateTitleStylesheet(self.rateCheckLabel, worst)


        if not self.isVisible():
            return
            
        # NET CHECKER
        worst = 0
        for i, item in enumerate(self.netChecker.required_performances):
            box = self.netCheckBoxes[i]
            val, resp = self.netChecker.get_rtt(item)
            worst = return_higher_resp(worst, resp)
            box.est_rtt_box.setText(makeValPretty(val))
            setAppropriateStylesheet(box.est_rtt_box, resp)
            val, resp = self.netChecker.get_drop_rate(item)
            worst = return_higher_resp(worst, resp)
            box.est_drop_rate_box.setText(makeValPretty(val))
            setAppropriateStylesheet(box.est_drop_rate_box, resp)
        setAppropriateTitleStylesheet(self.netCheckLabel, worst)

        # CONTROL CHECKS
        val, resp1 = self.controlChecker.get_oscillations()
        self.est_oscillations_box.setText(makeValPretty(val))
        setAppropriateStylesheet(self.est_oscillations_box, resp1)
        val, resp2 = self.controlChecker.get_watchdogs()
        self.watchdogs_state_box.setText(makeValPretty(val))
        setAppropriateStylesheet(self.watchdogs_state_box, resp2)
        setAppropriateTitleStylesheet(self.controlCheckLabel, return_higher_resp(resp1, resp2))

        # SENSOR CHECKS
        all_worst = 0
        for group in self.sensor_check_groups.keys():
            worst_here = 0
            for item in self.sensor_check_groups[group].keys():
                checker = self.sensor_check_groups[group][item]
                boxes = self.sensorCheckBoxes[group][item]
                val, resp = checker.get_mean()
                worst_here = return_higher_resp(worst_here, resp)
                boxes.val_box.setText(makeValPretty(val))
                setAppropriateStylesheet(boxes.val_box, resp)
                val, resp = checker.get_var()
                boxes.var_box.setText(makeValPretty(val))
                setAppropriateStylesheet(boxes.var_box, resp)
                worst_here = return_higher_resp(worst_here, resp)
            all_worst = return_higher_resp(worst_here, all_worst)
            setAppropriateTitleStylesheet(self.sensorGroupLabels[group], worst_here)
        setAppropriateTitleStylesheet(self.sensorCheckLabel, all_worst)

if __name__ == '__main__':
    # hook up interrupt signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    
    lc = create_lcm()

    app = QtGui.QApplication(sys.argv)
    window = PodHealthCheckerPanel("../config/healthConfig.yaml", lc=lc)
    window.show()

    start_lcm(lc)
    sys.exit(app.exec_())
