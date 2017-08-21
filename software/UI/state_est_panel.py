# the python stuff
import sys
import math
import signal
import time

# numerics
import numpy as np

# the interface stuff
from PyQt4 import QtCore, QtGui

# the messaging stuff
import lcm
from mithl import trigger_t
from mithl import state_estimator_particle_set
from mithl import state_estimator_particle
from mithl import state_estimator_state_t
from mithl import state_estimator_config_t
from lcm_utils import *

from generic_message_sender import GenericMessageSender

LABEL_DEFAULT_STYLE_SHEET = "font: 12pt; border-style: outset; border-width: 0px; border-color: black;"
LABEL_DISPLAY_GOOD_STYLE_SHEET = "font: 14pt; border-style: outset; border-width: 2px; border-color: black; background-color: green; color: white"
LABEL_DISPLAY_WARN_STYLE_SHEET = "font: 14pt; border-style: outset; border-width: 2px; border-color: black; background-color: orange; color: white"
LABEL_DISPLAY_PANIC_STYLE_SHEET = "font: 14pt; border-style: outset; border-width: 2px; border-color: black; background-color: red; color: white"
LAYOUT_PANIC_STYLE_SHEET = "background-color: red"
LAYOUT_GOOD_STYLE_SHEET = "background-color: none"
TITLE_DEFAULT_STYLE_SHEET = "font: 24pt"



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

class StateEstPanel(QtGui.QWidget):
    ''' Summarizes state estimator info, and allows param tweaking.. '''
    def __init__(self, lc=None, parent=None, name=None):
        super(StateEstPanel, self).__init__(parent)

        self.lc = lc
        if name:
            self.setObjectName(name)

        vBoxLayout  = QtGui.QVBoxLayout()

        dataLayout = QtGui.QHBoxLayout()
        self.mean_data = QtGui.QTableWidget(1, 3)
        self.var_data = QtGui.QTableWidget(3, 3)
        for i in range(3):
            for j in range(3):
                newitem = QtGui.QTableWidgetItem()
                self.var_data.setItem(i, j, newitem)
            newitem = QtGui.QTableWidgetItem(newitem)
            self.mean_data.setItem(0, i, newitem)

        dataLayout.addWidget(wrapInVTitledItem("Mean Estimate", [self.mean_data]))
        dataLayout.addWidget(wrapInVTitledItem("Covariance", [self.var_data]))
        vBoxLayout.addLayout(dataLayout)

        window = GenericMessageSender(lc=lc, message_type=state_estimator_config_t,
          default_vals={
            "utime" : 0,
            "Sigma_meas_acc": 0.01,
            "Sigma_process_add_pos": 0.0,
            "Sigma_process_add_vel": 0.01,
            "Sigma_process_add_acc": 20.0,
            "fiducial_timeout": 1.0,
            "fiducial_FP_rate": 0.0005,
            "fiducial_sigma": 5.0,
            "fiducial_vel_sigma": 0.02,
            "fiducial_max_vel_difference": 20.0,
            "incremental_tare_alpha": 0.9995,
            "use_fiducial_detections": False,
            "use_multiple_particles": False,
            "max_particles": 10,
            "tube_length": 1280.16,
            "fiducial_separation": 30.48,
            "distance_after_last_fiducial": 30.48,
            "fiducial_width": 0.1016,
            "SD_update_period": 0.01,
            "SD_stopping_threshold": 0.05,
            "SD_forward_acc_threshold": 0.5,
            "SD_stop_timeout": 0.5
          }, 
          limits_dict={
            "tube_length": [0, 1000000],
            "max_particles": [0, 100],
            "fiducial_separation": [0, 1000],
            "distance_after_last_fiducial": [0, 1000],
            "fiducial_timeout": [0, 1000],
            "fiducial_sigma": [0, 1000],
            "fiducial_vel_sigma": [0, 1000],
            "fiducial_max_vel_difference": [0, 1000],
            "Sigma_meas_acc": [0, 10000000],
            "Sigma_process_add_pos": [0, 10000000],
            "Sigma_process_add_vel": [0, 10000000],
            "Sigma_process_add_acc": [0, 10000000]
          },
          publish_channel="_FC_SE_CONFIG_SET", receive_channel="_FC_SE_CONFIG")
        vBoxLayout.addWidget(window)

        resetButton = QtGui.QPushButton('Reset')
        resetButton.clicked.connect(self.sendNavReset)
        resetButton.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        vBoxLayout.addWidget(resetButton)


        self.particles = np.zeros((0, 3))
        self.podse_sub = self.lc.subscribe("_FC_SE", self.handle_state_estimate)

        self.setLayout(vBoxLayout)
        
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(33)

    def update(self):
        for p in self.particles:
            mean = np.array(p[0])
            var = np.array(p[1])

            for i in range(3):
                self.mean_data.item(0, i).setText("%3.3f" % mean[i])


            for i in range(3):
                for j in range(3):
                    self.var_data.item(i, j).setText("%3.3f" % var[i][j])

    def sendNavReset(self):
        msg = trigger_t()
        msg.utime = time.time()*1000*1000
        self.lc.publish("RESET", msg.encode())

    def handle_state_estimate(self, channel, data):
        msg = state_estimator_particle_set.decode(data)
        self.pod_mle = msg.particles[0].mu[0]

        particles = []
        for i in range(msg.n_particles):
            if msg.particles[i].id >= 0 and msg.particles[i].weight > 0.:
                particles.append([msg.particles[i].mu,
                                  msg.particles[i].Sigma,
                                  msg.particles[i].weight])
        self.particles = particles

if __name__ == '__main__':
    # hook up interrupt signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    
    lc = create_lcm()

    app = QtGui.QApplication(sys.argv)
    window = StateEstPanel(lc=lc)
    window.show()

    start_lcm(lc)
    sys.exit(app.exec_())
