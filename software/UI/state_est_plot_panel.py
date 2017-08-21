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
from data_plot_widget import DataPlotWidget
from lcm_utils import *

class StateEstPlotPanel(QtGui.QWidget):
    ''' Summarizes state estimator info, and allows param tweaking.. '''
    def __init__(self, lc=None, parent=None, name=None):
        super(StateEstPlotPanel, self).__init__(parent)

        self.lc = lc
        if name:
            self.setObjectName(name)

        vBoxLayout  = QtGui.QVBoxLayout()

        self.PosPlotter = DataPlotWidget(1, title="Position Estimates", histLength=100, 
            alsoNumeric=True, name="x (m)", levelLines = [0.0])
        self.VelPlotter = DataPlotWidget(1, title="Velocity Estimates", histLength=100, 
            alsoNumeric=True, name="xd (m/s)", levelLines = [0.0])
        self.AccPlotter = DataPlotWidget(1, title="Acceleration Estimates", histLength=100, 
            alsoNumeric=True, name="xdd (m/s/s)", levelLines = [0.0])

        vBoxLayout.addWidget(self.PosPlotter)
        vBoxLayout.addWidget(self.VelPlotter)
        vBoxLayout.addWidget(self.AccPlotter)

        self.particles = None
        self.podse_sub = self.lc.subscribe("_FC_SE", self.handle_state_estimate)

        self.setLayout(vBoxLayout)
        
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(10)

    def update(self):
      if (self.isVisible):
        # todo: plot all particles. should be simple iteration
        if self.particles and len(self.particles) > 0:
          p = self.particles[0]
          self.PosPlotter.addDataPoint(0, float(time.time())/1000/1000, p[0][0])
          self.VelPlotter.addDataPoint(0, float(time.time())/1000/1000, p[0][1])
          self.AccPlotter.addDataPoint(0, float(time.time())/1000/1000, p[0][2])

    def handle_state_estimate(self, channel, data):
        msg = state_estimator_particle_set.decode(data)

        if (self.isVisible()):
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
    window = StateEstPlotPanel(lc=lc)
    window.show()

    start_lcm(lc)
    sys.exit(app.exec_())
