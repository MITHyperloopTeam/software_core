# the python stuff
import sys
import math
import signal
import random
import time
from threading import Lock
from collections import namedtuple

# numerics
import numpy as np
from numpy import linalg

# the interface stuff
from PyQt4 import QtCore, QtGui
import pyqtgraph as pg

# the messaging stuff
import lcm
from mithl import net_health_t
from lcm_utils import *


class NetDiagVisWidget(QtGui.QWidget):
    ''' Displays analog data and sends analog on/off commands. '''
    def __init__(self, lc=None, parent=None, name=None):
        super(NetDiagVisWidget, self).__init__(parent)

        self.lc = lc
        if name:
            self.setObjectName(name)

        self.plot = pg.PlotWidget(title="Network Health")
        self.plot.hideAxis("left")
        self.plot.hideAxis("bottom")

        self.viewBox = self.plot.getViewBox()

        self.graph = pg.GraphItem()
        self.viewBox.addItem(self.graph)
        self.viewBox.setMouseEnabled(x=False, y=False)

        self.node_to_ind_map = {}
        self.ind_to_node_list = []
        self.next_ind = 0

        self.node_pos = np.zeros((0, 2))
        self.node_vel = np.zeros((0, 2))
        
        self.color_type = [('red',np.ubyte),('green',np.ubyte),('blue',np.ubyte),('alpha',np.ubyte),('width',float)]
        self.AdjacencyInfo = namedtuple('AdjacencyInfo', ('color_type', 'est_rtt', 'est_drop_rate'))
        self.adjacency_info_map = {}

        self.adjacency_labels = {}
        self.node_labels = {}

        vBoxLayout  = QtGui.QVBoxLayout()
        vBoxLayout.addWidget(self.plot)
        self.setLayout(vBoxLayout)

        nethealth_sub = self.lc.subscribe("_NSUM", self.handle_net_health_t)

        self.lastUpdateTime = time.time()
        self.update_mutex = Lock()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(50)

    def update(self):
        if not self.isVisible():
            return

        self.update_mutex.acquire()

        dt = time.time() - self.lastUpdateTime
        self.lastUpdateTime = time.time()

        # let the nodes filter around
        node_forces = np.zeros(self.node_pos.shape)
        ATTRACT_FORCE = 1.0
        REPEL_FORCE = 1.0
        DAMPING = 1.0
        SPIN_FORCE = 0.1
        for node_i in range(node_forces.shape[0]):
            # attract to adjacent nodes
            for node_other_i in range(node_forces.shape[0]):
                vector = self.node_pos[node_other_i, :] - self.node_pos[node_i, :]
                if (node_i, node_other_i) in self.adjacency_info_map.keys():
                    node_forces[node_i, :] += ATTRACT_FORCE * vector
                if (np.linalg.norm(vector) > 0):
                    node_forces[node_i, :] -= REPEL_FORCE * vector / np.linalg.norm(vector)
            # for cool factor give it a little spin too
            node_forces[node_i, 0] += SPIN_FORCE * self.node_pos[node_i, 1]
            node_forces[node_i, 1] -= SPIN_FORCE * self.node_pos[node_i, 0]

        # also attact to origin
        node_forces -= ATTRACT_FORCE * self.node_pos
        node_forces -= DAMPING * self.node_vel

        self.node_vel += node_forces * min(dt, 0.1)
        self.node_pos += self.node_vel * min(dt, 0.1)

        if (len(self.adjacency_info_map.keys()) > 0):
            node_adj = []
            node_color = []
            for i in range(self.node_pos.shape[0]):
                for j in range(self.node_pos.shape[0]):
                    if i < j and (i, j) in self.adjacency_info_map.keys():
                        info = self.adjacency_info_map[(i, j)]
                        est_rtt_other = -1.
                        est_drop_rate_other = -1.
                        if (j, i) in self.adjacency_info_map.keys():
                            est_rtt_other = self.adjacency_info_map[(j, i)].est_rtt
                            est_drop_rate_other = self.adjacency_info_map[(j, i)].est_drop_rate

                        node_adj.append([i, j])
                        node_color.append(info.color_type)
                        midpoint = (self.node_pos[i, :] + self.node_pos[j, :]) / 2.0

                        # handle label update as well
                        if (i, j) not in self.adjacency_labels.keys():
                            newTextItem = pg.TextItem("Test")
                            self.adjacency_labels[(i, j)] = newTextItem
                            self.viewBox.addItem(newTextItem)
                        # one label per bi-directional edge
                        self.adjacency_labels[(i, j)].setPos(midpoint[0], midpoint[1])
                        self.adjacency_labels[(i, j)].setText("%02d->%02d: %0.3fms,%0.2f\nrtt %02d<-%02d: %0.3fms,%0.2f" % (self.ind_to_node_list[i], self.ind_to_node_list[j], info.est_rtt*1000.0, 
                                                                    info.est_drop_rate, self.ind_to_node_list[j], self.ind_to_node_list[i], est_rtt_other*1000.0, est_drop_rate_other))

                if i not in self.node_labels.keys():
                    newTextItem = pg.TextItem("Node %2d" % self.ind_to_node_list[i])
                    self.node_labels[i] = newTextItem
                    self.viewBox.addItem(newTextItem)  
                self.node_labels[i].setPos(self.node_pos[i, 0], self.node_pos[i, 1])
            self.graph.setData(adj=np.array(node_adj), pen=np.array(node_color))

        if (self.node_pos.size > 0):
            self.graph.setData(pos=self.node_pos)
            
            SPRINGFACTOR = 0.1
            BORDER=0.3
            xRangeRange = np.max(self.node_pos[:, 0]) - np.min(self.node_pos[:, 0])
            desiredXRange = np.array([np.mean(self.node_pos[:, 0]), np.mean(self.node_pos[:, 0])]) + [-xRangeRange*(0.5+BORDER), xRangeRange*(0.5+BORDER)]

            yRangeRange = np.max(self.node_pos[:, 1]) - np.min(self.node_pos[:, 1])
            desiredYRange = np.array([np.mean(self.node_pos[:, 1]), np.mean(self.node_pos[:, 1])]) + [-yRangeRange*(0.5+BORDER), yRangeRange*(0.5+BORDER)]

            currentXRange = np.array(self.viewBox.viewRange()[0])
            currentYRange = np.array(self.viewBox.viewRange()[1])
            commandXRange = (1.0-SPRINGFACTOR)*currentXRange + SPRINGFACTOR*desiredXRange
            commandYRange = (1.0-SPRINGFACTOR)*currentYRange + SPRINGFACTOR*desiredYRange
            self.viewBox.setRange(xRange = commandXRange, yRange=desiredYRange, padding=0.0)


        self.update_mutex.release()

    def handle_net_health_t(self, channel, data):
        msg = net_health_t.decode(data)

        self.update_mutex.acquire()

        for host_i in range(msg.num_pairs):
            if msg.pair_transport_id[host_i] == 0:
                if msg.host_a[host_i] not in self.node_to_ind_map.keys():
                    # insert! record node positions:
                    self.node_to_ind_map[msg.host_a[host_i]] = self.next_ind
                    self.ind_to_node_list.append(msg.host_a[host_i])
                    self.node_pos = np.vstack([self.node_pos, [random.random(), random.random()]])
                    self.node_vel = np.vstack([self.node_vel, [0., 0.]])
                    # increase size of adjacency matrices:
                    self.next_ind += 1
                if msg.host_b[host_i] not in self.node_to_ind_map.keys():
                    # insert! record node positions:
                    self.node_to_ind_map[msg.host_b[host_i]] = self.next_ind
                    self.ind_to_node_list.append(msg.host_b[host_i])
                    self.node_pos = np.vstack([self.node_pos, [random.random(), random.random()]])
                    self.node_vel = np.vstack([self.node_vel, [0., 0.]])
                    # increase size of adjacency matrices:
                    self.next_ind += 1

                ind_a = self.node_to_ind_map[msg.host_a[host_i]]
                ind_b = self.node_to_ind_map[msg.host_b[host_i]]

                edge_info = self.AdjacencyInfo(color_type=np.array((255, 125, 0, 255, 1.0), dtype=self.color_type), est_rtt=msg.est_rtt[host_i], est_drop_rate=msg.est_drop_rate[host_i])
                self.adjacency_info_map[(ind_a, ind_b)] = edge_info

                
        self.update_mutex.release()
        

if __name__ == '__main__':
    # hook up interrupt signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    
    lc = create_lcm()

    app = QtGui.QApplication(sys.argv)
    window = NetDiagVisWidget(lc=lc)
    window.show()

    start_lcm(lc)
    sys.exit(app.exec_())
