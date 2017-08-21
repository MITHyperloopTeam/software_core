#!/usr/bin/env python
#
# signif reference to http://pastebin.com/k87sfiEf
#
# authors: @gizatt, @geronm
#
# The purpose of this panel is to make it easier for a
# pilot to check that LCM values are within
# nominal ranges.
#
# input: Accepts a config file consisting of a list of
#   rows satisfying the following spec:
#
#         CHANNEL_NAME,mithl_lcm_class_name,field_name,desc,group_name,nom1,nom2[,nom3,nom4]
# 
# output: displays all request values, along with color codes indicating status.
#
#
# in-depth config format:
#
#   CHANNEL_NAME    --  lcm channel to listen to
#   mithl_lcm_class_name  --  lcm class to parse channel as
#   field_name      --  field from the message that this entry describes. An empty string indicates message Hz.
#   desc            --  NO-COMMAS human-readable description, appended after the reported value in the UI.
#   group_name      --  specific to this UI tool, allows for related values to be grouped together
#   nom1-4          --  either [red_lo,red_hi] or [red_lo, yell_lo, yell_hi, red_hi].
#
#   Some Example Lines:
#     _AR_BATTERY,battery_status_t,,rear battery frequency,Frequencies,2,3,25,1000
#     _AR_BATTERY,battery_status_t,internal_temp,rear battery temp (Celsius),Temperatures,0,80
#     _AF_BATTERY,battery_status_t,,rear battery frequency,Frequencies,2,3,25,1000
#     _AF_BATTERY,battery_status_t,internal_temp,rear battery temp (Celsius),Temperatures,0,80
#
# Notes:
#
# - Aggregate elements with the same channel name together, for performance
# - Do a quick check that all CHANNEL_NAME,lcm_class_name pairs are consistent
# - Check Greg's generic_message_sender.py code for LCMtrospection
# - 

import sys
import math
import signal
import time 
import os

import math, random
import numpy as np
from numpy import linalg
from PIL import Image

#interface stuff
from PyQt4 import QtCore, QtGui, QtOpenGL, Qt
import pyqtgraph as pg

#comms stuff
import lcm
import mithl
from lcm_utils import *

from data_plot_widget import DataPlotWidget

#read yaml config information
import yaml

#check for optional espeak dependency
check = os.system('which espeak')
ESPEAKING = False
if check==0:
    ESPEAKING = True
else:
    print 'espeak is not installed, vocal reporting is DISABLED'

voices= ['en','en-us','en-sc','en-n','en-rp','en-wm']

tosay = 'posture'
if len(sys.argv) > 1:
    for l in sys.argv[1:]:
        if '"' in l:
            raise Exception('Sentence should not contain quotes')
    tosay = ' '.join(sys.argv[1:])

def speak(s, o={}):
    if ESPEAKING:
      command = 'espeak'
      for opt in o:
          command += ' ' + str(opt) + ' ' + str(o[opt])
      command += ' \"' + str(s) + '\"' + ' &'
      return os.system(command)

HeaderNames = ['channel', 'lcm_type', 'field_name', 'group', 'lo', 'hi', '___value___', 'desc']
InputNames = ['channel', 'lcm_type', 'field_name', 'desc', 'group_name', 'lo', 'hi']

QtRedColor = QtGui.QColor(255,150,150)
QtOrangeColor = QtGui.QColor(251,79,20)
QtYellowColor = QtGui.QColor(255,255,200)
QtGreenColor = QtGui.QColor(180,255,200)

RedStyle = "{ background-color : rgb(255,100,100); color : black; }"
OrangeStyle = "{ background-color : rgb(251,79,20); color : black; }"
YellowStyle = "{ background-color : rgb(255,255,100); color : black; }"
GreenStyle = "{ background-color : rgb(180,255,200); color : black; }"

distancesPlotter = []
estimatedDistancePlotter = []
effortsPlotter = []
errorPlotter = []
integratorPlotter = []
pressurePlotter = []

last_msg_plot_time = None

#class NominalStatusPanel(object):
#    def __init__(self, di):
#        self.

channels_dict = {}


class ChannelItem(object):
    ''' Represents a single row of the status table. '''
    def __init__(self, lcm_class_name, field_name, desc, group_name, red_range, yellow_range):
        self.lcm_class_name = lcm_class_name
        self.field_name = field_name
        self.desc = desc
        self.group_name = group_name
        self.red_range = red_range
        self.yellow_range = yellow_range
    
    def __repr__(self):
        return str((self.lcm_class_name, self.field_name, self.desc, self.group_name, self.red_range, self.yellow_range))


class NominalStatusWatcher(object):
    ''' Listens for LCM requests, processes them
        and gets frequencies, and maintains all
        these values in external-facing data
        structures. The "model" of this MVC UI. '''
    MIN_COUNT_TO_GET_FREQ = 3
    FREQ_ALPHA_FILTER = .9
    TIMEOUT_SECONDS= 2.0
    FREQ_AVERAGING_SAMPLES = 20

    def __init__(self, lc, channel_item_dict, ui_callback):
        self.lc = lc
        self.channel_item_dict = channel_item_dict
        self.ui_callback = ui_callback
        
        # Initialize some stat trackers
        now = time.time()
        self.latest_time_heard = {k:now for k in self.channel_item_dict}
        self.latest_n_times_heard = {k:[now]*NominalStatusWatcher.FREQ_AVERAGING_SAMPLES for k in self.channel_item_dict}
        self.heard_last_iter = {k:False for k in self.channel_item_dict}
        self.latest_frequency_heard = {k:float('Inf') for k in self.channel_item_dict}
        self.count_heard = {k:0 for k in self.channel_item_dict}
        self.latest_values_heard = {k:[None for _ in self.channel_item_dict[k]] \
                            for k in self.channel_item_dict}
        
        # Set up lcm listeners for all channels
        for channel in channel_item_dict:
            channel_item_class = self.channel_item_dict[channel][0].lcm_class_name
            lcmclass = getattr(mithl,channel_item_class);
            self.lc.subscribe(channel, self.handle_this_type(lcmclass))

        # And that's all!

    def handle_this_type(self, lcmclass):
        def handle_function(channel, data):
            # Update channel statistics
            self.count_heard[channel] = self.count_heard[channel] + 1
            now = time.time()
            new_freq = 1/(now - self.latest_time_heard[channel])
            self.heard_last_iter[channel] = ((now - self.latest_time_heard[channel]) < self.TIMEOUT_SECONDS)
            self.latest_time_heard[channel] = now
            self.latest_n_times_heard[channel] = self.latest_n_times_heard[channel][1:] + [now]
            
            if self.latest_n_times_heard[channel][-1] == self.latest_n_times_heard[channel][0]:
                self.latest_frequency_heard[channel] = float('Inf')
            else:
                self.latest_frequency_heard[channel] = \
                NominalStatusWatcher.FREQ_AVERAGING_SAMPLES / float(self.latest_n_times_heard[channel][-1] - self.latest_n_times_heard[channel][0])
            
            # Extract new values
            msg = lcmclass.decode(data)
            for i, channel_item in enumerate(self.channel_item_dict[channel]):
                if channel_item.field_name == ':freq':
                    self.latest_values_heard[channel][i] = self.latest_frequency_heard[channel]
                elif channel_item.field_name == ':bandwidth':
                	self.latest_values_heard[channel][i] = self.latest_frequency_heard[channel]*len(data)/1000.
                else:
                    self.latest_values_heard[channel][i] = getattr(msg,channel_item.field_name)
            
            # Fire UI changes for all heard values
            self.ui_callback(self,channel)
            
        return handle_function

    def __repr__(self):
        return str(self.channel_item_dict)
        
def parse_config_file_stream(config_file_generator):
    ''' Parses a file of proper specification into
        a dictionary representation of all the channels
        and fields '''
    channel_item_dict = {}
    channel_to_type_dict = {} # For ensuring each channels has only entries of the same lcmtype

    for l in config_file_generator:
        l = l[:-1]
        if len(l)==0:
            continue
        li = l.split(',')
        assert (len(li)==len(InputNames) or len(li)==len(InputNames)+2),\
            'Error: incorrect number of columns in line: ' + l

        channel_name = li[0]
        lcm_class_name = li[1]
        field_name = li[2]
        desc = li[3]
        group_name = li[4]
        
        red_range = []
        yellow_range = []
        if len(li)==len(InputNames):
            red_range = li[len(InputNames)-2:len(InputNames)]
            yellow_range = list(red_range)
        elif len(li)==len(InputNames)+2:
            red_range = [li[len(InputNames)-2], li[len(InputNames)+1]]
            yellow_range = [li[len(InputNames)-1], li[len(InputNames)]]
        else:
            assert False, 'Invalid config line: '+l 
        red_range = [float(nst) for nst in red_range]
        yellow_range = [float(nst) for nst in yellow_range]
        assert len(red_range)==2, str(red_range)
        assert len(yellow_range)==2, str(yellow_range)
        
        if channel_name in channel_to_type_dict:
            assert channel_to_type_dict[channel_name]==lcm_class_name, \
                    'Contradictory lcm types detected for channel ' + channel_name + ': ' +\
                    lcm_class_name + '  and  ' + channel_to_type_dict[channel_name]
        
        if channel_name not in channel_item_dict:
            channel_item_dict[channel_name] = []
        
        channel_to_type_dict[channel_name] = lcm_class_name
        newItem = ChannelItem(lcm_class_name, field_name, desc, group_name, red_range, yellow_range)
        channel_item_dict[channel_name].append(newItem)

    return channel_item_dict
    
class StatusPanelTableWidget(QtGui.QTableWidget):
    ''' A QTableWidget which serves as
        the view for this MVC system.
        Updates self at 30 Hz while delivering
        dem premium features doe '''
    def __init__(self, watcher, parent=None):
        super(StatusPanelTableWidget, self).__init__(parent)

        self.watcher= watcher

        mainLayout= QtGui.QVBoxLayout()

        self.setColumnCount(len(HeaderNames))
        header= self.horizontalHeader()

        for i in range(len(HeaderNames)):
            field_label = QtGui.QTableWidgetItem(HeaderNames[i])
            self.setHorizontalHeaderItem(i, field_label)
            # header.setResizeMode(i, QtGui.QHeaderView.ResizeToContents)

        row_i= self.rowCount()

        global watched_value_ui_labels
        watched_value_ui_labels= {}

        for channel in sorted(watcher.channel_item_dict.keys()):
            watched_value_ui_labels[channel]= []
            channel_items= watcher.channel_item_dict[channel]
            for k, channel_item in enumerate(channel_items):
            	self.insertRow(row_i)

                labels= [channel, channel_item.lcm_class_name, \
                (channel_item.field_name if (channel_item.field_name != '') else 'Hz'), \
                channel_item.group_name, \
                str(channel_item.red_range[0]), str(channel_item.red_range[1]), \
                '', \
                channel_item.desc]

            	field_labels= range(len(labels))

                for col_i, label_str in enumerate(labels):
                    field_labels[col_i]= QtGui.QTableWidgetItem(label_str)
                    self.setItem(row_i, col_i, field_labels[col_i])

                row_i= self.rowCount()
                watched_value_ui_labels[channel].append(field_labels[-2])

        self.watched_value_ui_labels = watched_value_ui_labels

        self.watched_channel_screamed_already = {}
        for channel in self.watched_value_ui_labels:
            self.watched_channel_screamed_already[channel] = False

        self.setLayout(mainLayout)

        self.timer= QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(33)

    def update(self):
     	for channel in self.watcher.channel_item_dict:
            ui_labels = self.watched_value_ui_labels[channel]
            for i, channel_item in enumerate(watcher.channel_item_dict[channel]):
                ui_label = ui_labels[i]
                value = watcher.latest_values_heard[channel][i]
                if value is None:
                    value_fl = float('Inf')
                elif hasattr(value,'__len__'):
                    noinf = 0
                    value_fl = value
                    while hasattr(value_fl,'__getitem__') and noinf < 50:
                        noinf += 1
                        value_fl = value_fl[0]
                    value_fl = float(value_fl)
                else:
                    value_fl = float(value)
                
                if time.time() - watcher.latest_time_heard[channel] <= watcher.TIMEOUT_SECONDS:
                    # Didn't time out. Note this and proceed as usual
                    self.watched_channel_screamed_already[channel] = False
                    if value_fl >= channel_item.yellow_range[0] and value_fl <= channel_item.yellow_range[1]:
                        ui_label.setBackground(QtGreenColor)
                    elif value_fl >= channel_item.red_range[0] and value_fl <= channel_item.red_range[1]:
                        ui_label.setBackground(QtYellowColor)
                    else:
                        ui_label.setBackground(QtRedColor)
                else:
                    # Timed out. Scream if this wasn't true already.
                    if watcher.heard_last_iter[channel] and (not self.watched_channel_screamed_already[channel]):
                        # It wasn't, so SCREAM!
                        speak("lost comms to channel " + ' '.join(channel.split('_')), {'-v':voices[0], '-p':89})
                        self.watched_channel_screamed_already[channel] = True
                    ui_label.setBackground(QtOrangeColor)
                if value is None:
                    ui_label.setText(str(value))
                elif value_fl <= 10E6 and value_fl >= 10E-3:
                    ui_label.setText('%5.5F' % value_fl)
                else:
                    ui_label.setText('%2.5E' % value_fl)
                


class StatusPanelWidget(QtGui.QWidget):
    ''' A QWidget which serves as the
        view for this MVC system. Updates self at
        30 Hz '''
    def __init__(self, watcher, parent=None):
        super(StatusPanelWidget, self).__init__(parent)
        
        self.watcher = watcher
        
        mainLayout = QtGui.QGridLayout()
        
        for i in range(len(HeaderNames)):
            field_label = QtGui.QLabel(HeaderNames[i])
            mainLayout.addWidget(field_label, 0, i)
        
        row_i = 1
        global watched_value_ui_labels
        watched_value_ui_labels = {}
        for channel in sorted(watcher.channel_item_dict.keys()):
            watched_value_ui_labels[channel] = []
            channel_items = watcher.channel_item_dict[channel]
            for k, channel_item in enumerate(channel_items):
            	if (channel_item.field_name == ':freq'):
            		print_name = 'Frequency (Hz)'
            	elif (channel_item.field_name == ':bandwidth'):
            		print_name = 'Bandwidth (kB/s)'
            	else:
            		print_name = channel_item.field_name
                labels = [channel, channel_item.lcm_class_name, \
                    print_name, \
                    channel_item.group_name, \
                    str(channel_item.red_range[0]), str(channel_item.red_range[1]), \
                    '', \
                    channel_item.desc]
                if k > 0: # only need to show channel name and lcmtype once
                    labels[0] = ''
                    labels[1] = ''
                
                field_labels = range(len(labels))
                for col_i, label_str in enumerate(labels):
                    field_labels[col_i] = QtGui.QLabel(label_str)
                    mainLayout.addWidget(field_labels[col_i], row_i, col_i)
                row_i += 1
                watched_value_ui_labels[channel].append(field_labels[-2])
        self.watched_value_ui_labels = watched_value_ui_labels
        
        self.watched_channel_screamed_already = {}
        for channel in self.watched_value_ui_labels:
            self.watched_channel_screamed_already[channel] = False
        #def true_ui_callback(watcher, channel):
        #    self.repaint()
        #watcher.ui_callback = true_ui_callback

        self.setLayout(mainLayout)
        
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(33)

    def update(self):
        for channel in self.watcher.channel_item_dict:
            ui_labels = self.watched_value_ui_labels[channel]
            for i, channel_item in enumerate(watcher.channel_item_dict[channel]):
                ui_label = ui_labels[i]
                value = watcher.latest_values_heard[channel][i]
                if value is None:
                    value_fl = float('Inf')
                elif hasattr(value,'__len__'):
                    noinf = 0
                    value_fl = value
                    while hasattr(value_fl,'__getitem__') and noinf < 50:
                        noinf += 1
                        value_fl = value_fl[0]
                    value_fl = float(value_fl)
                else:
                    value_fl = float(value)
                if time.time() - watcher.latest_time_heard[channel] <= watcher.TIMEOUT_SECONDS:
                    # Didn't time out. Note this and proceed as usual
                    self.watched_channel_screamed_already[channel] = False
                    if value_fl >= channel_item.yellow_range[0] and value_fl <= channel_item.yellow_range[1]:
                        ui_label.setStyleSheet("QLabel " + GreenStyle)
                    elif value_fl >= channel_item.red_range[0] and value_fl <= channel_item.red_range[1]:
                        ui_label.setStyleSheet("QLabel " + YellowStyle)
                    else:
                        ui_label.setStyleSheet("QLabel " + RedStyle)
                else:
                    # Timed out. Scream if this wasn't true already.
                    if watcher.heard_last_iter[channel] and (not self.watched_channel_screamed_already[channel]):
                        # It wasn't, so SCREAM!
                        speak("lost comms to channel " + ' '.join(channel.split('_')), {'-v':voices[0], '-p':89})
                        self.watched_channel_screamed_already[channel] = True
                    ui_label.setStyleSheet("QLabel " + OrangeStyle)
                if value is None:
                    ui_label.setText(str(value))
                elif value_fl <= 10E6 and value_fl >= 10E-3:
                    ui_label.setText('%5.5F' % value_fl)
                else:
                    ui_label.setText('%2.5E' % value_fl)

if __name__ == '__main__':
    if len(sys.argv) <= 1:
        print "Usage: python status_panel.py PATH_TO_CONFIG_FILE"
        sys.exit(1)

    # hook up interrupt signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    lc = create_lcm()

    #testFile = open('/home/mithyperloop/mit-hyperloop-es/software/config/nominalStatusConfig.cfg','r')
    testFile = open(sys.argv[1],'r')
    channel_item_dict = parse_config_file_stream(testFile)
    
    def ui_callback(watch, chan):
        #print watch.channel_item_dict[chan][0].lcm_class_name
        #print str({(watch.channel_item_dict[chan][i].field_name):(watch.latest_values_heard[chan][i]) for i in range(len(watch.channel_item_dict[chan]))})
        pass
    
    watcher = NominalStatusWatcher(lc, channel_item_dict, ui_callback)
    
    print watcher

    app = QtGui.QApplication(sys.argv)

    window = QtGui.QWidget()

    windowLayout = QtGui.QHBoxLayout()

    # mainPanel = StatusPanelWidget(watcher)
    mainPanel = StatusPanelTableWidget(watcher)
    mainPanel.setSortingEnabled(True)

    scrollArea = QtGui.QScrollArea()
    scrollArea.setWidget(mainPanel)
    scrollArea.setWidgetResizable(True)

    windowLayout.addWidget(scrollArea)

    mainSizePolicy = QtGui.QSizePolicy()
    mainSizePolicy.setHorizontalPolicy(QtGui.QSizePolicy.MinimumExpanding)
    mainSizePolicy.setVerticalPolicy(QtGui.QSizePolicy.MinimumExpanding)

    window.setLayout(windowLayout)
    window.setSizePolicy(mainSizePolicy)
    window.show()

    start_lcm(lc)

    sys.exit(app.exec_())
