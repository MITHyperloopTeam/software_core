#!/usr/bin/env python

#signif reference to http://pastebin.com/k87sfiEf


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
from PyQt4 import QtCore, QtGui, QtOpenGL
import pyqtgraph as pg

#comms stuff
import lcm
from mithl import fiducial_teach_table_t
from mithl import fiducial_config_t
from mithl import fiducial_t
from mithl import fiducial_color_row_t
from mithl import fiducial_color_dataval_t
from lcm_utils import *

# what actually drives this
from generic_message_sender import GenericMessageSender

from data_plot_widget import DataPlotWidget

#read yaml config information
import yaml

from copy import deepcopy

distancesPlotter = []
durationsPlotter = []
colorPlotter = []
errorPlotter = []
integratorPlotter = []
pressurePlotter = []

last_msg_plot_time_high_rate = None
last_msg_plot_time_dataval = None

def handle_fiducial_state_high_rate_t(channel, data):
    global last_msg_plot_time_high_rate
    msg = fiducial_t.decode(data)
    
    t = msg.utime / 1000. / 1000.
    if (not last_msg_plot_time_high_rate or t - last_msg_plot_time_high_rate > 0.05):
        distancesPlotter.addDataPoint(0, t, msg.average_time_between)
        durationsPlotter.addDataPoint(0, t, msg.average_time_strip)
        
        last_msg_plot_time_high_rate = t

def handle_fiducial_color_dataval_t(channel, data):
    msg = fiducial_color_dataval_t.decode(data)
    global last_msg_plot_time_dataval
    
    t = msg.utime / 1000. / 1000.
    if (not last_msg_plot_time_dataval or t - last_msg_plot_time_dataval > 0.05):
        colorPlotter.addDataPoint(0, t, msg.color_x)
        colorPlotter.addDataPoint(1, t, msg.color_y)
        colorPlotter.addDataPoint(2, t, msg.color_int)
        
        last_msg_plot_time_dataval = t

def handle_fiducial_set_color_row_t(channel, data):
    global lc
    TOTAL_NUM_ROWS = 31

    desired_row0 = fiducial_color_row_t.decode(data)
    empty_row0 = fiducial_color_row_t()

    desired_tt = fiducial_teach_table_t()
    desired_tt.utime = time.time() * 1000 * 1000
    desired_tt.rows = [desired_row0] + [empty_row0 for _ in range(TOTAL_NUM_ROWS - 1)]
    
    for i in range(TOTAL_NUM_ROWS):
        desired_tt.rows[i].row_no = i

    # print "Heard command to set color_row_t..."

    lc.publish("_FIDUCIAL_SET_TEACH_TABLE", desired_tt.encode())

def handle_fiducial_cur_teach_table_t(channel, data):
    global lc
    msg = fiducial_teach_table_t.decode(data)

    row_data = msg.rows[0]; # row_data will be an instance of mithl.color_row_t
    row_data.utime = time.time() * 1000 * 1000

    lc.publish("_FIDUCIAL_UI_CUR_ONE_ROW", row_data.encode())


if __name__ == '__main__':
    DEFAULT_PARAMS_FILEPATH = '../config/fiducialParamsDefault.yaml'

    # hook up interrupt signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    # load config file
    fiducial_params = None
    default_fiducial_params = None
    with open(DEFAULT_PARAMS_FILEPATH, 'r') as f:
        default_fiducial_params = yaml.load(f) # THIS IS ASSUMED TO BE GOOD
    assert default_fiducial_params is not None, 'ERROR: Could not find default params in ' + DEFAULT_PARAMS_FILEPATH
    # If there is an argument, check to see whether it is a valid substitute for default config.
    if len(sys.argv) > 1:
        fname = sys.argv[1]
        try:
            assert os.path.exists(fname), 'Could not find file ' + str(fname)
            with open(fname, 'r') as f:
                new_fiducial_params = yaml.load(f) # this may or may not be good
            # Check that these fiducial params are up to snuff, ie. that they
            # share all necessary fields with default config
            for default_key in default_fiducial_params['config']:
                assert default_key in new_fiducial_params['config'], 'Error: missing key %s' % default_key
            for default_key in default_fiducial_params['teach_table_row']:
                assert default_key in new_fiducial_params['teach_table_row'],  'Error: missing key %s' % default_key

            print 'Successfully loaded from non-default config file ' + fname
            fiducial_params = new_fiducial_params
        except Exception as e:
            print "Could not load params file from " + fname
            print "   > Error:" + str(e.args)
    if fiducial_params is None:
        fiducial_params = default_fiducial_params

    global lc
    lc = create_lcm()

    app = QtGui.QApplication(sys.argv)

    window = QtGui.QWidget()
    
    windowLayout = QtGui.QHBoxLayout()

    controlPanelLayout = QtGui.QVBoxLayout()

    tabWidget = QtGui.QTabWidget()

    fiducial_config_scroll_area = QtGui.QScrollArea()
    #fiducial_config_scroll_area.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
    
    config_default_vals = fiducial_params['config']
    config_message_sender = GenericMessageSender(lc=lc, message_type=fiducial_config_t, 
            default_vals=config_default_vals,
            limits_dict={"power":[1,1000]},
            publish_channel="_FIDUCIAL_SET_CONFIG", receive_channel="_FIDUCIAL_CUR_CONFIG", spam_period=1.0)
    config_message_sender.toggleActiveCommandMode() # start with command mode active, so yaml-populated fields get commanded.

    fiducial_config_scroll_area.setWidget(config_message_sender)
    
    tabWidget.addTab(fiducial_config_scroll_area, "Config")

    fiducial_mode_scroll_area = QtGui.QScrollArea()
    
    tt_default_vals = fiducial_params['teach_table_row']
    tt_message_sender = GenericMessageSender(lc=lc, message_type=fiducial_color_row_t, 
            default_vals=tt_default_vals, publish_channel="_FIDUCIAL_UI_SET_ONE_ROW", receive_channel="_FIDUCIAL_UI_CUR_ONE_ROW",
            options_dict={ }, spam_period=1.0)
          #      "mode": ["MODE_ESTOP", "MODE_TELEOP", "MODE_AUTO"]
          #  }))
    tt_message_sender.toggleActiveCommandMode() # start with command mode active, so yaml-populated fields get commanded.

    fiducial_mode_scroll_area.setWidget(tt_message_sender)

    tabWidget.addTab(fiducial_mode_scroll_area, "Teach Table")

    saveload = QtGui.QWidget()
    layout = QtGui.QVBoxLayout()

    def save_config():
        fname = QtGui.QFileDialog.getSaveFileName(saveload, 'Open params file', 
            '../config/fiducialParamsCustom.yaml',"YAML files (*.yaml)")
        fname = str(fname)
        if os.path.realpath(fname) == os.path.realpath(DEFAULT_PARAMS_FILEPATH):
            print 'ERROR: Refusing to overwrite default params file ' + DEFAULT_PARAMS_FILEPATH
            msg = QtGui.QMessageBox()
            msg.setIcon(QtGui.QMessageBox.Information)
            msg.setText("Error: File Not Saved")
            msg.setInformativeText("Refusing to overwrite default params file %s (Save to a different file and overwrite it manually if necessary)" % os.path.realpath(DEFAULT_PARAMS_FILEPATH))
            msg.setWindowTitle("Error")
            retval = msg.exec_()
        else:
            with open(fname,'w') as ostream:
                try:
                    tentative_params = {}
                    tentative_params['config'] = {}
                    tentative_params['teach_table_row'] = {}
                    for fieldname in default_fiducial_params['config']:
                        tentative_params['config'][fieldname] = getattr(config_message_sender.desiredConfigMsg, fieldname)
                    for fieldname in default_fiducial_params['teach_table_row']:
                        tentative_params['teach_table_row'][fieldname] = getattr(tt_message_sender.desiredConfigMsg, fieldname)
                    yaml.dump(tentative_params, ostream)
                    print "Successfully saved params to " + str(fname)
                except yaml.YAMLError as exc:
                    print 'ERROR: Failed to write file to ' + str(fname)
    saveload.btn = QtGui.QPushButton("Save Params As...")
    saveload.btn.clicked.connect(save_config)
    layout.addWidget(saveload.btn)

#    def load_config():
#        fname = QtGui.QFileDialog.getOpenFileName(saveload, 'Open params file', 
#            '../config',"YAML files (*.yaml);; All files (*.*)")
#        try:
#            if fname is None:
#                raise Exception('fname was None')
#            with open(str(fname), 'r') as f:
#                new_fiducial_params = yaml.load(f)
#            # Check that these fiducial params are up to snuff, ie. that they
#            # share all necessary fields with default config
#            with open(DEFAULT_PARAMS_FILEPATH, 'r') as f:
#                default_fiducial_params = yaml.load(f)
#            for default_key in default_fiducial_params['config']:
#                assert default_key in new_fiducial_params['config'], 'Error: missing key %s' % default_key
#            for default_key in default_fiducial_params['teach_table_row']:
#                assert default_key in new_fiducial_params['teach_table_row'],  'Error: missing key %s' % default_key
#            # If everything checks out, take necessary steps to incorporate new values
#            # into UI
#            fiducial_params = new_fiducial_params
#            config_message_sender.tentative_config = deepcopy(fiducial_params['config'])
#            config_message_sender.update()
#            tt_message_sender.tentative_config = deepcopy(fiducial_params['teach_table_row'])
#            tt_message_sender.update()
#            print 'Successfully loaded params from ' + fname
#        except Exception as e:
#            print "Error loading file. Fname had value " + str(fname)
#            print "   > Error message: " + str(e.args)
#    saveload.btn2 = QtGui.QPushButton("Load Params...")
#    saveload.btn2.clicked.connect(load_config)
#    layout.addWidget(saveload.btn2)

    saveload.setLayout(layout)

    saveload_scroll_area = QtGui.QScrollArea()
    saveload_scroll_area.setWidget(saveload)
    
    tabWidget.addTab(saveload_scroll_area, "Save/Load")

    # print "Teach table message sender constructed..."

    controlPanelLayout.addWidget(tabWidget)

    controlPanel = QtGui.QWidget()
    controlPanel.setLayout(controlPanelLayout)
    controlPanel.setMinimumWidth(400)
    windowLayout.addWidget(controlPanel)

    plotsLayout = QtGui.QVBoxLayout()

    distancesPlotter = DataPlotWidget(1, title="Strip Period Est (s)", alsoNumeric=True)
    durationsPlotter = DataPlotWidget(1, title="Intra-Strip Time Est (s)", alsoNumeric=True)
    colorPlotter = DataPlotWidget(3, title="Color Values", dataRange=[0, 4096])
#    pressurePlotter = DataPlotWidget(1, title="Pressure", alsoNumeric=True, avgWindow = 0.05, dataRange=[0, 1500])
    
    distancesLayout = QtGui.QHBoxLayout()
    distancesLayout.addWidget(distancesPlotter)
    distancesLayout.addWidget(durationsPlotter)

    colorLayout = QtGui.QHBoxLayout()
    colorLayout.addWidget(colorPlotter)

    plotsLayout.addLayout(distancesLayout)
    plotsLayout.addLayout(colorLayout)
#    plotsLayout.addLayout(errorsLayout)
#    plotsLayout.addWidget(pressurePlotter)


    windowLayout.addLayout(plotsLayout)
    window.setLayout(windowLayout)
    window.show()

    print "Starting LCM..."

    lc.subscribe("_FD_O", handle_fiducial_state_high_rate_t)
    lc.subscribe("_FIDUCIAL_INCOMING_DATAVAL", handle_fiducial_color_dataval_t)
    lc.subscribe("_FIDUCIAL_UI_SET_ONE_ROW", handle_fiducial_set_color_row_t)
    lc.subscribe("_FIDUCIAL_CUR_TEACH_TABLE", handle_fiducial_cur_teach_table_t)

    start_lcm(lc)

    print "Running..."

    sys.exit(app.exec_())

