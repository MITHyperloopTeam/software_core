from director.consoleapp import ConsoleApp
from director import consoleapp
from director.timercallback import TimerCallback
from PythonQt import QtCore, QtGui
from collections import namedtuple
import time, math, threading, select

import lcm
from exlcm import example_t


FPS = 4
LABEL_DEFAULT_STYLE_SHEET = "font: 36pt; border-style: outset; border-width: 2px; border-color: black;"
TITLE_DEFAULT_STYLE_SHEET = "font: 24pt"

app = ConsoleApp()
app.setupGlobals(globals())
view = app.createView()

w = QtGui.QWidget()
l = QtGui.QHBoxLayout(w)
 
def spawnBasicButton(txt = "<>"):
  qs = QtGui.QPushButton(txt)
  qs.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
  qs.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET)
  return qs

def spawnBasicLabel(txt = "<>"):
  qs = QtGui.QLabel(txt)
  qs.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
  qs.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET)
  return qs

def spawnLineEdit():
  qs = QtGui.QLineEdit()
  qs.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
  qs.setStyleSheet(LABEL_DEFAULT_STYLE_SHEET)
  return qs

textbox = spawnLineEdit()
l.addWidget(textbox)

lc = lcm.LCM()

last_sent = time.time()
last_sent_msg = ""

def send (lc = lc, textbox=textbox):
  global last_sent, last_sent_msg
  msg = example_t()
  msg.timestamp = 0
  msg.position = (1,2,3)
  msg.orientation = (1,0,0,0)
  msg.ranges = range(15)
  msg.num_ranges = len(msg.ranges)
  msg.name = textbox.text
  msg.enabled = True

  lc.publish("TO_ARDUINO", msg.encode())
  last_sent = time.time()
  last_sent_msg = textbox.text
  print "Message Sent"

globalTimerSetBtn = spawnBasicButton("Send MSG to Pod")
l.addWidget(globalTimerSetBtn)

globalTimerSetBtn.connect('clicked()', send)

label = spawnBasicLabel("Received: <>")
l.addWidget(label)

def my_handler(channel, data):
  global label, last_sent, last_sent_msg
  msg = example_t.decode(data)
  print "Received message on channel \"%s\"" % channel
  " timestamp = %s" % str(msg.timestamp)
  " position = %s" % str(msg.position)
  " orientation = %s" % str(msg.orientation)
  " ranges: %s" % str(msg.ranges)
  " name = '%s'" % msg.name
  " enabled = %s" % str(msg.enabled)
  elapsed = time.time() - last_sent
  if (msg.name.strip() == last_sent_msg.strip()):
    label.setText("Received: \"" + msg.name.strip() + "\" after " + str(int(elapsed*1000)) + " ms")
  else:
    print "Received ", msg.name, " but not ", last_sent_msg
    label.setText("Received: \"" + msg.name.strip() + "\" out of order")

subscription = lc.subscribe("FROM_ARDUINO", my_handler)

w.setWindowTitle('Totally a Pod UI')
w.show()
w.resize(1000,100)

def lcmhandle():
  global lc
  rfds, wfds, efds = select.select([lc.fileno()], [], [], 0.001)
  if rfds:
    lc.handle()
timer = QtCore.QTimer()
timer.timeout.connect(lcmhandle)
timer.start(100)

t = threading.Thread(target=lcmhandle, args=())
t.start()
app.start()
