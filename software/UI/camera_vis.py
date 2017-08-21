import cv
import cv2
 
# the python stuff
import sys
import math
import signal

# numerics
import numpy as np

# the interface stuff
from PyQt4 import QtCore, QtGui
import pyqtgraph as pg

# the messaging stuff
import lcm
from bot_core import image_t
from lcm_utils import *

class OpenCVQImage(QtGui.QImage):
    def __init__(self, opencvBgrImg):
        depth, nChannels = opencvBgrImg.depth, opencvBgrImg.nChannels
        if depth != cv.IPL_DEPTH_8U or nChannels != 3:
            raise ValueError("the input image must be 8-bit, 3-channel")
        w, h = cv.GetSize(opencvBgrImg)
        opencvRgbImg = cv.CreateImage((w, h), depth, nChannels)
        # it's assumed the image is in BGR format
        cv.CvtColor(opencvBgrImg, opencvRgbImg, cv.CV_BGR2RGB)
        self._imgData = opencvRgbImg.tostring()
        super(OpenCVQImage, self).__init__(self._imgData, w, h, \
            QtGui.QImage.Format_RGB888)



class CameraVisWidget(QtGui.QWidget):
    ''' Displays camera data relayed over LCM'''

    def __init__(self, channel, lc=None, parent=None, name=None):
        super(CameraVisWidget, self).__init__(parent)

        self.lc = lc
        if name:
            self.setObjectName(name)

        self._frame = None       
        #self.setMinimumSize(640, 480)
        #self.setMaximumSize(1920, 1080)
 

        image_sub = self.lc.subscribe(channel, self.handle_image_t)

    def changeEvent(self, e):
        if e.type() == QtCore.QEvent.EnabledChange:
            if self.isEnabled():
                self._cameraDevice.newFrame.connect(self._onNewFrame)
            else:
                self._cameraDevice.newFrame.disconnect(self._onNewFrame)
    
    def paintEvent(self, e):
        if self._frame is None:
            return
        painter = QtGui.QPainter(self)
        contentsRect = self.contentsRect()
        w = contentsRect.width()
        h = contentsRect.height()
        painter.drawImage(QtCore.QPoint(0, 0), OpenCVQImage(self._frame).scaled(w, h, aspectRatioMode=QtCore.Qt.KeepAspectRatio))

    def handle_image_t(self, channel, data):
        msg = image_t.decode(data)

        # conversion black magic...
        if msg.pixelformat == image_t.PIXEL_FORMAT_MJPEG:
            source = cv2.imdecode(np.asarray(bytearray(msg.data), dtype="uint8"), flags=cv2.CV_LOAD_IMAGE_COLOR)
            frame =  cv.CreateImageHeader((source.shape[1], source.shape[0]), cv.IPL_DEPTH_8U, 3)
            cv.SetData(frame, source.tostring(), source.dtype.itemsize * 3 * source.shape[1])
            self._frame = cv.CloneImage(frame)
        else:
            print "Don't know how to handle this image type: ", msg.pixelformat

        self.update()

if __name__ == '__main__':
    # hook up interrupt signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    
    lc = create_lcm()

    app = QtGui.QApplication(sys.argv)
    window = CameraVisWidget("WEBCAM", lc=lc)
    window.show()

    start_lcm(lc)
    sys.exit(app.exec_())