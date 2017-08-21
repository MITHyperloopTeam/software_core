import sys
from PyQt4.QtGui import QApplication, QDialog
from ui_elems.ui_brake_state import Ui_BrakeStateWidget

app = QApplication(sys.argv)
window = QDialog()
ui = Ui_BrakeStateWidget()
ui.setupUi(window)

window.show()
sys.exit(app.exec_())
