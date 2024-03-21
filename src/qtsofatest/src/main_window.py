#!/usr/bin/env python3

import sys
from PySide6.QtWidgets import QApplication, QMainWindow
from ui_mainwindow import Ui_MainWindow
from sofaqtviewer import SofaQtViewer

class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.sofaViewer = SofaQtViewer(self.ui.centralwidget)
        self.ui.openGLWidget.setParent(None)  # Hide the placeholder
        self.ui.centralwidget.layout().addWidget(self.sofaViewer)

def main():
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
