# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'mainwindow.ui'
##
## Created by: Qt User Interface Compiler version 6.6.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtOpenGLWidgets import QOpenGLWidget
from PySide6.QtWidgets import (QApplication, QCheckBox, QComboBox, QDoubleSpinBox,
    QFrame, QGroupBox, QHBoxLayout, QLCDNumber,
    QLabel, QLayout, QMainWindow, QMenuBar,
    QPushButton, QScrollBar, QSizePolicy, QSlider,
    QSpinBox, QStatusBar, QToolBar, QVBoxLayout,
    QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(1692, 924)
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Preferred)
        sizePolicy.setHorizontalStretch(20)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(MainWindow.sizePolicy().hasHeightForWidth())
        MainWindow.setSizePolicy(sizePolicy)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.automatic_grasping_control_panel = QGroupBox(self.centralwidget)
        self.automatic_grasping_control_panel.setObjectName(u"automatic_grasping_control_panel")
        self.automatic_grasping_control_panel.setGeometry(QRect(420, 442, 331, 121))
        self.verticalLayoutWidget_2 = QWidget(self.automatic_grasping_control_panel)
        self.verticalLayoutWidget_2.setObjectName(u"verticalLayoutWidget_2")
        self.verticalLayoutWidget_2.setGeometry(QRect(10, 30, 311, 89))
        self.verticalLayout_2 = QVBoxLayout(self.verticalLayoutWidget_2)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_2 = QHBoxLayout()
        self.horizontalLayout_2.setSpacing(9)
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.horizontalLayout_2.setContentsMargins(6, -1, 6, -1)
        self.restart_button = QPushButton(self.verticalLayoutWidget_2)
        self.restart_button.setObjectName(u"restart_button")
        self.restart_button.setStyleSheet(u"background-color: lightgray")

        self.horizontalLayout_2.addWidget(self.restart_button)

        self.line_12 = QFrame(self.verticalLayoutWidget_2)
        self.line_12.setObjectName(u"line_12")
        self.line_12.setFrameShape(QFrame.VLine)
        self.line_12.setFrameShadow(QFrame.Sunken)

        self.horizontalLayout_2.addWidget(self.line_12)

        self.release_button = QPushButton(self.verticalLayoutWidget_2)
        self.release_button.setObjectName(u"release_button")
        self.release_button.setStyleSheet(u"background-color: lightgray")

        self.horizontalLayout_2.addWidget(self.release_button)


        self.verticalLayout_2.addLayout(self.horizontalLayout_2)

        self.hline1_6 = QFrame(self.verticalLayoutWidget_2)
        self.hline1_6.setObjectName(u"hline1_6")
        self.hline1_6.setFrameShape(QFrame.HLine)
        self.hline1_6.setFrameShadow(QFrame.Raised)

        self.verticalLayout_2.addWidget(self.hline1_6)

        self.horizontalLayout_3 = QHBoxLayout()
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.stop_all_button = QPushButton(self.verticalLayoutWidget_2)
        self.stop_all_button.setObjectName(u"stop_all_button")
        font = QFont()
        font.setPointSize(12)
        font.setBold(True)
        self.stop_all_button.setFont(font)
        self.stop_all_button.setStyleSheet(u"background-color: red;\n"
"	color: white;\n"
"    border: 2px solid darkred;\n"
"    border-radius: 10px;\n"
"    padding: 10px 20px;")

        self.horizontalLayout_3.addWidget(self.stop_all_button)


        self.verticalLayout_2.addLayout(self.horizontalLayout_3)

        self.verticalLayout_2.setStretch(0, 4)
        self.verticalLayout_2.setStretch(1, 1)
        self.verticalLayout_2.setStretch(2, 1)
        self.force_monitoring_panel = QGroupBox(self.centralwidget)
        self.force_monitoring_panel.setObjectName(u"force_monitoring_panel")
        self.force_monitoring_panel.setGeometry(QRect(840, 20, 391, 401))
        self.force_monitoring_panel.setFlat(False)
        self.force_monitoring_panel.setCheckable(False)
        self.line_19 = QFrame(self.force_monitoring_panel)
        self.line_19.setObjectName(u"line_19")
        self.line_19.setGeometry(QRect(60, 212, 291, 2))
        self.line_19.setStyleSheet(u"    border: none;\n"
"    background: qlineargradient(spread:pad, x1:0, x2:1, stop:0 rgba(179, 229, 252, 1), stop:0.5 rgba(0, 0, 0, 0), stop:1 rgba(255, 192, 192, 1));")
        self.line_19.setFrameShape(QFrame.HLine)
        self.line_19.setFrameShadow(QFrame.Sunken)
        self.force_val_1_graph = QFrame(self.force_monitoring_panel)
        self.force_val_1_graph.setObjectName(u"force_val_1_graph")
        self.force_val_1_graph.setGeometry(QRect(10, 30, 371, 184))
        sizePolicy1 = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Preferred)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.force_val_1_graph.sizePolicy().hasHeightForWidth())
        self.force_val_1_graph.setSizePolicy(sizePolicy1)
        self.force_val_1_graph.setAutoFillBackground(False)
        self.force_val_1_graph.setStyleSheet(u"background-color: rgba(0, 0, 0, 0); /* Fully transparent background */\n"
"    border: none; /* No border */")
        self.force_val_1_graph.setFrameShape(QFrame.NoFrame)
        self.force_val_1_graph.setFrameShadow(QFrame.Raised)
        self.force_val_1_graph.setLineWidth(0)
        self.force_val_2_graph = QFrame(self.force_monitoring_panel)
        self.force_val_2_graph.setObjectName(u"force_val_2_graph")
        self.force_val_2_graph.setGeometry(QRect(10, 213, 371, 186))
        self.force_val_2_graph.setStyleSheet(u"background-color: rgba(0, 0, 0, 0); /* Fully transparent background */\n"
"    border: none; /* No border */")
        self.force_val_2_graph.setFrameShape(QFrame.StyledPanel)
        self.force_val_2_graph.setFrameShadow(QFrame.Raised)
        self.force_val_1_graph.raise_()
        self.force_val_2_graph.raise_()
        self.line_19.raise_()
        self.tactile_monitoring_panel = QGroupBox(self.centralwidget)
        self.tactile_monitoring_panel.setObjectName(u"tactile_monitoring_panel")
        self.tactile_monitoring_panel.setGeometry(QRect(10, 20, 261, 401))
        self.layoutWidget = QWidget(self.tactile_monitoring_panel)
        self.layoutWidget.setObjectName(u"layoutWidget")
        self.layoutWidget.setGeometry(QRect(10, 30, 239, 361))
        self.verticalLayout_3 = QVBoxLayout(self.layoutWidget)
        self.verticalLayout_3.setSpacing(6)
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.verticalLayout_3.setSizeConstraint(QLayout.SetDefaultConstraint)
        self.verticalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.S1_tactile_monitoring_label = QLabel(self.layoutWidget)
        self.S1_tactile_monitoring_label.setObjectName(u"S1_tactile_monitoring_label")
        self.S1_tactile_monitoring_label.setAlignment(Qt.AlignBottom|Qt.AlignHCenter)
        self.S1_tactile_monitoring_label.setMargin(0)

        self.verticalLayout_3.addWidget(self.S1_tactile_monitoring_label)

        self.horizontalLayout_11 = QHBoxLayout()
        self.horizontalLayout_11.setObjectName(u"horizontalLayout_11")
        self.widget1 = QWidget(self.layoutWidget)
        self.widget1.setObjectName(u"widget1")

        self.horizontalLayout_11.addWidget(self.widget1)

        self.force_value_1_x = QLabel(self.layoutWidget)
        self.force_value_1_x.setObjectName(u"force_value_1_x")
        font1 = QFont()
        font1.setPointSize(12)
        self.force_value_1_x.setFont(font1)
        self.force_value_1_x.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_11.addWidget(self.force_value_1_x)

        self.force_val_1_x = QLCDNumber(self.layoutWidget)
        self.force_val_1_x.setObjectName(u"force_val_1_x")
        font2 = QFont()
        font2.setFamilies([u"Ubuntu"])
        font2.setPointSize(11)
        font2.setBold(True)
        font2.setItalic(False)
        self.force_val_1_x.setFont(font2)
        self.force_val_1_x.setAutoFillBackground(False)
        self.force_val_1_x.setStyleSheet(u"background-color: rgba(179, 229, 252, 0.35);\n"
"color: rgba(0, 0, 139, 1); /* Dark blue color */\n"
"font-weight: bold;")
        self.force_val_1_x.setFrameShape(QFrame.WinPanel)
        self.force_val_1_x.setFrameShadow(QFrame.Raised)
        self.force_val_1_x.setLineWidth(1)
        self.force_val_1_x.setMidLineWidth(0)
        self.force_val_1_x.setSmallDecimalPoint(True)
        self.force_val_1_x.setDigitCount(3)
        self.force_val_1_x.setSegmentStyle(QLCDNumber.Flat)
        self.force_val_1_x.setProperty("value", 0.000000000000000)

        self.horizontalLayout_11.addWidget(self.force_val_1_x)

        self.widget11 = QWidget(self.layoutWidget)
        self.widget11.setObjectName(u"widget11")

        self.horizontalLayout_11.addWidget(self.widget11)

        self.force_value_1_y = QLabel(self.layoutWidget)
        self.force_value_1_y.setObjectName(u"force_value_1_y")
        self.force_value_1_y.setFont(font1)
        self.force_value_1_y.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_11.addWidget(self.force_value_1_y)

        self.force_val_1_y = QLCDNumber(self.layoutWidget)
        self.force_val_1_y.setObjectName(u"force_val_1_y")
        self.force_val_1_y.setFont(font2)
        self.force_val_1_y.setAutoFillBackground(False)
        self.force_val_1_y.setStyleSheet(u"background-color: rgba(179, 229, 252, 0.35);\n"
"color: rgba(0, 0, 139, 1); /* Dark blue color */\n"
"font-weight: bold;")
        self.force_val_1_y.setFrameShape(QFrame.WinPanel)
        self.force_val_1_y.setFrameShadow(QFrame.Raised)
        self.force_val_1_y.setLineWidth(1)
        self.force_val_1_y.setMidLineWidth(0)
        self.force_val_1_y.setSmallDecimalPoint(True)
        self.force_val_1_y.setDigitCount(3)
        self.force_val_1_y.setSegmentStyle(QLCDNumber.Flat)
        self.force_val_1_y.setProperty("value", 0.000000000000000)

        self.horizontalLayout_11.addWidget(self.force_val_1_y)

        self.widget_10 = QWidget(self.layoutWidget)
        self.widget_10.setObjectName(u"widget_10")

        self.horizontalLayout_11.addWidget(self.widget_10)

        self.force_value_1_z = QLabel(self.layoutWidget)
        self.force_value_1_z.setObjectName(u"force_value_1_z")
        self.force_value_1_z.setFont(font1)
        self.force_value_1_z.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_11.addWidget(self.force_value_1_z)

        self.force_val_1_z = QLCDNumber(self.layoutWidget)
        self.force_val_1_z.setObjectName(u"force_val_1_z")
        self.force_val_1_z.setFont(font2)
        self.force_val_1_z.setAutoFillBackground(False)
        self.force_val_1_z.setStyleSheet(u"background-color: rgba(179, 229, 252, 0.35);\n"
"color: rgba(0, 0, 139, 1); /* Dark blue color */\n"
"font-weight: bold;")
        self.force_val_1_z.setFrameShape(QFrame.WinPanel)
        self.force_val_1_z.setFrameShadow(QFrame.Raised)
        self.force_val_1_z.setLineWidth(1)
        self.force_val_1_z.setMidLineWidth(0)
        self.force_val_1_z.setSmallDecimalPoint(True)
        self.force_val_1_z.setDigitCount(3)
        self.force_val_1_z.setSegmentStyle(QLCDNumber.Flat)
        self.force_val_1_z.setProperty("value", 0.000000000000000)

        self.horizontalLayout_11.addWidget(self.force_val_1_z)

        self.widget_14 = QWidget(self.layoutWidget)
        self.widget_14.setObjectName(u"widget_14")

        self.horizontalLayout_11.addWidget(self.widget_14)

        self.horizontalLayout_11.setStretch(0, 1)
        self.horizontalLayout_11.setStretch(1, 1)
        self.horizontalLayout_11.setStretch(2, 1)
        self.horizontalLayout_11.setStretch(3, 1)
        self.horizontalLayout_11.setStretch(4, 1)
        self.horizontalLayout_11.setStretch(5, 1)
        self.horizontalLayout_11.setStretch(6, 1)
        self.horizontalLayout_11.setStretch(7, 1)
        self.horizontalLayout_11.setStretch(8, 1)
        self.horizontalLayout_11.setStretch(9, 1)

        self.verticalLayout_3.addLayout(self.horizontalLayout_11)

        self.hline1_2 = QFrame(self.layoutWidget)
        self.hline1_2.setObjectName(u"hline1_2")
        self.hline1_2.setFrameShape(QFrame.HLine)
        self.hline1_2.setFrameShadow(QFrame.Raised)

        self.verticalLayout_3.addWidget(self.hline1_2)

        self.horizontalLayout_9 = QHBoxLayout()
        self.horizontalLayout_9.setObjectName(u"horizontalLayout_9")
        self.quadrant_1 = QWidget(self.layoutWidget)
        self.quadrant_1.setObjectName(u"quadrant_1")
        sizePolicy2 = QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        sizePolicy2.setHorizontalStretch(0)
        sizePolicy2.setVerticalStretch(0)
        sizePolicy2.setHeightForWidth(self.quadrant_1.sizePolicy().hasHeightForWidth())
        self.quadrant_1.setSizePolicy(sizePolicy2)
        self.quadrant_1.setStyleSheet(u"background-color: rgba(0, 0, 0, 0); /* Fully transparent background */\n"
"    border: none; /* No border */")
        self.quadrant_1_1 = QWidget(self.quadrant_1)
        self.quadrant_1_1.setObjectName(u"quadrant_1_1")
        self.quadrant_1_1.setGeometry(QRect(-32, -33, 171, 171))
        self.quadrant_1_1.setStyleSheet(u"background-color: rgba(0, 0, 0, 0); /* Fully transparent background */\n"
"    border: none; /* No border */")

        self.horizontalLayout_9.addWidget(self.quadrant_1)

        self.line_7 = QFrame(self.layoutWidget)
        self.line_7.setObjectName(u"line_7")
        self.line_7.setFrameShape(QFrame.VLine)
        self.line_7.setFrameShadow(QFrame.Sunken)

        self.horizontalLayout_9.addWidget(self.line_7)

        self.shear_1 = QWidget(self.layoutWidget)
        self.shear_1.setObjectName(u"shear_1")
        self.shear_1.setStyleSheet(u"background-color: rgba(0, 0, 0, 0); /* Fully transparent background */\n"
"    border: none; /* No border */")
        self.shear_map_1_1 = QWidget(self.shear_1)
        self.shear_map_1_1.setObjectName(u"shear_map_1_1")
        self.shear_map_1_1.setGeometry(QRect(-36, -30, 171, 171))
        self.shear_map_1_1.setStyleSheet(u"background-color: rgba(0, 0, 0, 0); /* Fully transparent background */\n"
"    border: none; /* No border */")

        self.horizontalLayout_9.addWidget(self.shear_1)

        self.horizontalLayout_9.setStretch(0, 1)
        self.horizontalLayout_9.setStretch(2, 1)

        self.verticalLayout_3.addLayout(self.horizontalLayout_9)

        self.hline1 = QFrame(self.layoutWidget)
        self.hline1.setObjectName(u"hline1")
        self.hline1.setFrameShape(QFrame.HLine)
        self.hline1.setFrameShadow(QFrame.Raised)

        self.verticalLayout_3.addWidget(self.hline1)

        self.S2_tactile_monitoring_label = QLabel(self.layoutWidget)
        self.S2_tactile_monitoring_label.setObjectName(u"S2_tactile_monitoring_label")
        self.S2_tactile_monitoring_label.setAlignment(Qt.AlignCenter)

        self.verticalLayout_3.addWidget(self.S2_tactile_monitoring_label)

        self.horizontalLayout_13 = QHBoxLayout()
        self.horizontalLayout_13.setObjectName(u"horizontalLayout_13")
        self.widget_15 = QWidget(self.layoutWidget)
        self.widget_15.setObjectName(u"widget_15")

        self.horizontalLayout_13.addWidget(self.widget_15)

        self.force_value_2_x = QLabel(self.layoutWidget)
        self.force_value_2_x.setObjectName(u"force_value_2_x")
        self.force_value_2_x.setFont(font1)
        self.force_value_2_x.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_13.addWidget(self.force_value_2_x)

        self.force_val_2_x = QLCDNumber(self.layoutWidget)
        self.force_val_2_x.setObjectName(u"force_val_2_x")
        self.force_val_2_x.setFont(font2)
        self.force_val_2_x.setAutoFillBackground(False)
        self.force_val_2_x.setStyleSheet(u"background-color: rgba(255, 192, 192, 0.3);\n"
"color: rgba(0, 0, 139, 1); /* Dark blue color */\n"
"font-weight: bold;")
        self.force_val_2_x.setFrameShape(QFrame.WinPanel)
        self.force_val_2_x.setFrameShadow(QFrame.Raised)
        self.force_val_2_x.setLineWidth(1)
        self.force_val_2_x.setMidLineWidth(0)
        self.force_val_2_x.setSmallDecimalPoint(True)
        self.force_val_2_x.setDigitCount(3)
        self.force_val_2_x.setSegmentStyle(QLCDNumber.Flat)
        self.force_val_2_x.setProperty("value", 0.000000000000000)

        self.horizontalLayout_13.addWidget(self.force_val_2_x)

        self.widget_17 = QWidget(self.layoutWidget)
        self.widget_17.setObjectName(u"widget_17")

        self.horizontalLayout_13.addWidget(self.widget_17)

        self.force_value_2_y = QLabel(self.layoutWidget)
        self.force_value_2_y.setObjectName(u"force_value_2_y")
        self.force_value_2_y.setFont(font1)
        self.force_value_2_y.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_13.addWidget(self.force_value_2_y)

        self.force_val_2_y = QLCDNumber(self.layoutWidget)
        self.force_val_2_y.setObjectName(u"force_val_2_y")
        self.force_val_2_y.setFont(font2)
        self.force_val_2_y.setAutoFillBackground(False)
        self.force_val_2_y.setStyleSheet(u"background-color: rgba(255, 192, 192, 0.3);\n"
"color: rgba(0, 0, 139, 1); /* Dark blue color */\n"
"font-weight: bold;")
        self.force_val_2_y.setFrameShape(QFrame.WinPanel)
        self.force_val_2_y.setFrameShadow(QFrame.Raised)
        self.force_val_2_y.setLineWidth(1)
        self.force_val_2_y.setMidLineWidth(0)
        self.force_val_2_y.setSmallDecimalPoint(True)
        self.force_val_2_y.setDigitCount(3)
        self.force_val_2_y.setSegmentStyle(QLCDNumber.Flat)
        self.force_val_2_y.setProperty("value", 0.000000000000000)

        self.horizontalLayout_13.addWidget(self.force_val_2_y)

        self.widget_18 = QWidget(self.layoutWidget)
        self.widget_18.setObjectName(u"widget_18")

        self.horizontalLayout_13.addWidget(self.widget_18)

        self.force_value_2_z = QLabel(self.layoutWidget)
        self.force_value_2_z.setObjectName(u"force_value_2_z")
        self.force_value_2_z.setFont(font1)
        self.force_value_2_z.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_13.addWidget(self.force_value_2_z)

        self.force_val_2_z = QLCDNumber(self.layoutWidget)
        self.force_val_2_z.setObjectName(u"force_val_2_z")
        font3 = QFont()
        font3.setFamilies([u"Ubuntu"])
        font3.setPointSize(12)
        font3.setBold(True)
        font3.setItalic(False)
        self.force_val_2_z.setFont(font3)
        self.force_val_2_z.setStyleSheet(u"background-color: rgba(255, 192, 192, 0.3);\n"
"color: rgba(0, 0, 139, 1); /* Dark blue color */\n"
"font-weight: bold;")
        self.force_val_2_z.setFrameShape(QFrame.WinPanel)
        self.force_val_2_z.setSmallDecimalPoint(True)
        self.force_val_2_z.setDigitCount(3)
        self.force_val_2_z.setSegmentStyle(QLCDNumber.Flat)
        self.force_val_2_z.setProperty("value", 0.000000000000000)

        self.horizontalLayout_13.addWidget(self.force_val_2_z)

        self.widget_16 = QWidget(self.layoutWidget)
        self.widget_16.setObjectName(u"widget_16")

        self.horizontalLayout_13.addWidget(self.widget_16)

        self.horizontalLayout_13.setStretch(0, 1)
        self.horizontalLayout_13.setStretch(1, 1)
        self.horizontalLayout_13.setStretch(2, 1)
        self.horizontalLayout_13.setStretch(4, 1)
        self.horizontalLayout_13.setStretch(5, 1)
        self.horizontalLayout_13.setStretch(7, 1)
        self.horizontalLayout_13.setStretch(8, 1)
        self.horizontalLayout_13.setStretch(9, 1)

        self.verticalLayout_3.addLayout(self.horizontalLayout_13)

        self.hline1_5 = QFrame(self.layoutWidget)
        self.hline1_5.setObjectName(u"hline1_5")
        self.hline1_5.setFrameShape(QFrame.HLine)
        self.hline1_5.setFrameShadow(QFrame.Raised)

        self.verticalLayout_3.addWidget(self.hline1_5)

        self.horizontalLayout_19 = QHBoxLayout()
        self.horizontalLayout_19.setObjectName(u"horizontalLayout_19")
        self.quadrant_2 = QWidget(self.layoutWidget)
        self.quadrant_2.setObjectName(u"quadrant_2")
        sizePolicy2.setHeightForWidth(self.quadrant_2.sizePolicy().hasHeightForWidth())
        self.quadrant_2.setSizePolicy(sizePolicy2)
        self.quadrant_2.setSizeIncrement(QSize(9, 9))
        self.quadrant_2.setStyleSheet(u"background-color: rgba(0, 0, 0, 0); /* Fully transparent background */\n"
"    border: none; /* No border */")
        self.quadrant_2_2 = QWidget(self.quadrant_2)
        self.quadrant_2_2.setObjectName(u"quadrant_2_2")
        self.quadrant_2_2.setGeometry(QRect(-32, -28, 171, 171))
        self.quadrant_2_2.setStyleSheet(u"background-color: rgba(0, 0, 0, 0); /* Fully transparent background */\n"
"    border: none; /* No border */")

        self.horizontalLayout_19.addWidget(self.quadrant_2)

        self.line_16 = QFrame(self.layoutWidget)
        self.line_16.setObjectName(u"line_16")
        self.line_16.setFrameShape(QFrame.VLine)
        self.line_16.setFrameShadow(QFrame.Sunken)

        self.horizontalLayout_19.addWidget(self.line_16)

        self.shear_2 = QWidget(self.layoutWidget)
        self.shear_2.setObjectName(u"shear_2")
        self.shear_2.setStyleSheet(u"background-color: rgba(0, 0, 0, 0); /* Fully transparent background */\n"
"    border: none; /* No border */")
        self.shear_map_2_2 = QWidget(self.shear_2)
        self.shear_map_2_2.setObjectName(u"shear_map_2_2")
        self.shear_map_2_2.setGeometry(QRect(-36, -30, 171, 171))
        self.shear_map_2_2.setStyleSheet(u"background-color: rgba(0, 0, 0, 0); /* Fully transparent background */\n"
"    border: none; /* No border */")

        self.horizontalLayout_19.addWidget(self.shear_2)

        self.horizontalLayout_19.setStretch(0, 1)
        self.horizontalLayout_19.setStretch(2, 1)

        self.verticalLayout_3.addLayout(self.horizontalLayout_19)

        self.verticalLayout_3.setStretch(3, 3)
        self.verticalLayout_3.setStretch(4, 1)
        self.verticalLayout_3.setStretch(8, 3)
        self.sensing_signal_monitoring_panel = QGroupBox(self.centralwidget)
        self.sensing_signal_monitoring_panel.setObjectName(u"sensing_signal_monitoring_panel")
        self.sensing_signal_monitoring_panel.setGeometry(QRect(770, 442, 901, 411))
        self.horizontalLayoutWidget = QWidget(self.sensing_signal_monitoring_panel)
        self.horizontalLayoutWidget.setObjectName(u"horizontalLayoutWidget")
        self.horizontalLayoutWidget.setGeometry(QRect(480, 30, 411, 381))
        self.horizontalLayout = QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.pressure_graph = QFrame(self.horizontalLayoutWidget)
        self.pressure_graph.setObjectName(u"pressure_graph")
        self.pressure_graph.setStyleSheet(u"background-color: rgba(0, 0, 0, 0); /* Fully transparent background */\n"
"    border: none; /* No border */")
        self.pressure_graph.setFrameShape(QFrame.NoFrame)
        self.pressure_graph.setFrameShadow(QFrame.Raised)

        self.horizontalLayout.addWidget(self.pressure_graph)

        self.verticalLayoutWidget_7 = QWidget(self.sensing_signal_monitoring_panel)
        self.verticalLayoutWidget_7.setObjectName(u"verticalLayoutWidget_7")
        self.verticalLayoutWidget_7.setGeometry(QRect(10, 30, 451, 191))
        self.verticalLayout_10 = QVBoxLayout(self.verticalLayoutWidget_7)
        self.verticalLayout_10.setObjectName(u"verticalLayout_10")
        self.verticalLayout_10.setContentsMargins(0, 0, 0, 0)
        self.S1_graph = QFrame(self.verticalLayoutWidget_7)
        self.S1_graph.setObjectName(u"S1_graph")
        self.S1_graph.setStyleSheet(u"background-color: rgba(0, 0, 0, 0); /* Fully transparent background */\n"
"    border: none; /* No border */")
        self.S1_graph.setFrameShape(QFrame.NoFrame)
        self.S1_graph.setFrameShadow(QFrame.Raised)
        self.S1_graph.setLineWidth(0)

        self.verticalLayout_10.addWidget(self.S1_graph)

        self.horizontalLayoutWidget_8 = QWidget(self.sensing_signal_monitoring_panel)
        self.horizontalLayoutWidget_8.setObjectName(u"horizontalLayoutWidget_8")
        self.horizontalLayoutWidget_8.setGeometry(QRect(10, 220, 451, 191))
        self.horizontalLayout_22 = QHBoxLayout(self.horizontalLayoutWidget_8)
        self.horizontalLayout_22.setObjectName(u"horizontalLayout_22")
        self.horizontalLayout_22.setContentsMargins(0, 0, 0, 0)
        self.S2_graph = QFrame(self.horizontalLayoutWidget_8)
        self.S2_graph.setObjectName(u"S2_graph")
        self.S2_graph.setStyleSheet(u"background-color: rgba(0, 0, 0, 0); /* Fully transparent background */\n"
"    border: none; /* No border */")
        self.S2_graph.setFrameShape(QFrame.NoFrame)
        self.S2_graph.setFrameShadow(QFrame.Raised)

        self.horizontalLayout_22.addWidget(self.S2_graph)

        self.line_17 = QFrame(self.sensing_signal_monitoring_panel)
        self.line_17.setObjectName(u"line_17")
        self.line_17.setGeometry(QRect(67, 220, 381, 2))
        self.line_17.setStyleSheet(u"    border: none;\n"
"    background: qlineargradient(spread:pad, x1:0, x2:1, stop:0 rgba(179, 229, 252, 1), stop:0.5 rgba(0, 0, 0, 0), stop:1 rgba(255, 192, 192, 1));")
        self.line_17.setFrameShape(QFrame.HLine)
        self.line_17.setFrameShadow(QFrame.Sunken)
        self.line_18 = QFrame(self.sensing_signal_monitoring_panel)
        self.line_18.setObjectName(u"line_18")
        self.line_18.setGeometry(QRect(470, 30, 2, 371))
        self.line_18.setStyleSheet(u"    border: none;\n"
"    background: qlineargradient(spread:pad, y1:0, y2:1,stop:0 rgba(0, 0, 0, 0), stop:0.25 rgba(179, 229, 252, 1), stop:0.5 rgba(0, 0, 0, 0), stop:0.75 rgba(255, 192, 192, 1), stop:1 rgba(0, 0, 0, 0));")
        self.line_18.setFrameShape(QFrame.VLine)
        self.line_18.setFrameShadow(QFrame.Sunken)
        self.status_monitoring_panel_2 = QGroupBox(self.centralwidget)
        self.status_monitoring_panel_2.setObjectName(u"status_monitoring_panel_2")
        self.status_monitoring_panel_2.setGeometry(QRect(420, 572, 331, 281))
        self.verticalLayoutWidget_6 = QWidget(self.status_monitoring_panel_2)
        self.verticalLayoutWidget_6.setObjectName(u"verticalLayoutWidget_6")
        self.verticalLayoutWidget_6.setGeometry(QRect(10, 30, 311, 241))
        self.verticalLayout_7 = QVBoxLayout(self.verticalLayoutWidget_6)
        self.verticalLayout_7.setObjectName(u"verticalLayout_7")
        self.verticalLayout_7.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_9 = QVBoxLayout()
        self.verticalLayout_9.setObjectName(u"verticalLayout_9")
        self.hline1_10 = QFrame(self.verticalLayoutWidget_6)
        self.hline1_10.setObjectName(u"hline1_10")
        self.hline1_10.setFrameShape(QFrame.HLine)
        self.hline1_10.setFrameShadow(QFrame.Raised)

        self.verticalLayout_9.addWidget(self.hline1_10)

        self.verticalLayout_11 = QVBoxLayout()
        self.verticalLayout_11.setObjectName(u"verticalLayout_11")
        self.widget_2 = QWidget(self.verticalLayoutWidget_6)
        self.widget_2.setObjectName(u"widget_2")

        self.verticalLayout_11.addWidget(self.widget_2)


        self.verticalLayout_9.addLayout(self.verticalLayout_11)

        self.horizontalLayout_16 = QHBoxLayout()
        self.horizontalLayout_16.setSpacing(9)
        self.horizontalLayout_16.setObjectName(u"horizontalLayout_16")
        self.motor_pos_reading_bar_1 = QSlider(self.verticalLayoutWidget_6)
        self.motor_pos_reading_bar_1.setObjectName(u"motor_pos_reading_bar_1")
        self.motor_pos_reading_bar_1.setOrientation(Qt.Horizontal)

        self.horizontalLayout_16.addWidget(self.motor_pos_reading_bar_1)

        self.widget_7 = QWidget(self.verticalLayoutWidget_6)
        self.widget_7.setObjectName(u"widget_7")

        self.horizontalLayout_16.addWidget(self.widget_7)

        self.motor_pos_reading_label = QLabel(self.verticalLayoutWidget_6)
        self.motor_pos_reading_label.setObjectName(u"motor_pos_reading_label")
        self.motor_pos_reading_label.setScaledContents(True)
        self.motor_pos_reading_label.setAlignment(Qt.AlignCenter)
        self.motor_pos_reading_label.setWordWrap(True)

        self.horizontalLayout_16.addWidget(self.motor_pos_reading_label)

        self.motor_pos_reading_bar_2 = QSlider(self.verticalLayoutWidget_6)
        self.motor_pos_reading_bar_2.setObjectName(u"motor_pos_reading_bar_2")
        self.motor_pos_reading_bar_2.setOrientation(Qt.Horizontal)
        self.motor_pos_reading_bar_2.setInvertedAppearance(True)

        self.horizontalLayout_16.addWidget(self.motor_pos_reading_bar_2)

        self.horizontalLayout_16.setStretch(0, 2)
        self.horizontalLayout_16.setStretch(2, 1)
        self.horizontalLayout_16.setStretch(3, 2)

        self.verticalLayout_9.addLayout(self.horizontalLayout_16)

        self.verticalLayout_14 = QVBoxLayout()
        self.verticalLayout_14.setObjectName(u"verticalLayout_14")
        self.widget_8 = QWidget(self.verticalLayoutWidget_6)
        self.widget_8.setObjectName(u"widget_8")

        self.verticalLayout_14.addWidget(self.widget_8)


        self.verticalLayout_9.addLayout(self.verticalLayout_14)

        self.hline1_11 = QFrame(self.verticalLayoutWidget_6)
        self.hline1_11.setObjectName(u"hline1_11")
        self.hline1_11.setFrameShape(QFrame.HLine)
        self.hline1_11.setFrameShadow(QFrame.Raised)

        self.verticalLayout_9.addWidget(self.hline1_11)

        self.verticalLayout_9.setStretch(0, 1)
        self.verticalLayout_9.setStretch(1, 1)
        self.verticalLayout_9.setStretch(2, 5)
        self.verticalLayout_9.setStretch(3, 1)
        self.verticalLayout_9.setStretch(4, 1)

        self.verticalLayout_7.addLayout(self.verticalLayout_9)

        self.verticalLayout_17 = QVBoxLayout()
        self.verticalLayout_17.setObjectName(u"verticalLayout_17")
        self.horizontalLayout_17 = QHBoxLayout()
        self.horizontalLayout_17.setSpacing(0)
        self.horizontalLayout_17.setObjectName(u"horizontalLayout_17")
        self.horizontalLayout_17.setContentsMargins(0, 0, 0, 0)
        self.line_14 = QFrame(self.verticalLayoutWidget_6)
        self.line_14.setObjectName(u"line_14")
        self.line_14.setFrameShape(QFrame.VLine)
        self.line_14.setFrameShadow(QFrame.Sunken)

        self.horizontalLayout_17.addWidget(self.line_14)

        self.data_stable_flag_indicator = QLabel(self.verticalLayoutWidget_6)
        self.data_stable_flag_indicator.setObjectName(u"data_stable_flag_indicator")
        font4 = QFont()
        font4.setPointSize(12)
        font4.setBold(False)
        self.data_stable_flag_indicator.setFont(font4)
        self.data_stable_flag_indicator.setStyleSheet(u"background-color: lightgray; /* Light gray background */\n"
"    color: black; /* Black text */\n"
"    border: 2px solid gray; /* Gray border */\n"
"    border-radius: 5px; /* Rounded corners */\n"
"    padding: 5px; /* Padding */\n"
"    min-width: 100px; /* Minimum width */\n"
"    text-align: center; /* Center text alignment */")
        self.data_stable_flag_indicator.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_17.addWidget(self.data_stable_flag_indicator)

        self.line_2 = QFrame(self.verticalLayoutWidget_6)
        self.line_2.setObjectName(u"line_2")
        self.line_2.setFrameShape(QFrame.VLine)
        self.line_2.setFrameShadow(QFrame.Sunken)

        self.horizontalLayout_17.addWidget(self.line_2)

        self.line_6 = QFrame(self.verticalLayoutWidget_6)
        self.line_6.setObjectName(u"line_6")
        self.line_6.setFrameShape(QFrame.VLine)
        self.line_6.setFrameShadow(QFrame.Sunken)

        self.horizontalLayout_17.addWidget(self.line_6)

        self.contact_detect_indicator = QLabel(self.verticalLayoutWidget_6)
        self.contact_detect_indicator.setObjectName(u"contact_detect_indicator")
        self.contact_detect_indicator.setFont(font1)
        self.contact_detect_indicator.setStyleSheet(u"background-color: lightgray; /* Light gray background */\n"
"    color: black; /* Black text */\n"
"    border: 2px solid gray; /* Gray border */\n"
"    border-radius: 5px; /* Rounded corners */\n"
"    padding: 5px; /* Padding */\n"
"    min-width: 100px; /* Minimum width */\n"
"    text-align: center; /* Center text alignment */")
        self.contact_detect_indicator.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_17.addWidget(self.contact_detect_indicator)

        self.line_8 = QFrame(self.verticalLayoutWidget_6)
        self.line_8.setObjectName(u"line_8")
        self.line_8.setFrameShape(QFrame.VLine)
        self.line_8.setFrameShadow(QFrame.Sunken)

        self.horizontalLayout_17.addWidget(self.line_8)

        self.horizontalLayout_17.setStretch(1, 1)
        self.horizontalLayout_17.setStretch(4, 1)
        self.horizontalLayout_17.setStretch(5, 1)

        self.verticalLayout_17.addLayout(self.horizontalLayout_17)

        self.line_4 = QFrame(self.verticalLayoutWidget_6)
        self.line_4.setObjectName(u"line_4")
        self.line_4.setFrameShape(QFrame.HLine)
        self.line_4.setFrameShadow(QFrame.Sunken)

        self.verticalLayout_17.addWidget(self.line_4)

        self.horizontalLayout_18 = QHBoxLayout()
        self.horizontalLayout_18.setSpacing(0)
        self.horizontalLayout_18.setObjectName(u"horizontalLayout_18")
        self.horizontalLayout_18.setContentsMargins(0, 0, 0, 0)
        self.line_11 = QFrame(self.verticalLayoutWidget_6)
        self.line_11.setObjectName(u"line_11")
        self.line_11.setFrameShape(QFrame.VLine)
        self.line_11.setFrameShadow(QFrame.Sunken)

        self.horizontalLayout_18.addWidget(self.line_11)

        self.grasp_stable_flag_indicator = QLabel(self.verticalLayoutWidget_6)
        self.grasp_stable_flag_indicator.setObjectName(u"grasp_stable_flag_indicator")
        self.grasp_stable_flag_indicator.setStyleSheet(u"background-color: lightgray; /* Light gray background */\n"
"    color: black; /* Black text */\n"
"    border: 2px solid gray; /* Gray border */\n"
"    border-radius: 5px; /* Rounded corners */\n"
"    padding: 5px; /* Padding */\n"
"    min-width: 100px; /* Minimum width */\n"
"    text-align: center; /* Center text alignment */")
        self.grasp_stable_flag_indicator.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_18.addWidget(self.grasp_stable_flag_indicator)

        self.line_3 = QFrame(self.verticalLayoutWidget_6)
        self.line_3.setObjectName(u"line_3")
        self.line_3.setFrameShape(QFrame.VLine)
        self.line_3.setFrameShadow(QFrame.Sunken)

        self.horizontalLayout_18.addWidget(self.line_3)

        self.line_9 = QFrame(self.verticalLayoutWidget_6)
        self.line_9.setObjectName(u"line_9")
        self.line_9.setFrameShape(QFrame.VLine)
        self.line_9.setFrameShadow(QFrame.Sunken)

        self.horizontalLayout_18.addWidget(self.line_9)

        self.motor_stop_indicator = QLabel(self.verticalLayoutWidget_6)
        self.motor_stop_indicator.setObjectName(u"motor_stop_indicator")
        self.motor_stop_indicator.setAutoFillBackground(False)
        self.motor_stop_indicator.setStyleSheet(u"background-color: lightcyan;\n"
"color: black;\n"
"border: 2px solid #00ced1; /* Dark Cyan border */\n"
"border-radius: 5px;\n"
"padding: 5px;\n"
"min-width: 100px;\n"
"text-align: center;")
        self.motor_stop_indicator.setScaledContents(False)
        self.motor_stop_indicator.setAlignment(Qt.AlignCenter)
        self.motor_stop_indicator.setMargin(0)

        self.horizontalLayout_18.addWidget(self.motor_stop_indicator)

        self.line_10 = QFrame(self.verticalLayoutWidget_6)
        self.line_10.setObjectName(u"line_10")
        self.line_10.setFrameShape(QFrame.VLine)
        self.line_10.setFrameShadow(QFrame.Sunken)

        self.horizontalLayout_18.addWidget(self.line_10)


        self.verticalLayout_17.addLayout(self.horizontalLayout_18)


        self.verticalLayout_7.addLayout(self.verticalLayout_17)

        self.hline1_4 = QFrame(self.verticalLayoutWidget_6)
        self.hline1_4.setObjectName(u"hline1_4")
        self.hline1_4.setFrameShape(QFrame.HLine)
        self.hline1_4.setFrameShadow(QFrame.Raised)

        self.verticalLayout_7.addWidget(self.hline1_4)

        self.horizontalLayout_6 = QHBoxLayout()
        self.horizontalLayout_6.setObjectName(u"horizontalLayout_6")
        self.elapsed_time_label = QLabel(self.verticalLayoutWidget_6)
        self.elapsed_time_label.setObjectName(u"elapsed_time_label")
        self.elapsed_time_label.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_6.addWidget(self.elapsed_time_label)

        self.elapsed_time_display = QLCDNumber(self.verticalLayoutWidget_6)
        self.elapsed_time_display.setObjectName(u"elapsed_time_display")
        self.elapsed_time_display.setDigitCount(8)

        self.horizontalLayout_6.addWidget(self.elapsed_time_display)

        self.time_reset_button = QPushButton(self.verticalLayoutWidget_6)
        self.time_reset_button.setObjectName(u"time_reset_button")
        self.time_reset_button.setStyleSheet(u"background-color: lightgray")

        self.horizontalLayout_6.addWidget(self.time_reset_button)

        self.horizontalLayout_6.setStretch(0, 3)
        self.horizontalLayout_6.setStretch(1, 3)
        self.horizontalLayout_6.setStretch(2, 1)

        self.verticalLayout_7.addLayout(self.horizontalLayout_6)

        self.verticalLayout_7.setStretch(0, 3)
        self.verticalLayout_7.setStretch(1, 3)
        self.verticalLayout_7.setStretch(2, 1)
        self.verticalLayout_7.setStretch(3, 1)
        self.testing_control_panel = QGroupBox(self.centralwidget)
        self.testing_control_panel.setObjectName(u"testing_control_panel")
        self.testing_control_panel.setGeometry(QRect(10, 522, 391, 331))
        self.verticalLayoutWidget_9 = QWidget(self.testing_control_panel)
        self.verticalLayoutWidget_9.setObjectName(u"verticalLayoutWidget_9")
        self.verticalLayoutWidget_9.setGeometry(QRect(10, 30, 371, 291))
        sizePolicy3 = QSizePolicy(QSizePolicy.Policy.Maximum, QSizePolicy.Policy.Maximum)
        sizePolicy3.setHorizontalStretch(0)
        sizePolicy3.setVerticalStretch(0)
        sizePolicy3.setHeightForWidth(self.verticalLayoutWidget_9.sizePolicy().hasHeightForWidth())
        self.verticalLayoutWidget_9.setSizePolicy(sizePolicy3)
        self.testing_control_panel_layout = QVBoxLayout(self.verticalLayoutWidget_9)
        self.testing_control_panel_layout.setObjectName(u"testing_control_panel_layout")
        self.testing_control_panel_layout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_18 = QVBoxLayout()
        self.verticalLayout_18.setObjectName(u"verticalLayout_18")
        self.hline1_9 = QFrame(self.verticalLayoutWidget_9)
        self.hline1_9.setObjectName(u"hline1_9")
        self.hline1_9.setFrameShape(QFrame.HLine)
        self.hline1_9.setFrameShadow(QFrame.Raised)

        self.verticalLayout_18.addWidget(self.hline1_9)

        self.verticalLayout = QVBoxLayout()
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.widget_11 = QWidget(self.verticalLayoutWidget_9)
        self.widget_11.setObjectName(u"widget_11")

        self.verticalLayout.addWidget(self.widget_11)


        self.verticalLayout_18.addLayout(self.verticalLayout)

        self.horizontalLayout_15 = QHBoxLayout()
        self.horizontalLayout_15.setSpacing(9)
        self.horizontalLayout_15.setObjectName(u"horizontalLayout_15")
        self.motor_pos_ctrl_bar_1 = QSlider(self.verticalLayoutWidget_9)
        self.motor_pos_ctrl_bar_1.setObjectName(u"motor_pos_ctrl_bar_1")
        sizePolicy4 = QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        sizePolicy4.setHorizontalStretch(0)
        sizePolicy4.setVerticalStretch(1)
        sizePolicy4.setHeightForWidth(self.motor_pos_ctrl_bar_1.sizePolicy().hasHeightForWidth())
        self.motor_pos_ctrl_bar_1.setSizePolicy(sizePolicy4)
        self.motor_pos_ctrl_bar_1.setMaximum(44)
        self.motor_pos_ctrl_bar_1.setOrientation(Qt.Horizontal)

        self.horizontalLayout_15.addWidget(self.motor_pos_ctrl_bar_1)

        self.motor_speed_ctrl_label = QLabel(self.verticalLayoutWidget_9)
        self.motor_speed_ctrl_label.setObjectName(u"motor_speed_ctrl_label")

        self.horizontalLayout_15.addWidget(self.motor_speed_ctrl_label)

        self.motor_speed_ctrl_input = QSpinBox(self.verticalLayoutWidget_9)
        self.motor_speed_ctrl_input.setObjectName(u"motor_speed_ctrl_input")
        self.motor_speed_ctrl_input.setMaximum(20)
        self.motor_speed_ctrl_input.setValue(2)

        self.horizontalLayout_15.addWidget(self.motor_speed_ctrl_input)

        self.motor_pos_ctrl_bar_2 = QSlider(self.verticalLayoutWidget_9)
        self.motor_pos_ctrl_bar_2.setObjectName(u"motor_pos_ctrl_bar_2")
        sizePolicy4.setHeightForWidth(self.motor_pos_ctrl_bar_2.sizePolicy().hasHeightForWidth())
        self.motor_pos_ctrl_bar_2.setSizePolicy(sizePolicy4)
        self.motor_pos_ctrl_bar_2.setMaximum(44)
        self.motor_pos_ctrl_bar_2.setOrientation(Qt.Horizontal)
        self.motor_pos_ctrl_bar_2.setInvertedAppearance(True)

        self.horizontalLayout_15.addWidget(self.motor_pos_ctrl_bar_2)

        self.horizontalLayout_15.setStretch(0, 10)
        self.horizontalLayout_15.setStretch(1, 1)
        self.horizontalLayout_15.setStretch(2, 1)
        self.horizontalLayout_15.setStretch(3, 10)

        self.verticalLayout_18.addLayout(self.horizontalLayout_15)

        self.verticalLayout_8 = QVBoxLayout()
        self.verticalLayout_8.setObjectName(u"verticalLayout_8")
        self.widget_6 = QWidget(self.verticalLayoutWidget_9)
        self.widget_6.setObjectName(u"widget_6")

        self.verticalLayout_8.addWidget(self.widget_6)


        self.verticalLayout_18.addLayout(self.verticalLayout_8)

        self.hline1_8 = QFrame(self.verticalLayoutWidget_9)
        self.hline1_8.setObjectName(u"hline1_8")
        self.hline1_8.setFrameShape(QFrame.HLine)
        self.hline1_8.setFrameShadow(QFrame.Raised)

        self.verticalLayout_18.addWidget(self.hline1_8)

        self.horizontalLayout_7 = QHBoxLayout()
        self.horizontalLayout_7.setObjectName(u"horizontalLayout_7")
        self.motor_zero_button = QPushButton(self.verticalLayoutWidget_9)
        self.motor_zero_button.setObjectName(u"motor_zero_button")
        self.motor_zero_button.setStyleSheet(u"background-color: lightgray")

        self.horizontalLayout_7.addWidget(self.motor_zero_button)

        self.line_13 = QFrame(self.verticalLayoutWidget_9)
        self.line_13.setObjectName(u"line_13")
        self.line_13.setFrameShape(QFrame.VLine)
        self.line_13.setFrameShadow(QFrame.Sunken)

        self.horizontalLayout_7.addWidget(self.line_13)

        self.sensor_zero_button = QPushButton(self.verticalLayoutWidget_9)
        self.sensor_zero_button.setObjectName(u"sensor_zero_button")
        self.sensor_zero_button.setStyleSheet(u"background-color: lightgray")

        self.horizontalLayout_7.addWidget(self.sensor_zero_button)


        self.verticalLayout_18.addLayout(self.horizontalLayout_7)

        self.verticalLayout_18.setStretch(0, 1)
        self.verticalLayout_18.setStretch(1, 1)
        self.verticalLayout_18.setStretch(2, 4)
        self.verticalLayout_18.setStretch(3, 1)
        self.verticalLayout_18.setStretch(4, 1)
        self.verticalLayout_18.setStretch(5, 1)

        self.testing_control_panel_layout.addLayout(self.verticalLayout_18)

        self.hline1_15 = QFrame(self.verticalLayoutWidget_9)
        self.hline1_15.setObjectName(u"hline1_15")
        self.hline1_15.setFrameShape(QFrame.HLine)
        self.hline1_15.setFrameShadow(QFrame.Raised)

        self.testing_control_panel_layout.addWidget(self.hline1_15)

        self.verticalLayout_13 = QVBoxLayout()
        self.verticalLayout_13.setObjectName(u"verticalLayout_13")
        self.verticalLayout_13.setSizeConstraint(QLayout.SetNoConstraint)
        self.widget_5 = QWidget(self.verticalLayoutWidget_9)
        self.widget_5.setObjectName(u"widget_5")

        self.verticalLayout_13.addWidget(self.widget_5)

        self.horizontalLayout_4 = QHBoxLayout()
        self.horizontalLayout_4.setObjectName(u"horizontalLayout_4")
        self.horizontalLayout_4.setSizeConstraint(QLayout.SetMinimumSize)
        self.widget_13 = QWidget(self.verticalLayoutWidget_9)
        self.widget_13.setObjectName(u"widget_13")

        self.horizontalLayout_4.addWidget(self.widget_13)

        self.parasitic_decoupling_enable = QCheckBox(self.verticalLayoutWidget_9)
        self.parasitic_decoupling_enable.setObjectName(u"parasitic_decoupling_enable")
        self.parasitic_decoupling_enable.setFont(font1)

        self.horizontalLayout_4.addWidget(self.parasitic_decoupling_enable)

        self.widget_4 = QWidget(self.verticalLayoutWidget_9)
        self.widget_4.setObjectName(u"widget_4")

        self.horizontalLayout_4.addWidget(self.widget_4)

        self.force_feedback_ctrl_enable = QCheckBox(self.verticalLayoutWidget_9)
        self.force_feedback_ctrl_enable.setObjectName(u"force_feedback_ctrl_enable")
        self.force_feedback_ctrl_enable.setFont(font1)

        self.horizontalLayout_4.addWidget(self.force_feedback_ctrl_enable)

        self.horizontalLayout_4.setStretch(2, 1)
        self.horizontalLayout_4.setStretch(3, 7)

        self.verticalLayout_13.addLayout(self.horizontalLayout_4)

        self.widget_12 = QWidget(self.verticalLayoutWidget_9)
        self.widget_12.setObjectName(u"widget_12")

        self.verticalLayout_13.addWidget(self.widget_12)

        self.hline1_12 = QFrame(self.verticalLayoutWidget_9)
        self.hline1_12.setObjectName(u"hline1_12")
        self.hline1_12.setFrameShape(QFrame.HLine)
        self.hline1_12.setFrameShadow(QFrame.Raised)

        self.verticalLayout_13.addWidget(self.hline1_12)

        self.horizontalLayout_8 = QHBoxLayout()
        self.horizontalLayout_8.setObjectName(u"horizontalLayout_8")
        self.widget_9 = QWidget(self.verticalLayoutWidget_9)
        self.widget_9.setObjectName(u"widget_9")

        self.horizontalLayout_8.addWidget(self.widget_9)

        self.pressure_scrollbar_label = QLabel(self.verticalLayoutWidget_9)
        self.pressure_scrollbar_label.setObjectName(u"pressure_scrollbar_label")
        self.pressure_scrollbar_label.setFont(font1)
        self.pressure_scrollbar_label.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_8.addWidget(self.pressure_scrollbar_label)

        self.pressure_scrollbar = QScrollBar(self.verticalLayoutWidget_9)
        self.pressure_scrollbar.setObjectName(u"pressure_scrollbar")
        sizePolicy5 = QSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Preferred)
        sizePolicy5.setHorizontalStretch(0)
        sizePolicy5.setVerticalStretch(1)
        sizePolicy5.setHeightForWidth(self.pressure_scrollbar.sizePolicy().hasHeightForWidth())
        self.pressure_scrollbar.setSizePolicy(sizePolicy5)
        self.pressure_scrollbar.setAutoFillBackground(False)
        self.pressure_scrollbar.setStyleSheet(u"background: #f8f8f8; /* Light gray background */\n"
"    height: 15px;\n"
"    border: 2px solid #cccccc; /* Darker gray border on all sides */\n"
"    border-radius: 5px; /* Rounded corners */")
        self.pressure_scrollbar.setMaximum(100)
        self.pressure_scrollbar.setSingleStep(1)
        self.pressure_scrollbar.setOrientation(Qt.Horizontal)

        self.horizontalLayout_8.addWidget(self.pressure_scrollbar)

        self.horizontalLayout_8.setStretch(2, 3)

        self.verticalLayout_13.addLayout(self.horizontalLayout_8)

        self.hline1_7 = QFrame(self.verticalLayoutWidget_9)
        self.hline1_7.setObjectName(u"hline1_7")
        self.hline1_7.setFrameShape(QFrame.HLine)
        self.hline1_7.setFrameShadow(QFrame.Raised)

        self.verticalLayout_13.addWidget(self.hline1_7)

        self.horizontalLayout_10 = QHBoxLayout()
        self.horizontalLayout_10.setObjectName(u"horizontalLayout_10")
        self.pressure_type_input_label = QLabel(self.verticalLayoutWidget_9)
        self.pressure_type_input_label.setObjectName(u"pressure_type_input_label")
        self.pressure_type_input_label.setFont(font1)
        self.pressure_type_input_label.setAlignment(Qt.AlignCenter)
        self.pressure_type_input_label.setWordWrap(True)

        self.horizontalLayout_10.addWidget(self.pressure_type_input_label)

        self.pressure_type_input = QDoubleSpinBox(self.verticalLayoutWidget_9)
        self.pressure_type_input.setObjectName(u"pressure_type_input")
        self.pressure_type_input.setDecimals(1)
        self.pressure_type_input.setMinimum(0.000000000000000)
        self.pressure_type_input.setSingleStep(0.100000000000000)
        self.pressure_type_input.setValue(20.000000000000000)

        self.horizontalLayout_10.addWidget(self.pressure_type_input)

        self.pressure_unit = QLabel(self.verticalLayoutWidget_9)
        self.pressure_unit.setObjectName(u"pressure_unit")
        self.pressure_unit.setFont(font1)
        self.pressure_unit.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_10.addWidget(self.pressure_unit)

        self.line = QFrame(self.verticalLayoutWidget_9)
        self.line.setObjectName(u"line")
        self.line.setFrameShape(QFrame.VLine)
        self.line.setFrameShadow(QFrame.Sunken)

        self.horizontalLayout_10.addWidget(self.line)

        self.coefficient_label = QLabel(self.verticalLayoutWidget_9)
        self.coefficient_label.setObjectName(u"coefficient_label")
        self.coefficient_label.setFont(font1)
        self.coefficient_label.setAlignment(Qt.AlignCenter)
        self.coefficient_label.setWordWrap(True)

        self.horizontalLayout_10.addWidget(self.coefficient_label)

        self.ctrl_coefficient = QDoubleSpinBox(self.verticalLayoutWidget_9)
        self.ctrl_coefficient.setObjectName(u"ctrl_coefficient")
        self.ctrl_coefficient.setMaximum(2.000000000000000)
        self.ctrl_coefficient.setSingleStep(0.010000000000000)
        self.ctrl_coefficient.setValue(0.800000000000000)

        self.horizontalLayout_10.addWidget(self.ctrl_coefficient)

        self.horizontalLayout_10.setStretch(0, 2)
        self.horizontalLayout_10.setStretch(1, 1)
        self.horizontalLayout_10.setStretch(2, 1)
        self.horizontalLayout_10.setStretch(3, 1)
        self.horizontalLayout_10.setStretch(4, 2)
        self.horizontalLayout_10.setStretch(5, 2)

        self.verticalLayout_13.addLayout(self.horizontalLayout_10)

        self.verticalLayout_13.setStretch(1, 5)
        self.verticalLayout_13.setStretch(3, 1)
        self.verticalLayout_13.setStretch(4, 1)
        self.verticalLayout_13.setStretch(5, 1)
        self.verticalLayout_13.setStretch(6, 1)

        self.testing_control_panel_layout.addLayout(self.verticalLayout_13)

        self.hline1_3 = QFrame(self.verticalLayoutWidget_9)
        self.hline1_3.setObjectName(u"hline1_3")
        self.hline1_3.setFrameShape(QFrame.HLine)
        self.hline1_3.setFrameShadow(QFrame.Raised)

        self.testing_control_panel_layout.addWidget(self.hline1_3)

        self.horizontalLayout_5 = QHBoxLayout()
        self.horizontalLayout_5.setObjectName(u"horizontalLayout_5")
        self.widget = QWidget(self.verticalLayoutWidget_9)
        self.widget.setObjectName(u"widget")

        self.horizontalLayout_5.addWidget(self.widget)

        self.record_data_button = QPushButton(self.verticalLayoutWidget_9)
        self.record_data_button.setObjectName(u"record_data_button")
        self.record_data_button.setStyleSheet(u"background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:1, stop:0 rgba(179, 229, 252, 0.7), stop:1 rgba(255, 192, 192, 0.7));\n"
"    border: none;\n"
"    border-radius: 5px;\n"
"    padding: 5px;")

        self.horizontalLayout_5.addWidget(self.record_data_button)

        self.line_5 = QFrame(self.verticalLayoutWidget_9)
        self.line_5.setObjectName(u"line_5")
        self.line_5.setFrameShape(QFrame.VLine)
        self.line_5.setFrameShadow(QFrame.Sunken)

        self.horizontalLayout_5.addWidget(self.line_5)

        self.probing_button = QPushButton(self.verticalLayoutWidget_9)
        self.probing_button.setObjectName(u"probing_button")
        self.probing_button.setStyleSheet(u"background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:1, stop:0 rgba(255, 192, 192, 0.7), stop:1 rgba(179, 229, 252, 0.7));\n"
"    border: none;\n"
"    border-radius: 5px;\n"
"    padding: 5px;")

        self.horizontalLayout_5.addWidget(self.probing_button)

        self.line_25 = QFrame(self.verticalLayoutWidget_9)
        self.line_25.setObjectName(u"line_25")
        self.line_25.setFrameShape(QFrame.VLine)
        self.line_25.setFrameShadow(QFrame.Sunken)

        self.horizontalLayout_5.addWidget(self.line_25)

        self.firmness_value_label = QLabel(self.verticalLayoutWidget_9)
        self.firmness_value_label.setObjectName(u"firmness_value_label")
        self.firmness_value_label.setFont(font1)
        self.firmness_value_label.setAlignment(Qt.AlignCenter)
        self.firmness_value_label.setWordWrap(True)

        self.horizontalLayout_5.addWidget(self.firmness_value_label)

        self.firmness_value = QLCDNumber(self.verticalLayoutWidget_9)
        self.firmness_value.setObjectName(u"firmness_value")

        self.horizontalLayout_5.addWidget(self.firmness_value)

        self.widget_3 = QWidget(self.verticalLayoutWidget_9)
        self.widget_3.setObjectName(u"widget_3")

        self.horizontalLayout_5.addWidget(self.widget_3)

        self.horizontalLayout_5.setStretch(1, 2)
        self.horizontalLayout_5.setStretch(3, 2)
        self.horizontalLayout_5.setStretch(5, 1)

        self.testing_control_panel_layout.addLayout(self.horizontalLayout_5)

        self.testing_control_panel_layout.setStretch(0, 5)
        self.testing_control_panel_layout.setStretch(1, 1)
        self.testing_control_panel_layout.setStretch(2, 6)
        self.testing_control_panel_layout.setStretch(3, 1)
        self.testing_control_panel_layout.setStretch(4, 1)
        self.global_setting_panel = QGroupBox(self.centralwidget)
        self.global_setting_panel.setObjectName(u"global_setting_panel")
        self.global_setting_panel.setGeometry(QRect(10, 442, 391, 71))
        self.horizontalLayoutWidget_3 = QWidget(self.global_setting_panel)
        self.horizontalLayoutWidget_3.setObjectName(u"horizontalLayoutWidget_3")
        self.horizontalLayoutWidget_3.setGeometry(QRect(10, 30, 371, 31))
        self.horizontalLayout_12 = QHBoxLayout(self.horizontalLayoutWidget_3)
        self.horizontalLayout_12.setObjectName(u"horizontalLayout_12")
        self.horizontalLayout_12.setContentsMargins(0, 0, 0, 0)
        self.model_selection_comboBox = QComboBox(self.horizontalLayoutWidget_3)
        self.model_selection_comboBox.setObjectName(u"model_selection_comboBox")

        self.horizontalLayout_12.addWidget(self.model_selection_comboBox)

        self.line_15 = QFrame(self.horizontalLayoutWidget_3)
        self.line_15.setObjectName(u"line_15")
        self.line_15.setFrameShape(QFrame.VLine)
        self.line_15.setFrameShadow(QFrame.Sunken)

        self.horizontalLayout_12.addWidget(self.line_15)

        self.max_pressure_label_2 = QLabel(self.horizontalLayoutWidget_3)
        self.max_pressure_label_2.setObjectName(u"max_pressure_label_2")
        self.max_pressure_label_2.setFont(font1)
        self.max_pressure_label_2.setAlignment(Qt.AlignCenter)
        self.max_pressure_label_2.setWordWrap(True)

        self.horizontalLayout_12.addWidget(self.max_pressure_label_2)

        self.max_pressure_input = QDoubleSpinBox(self.horizontalLayoutWidget_3)
        self.max_pressure_input.setObjectName(u"max_pressure_input")
        self.max_pressure_input.setDecimals(1)
        self.max_pressure_input.setMinimum(0.000000000000000)
        self.max_pressure_input.setSingleStep(0.100000000000000)
        self.max_pressure_input.setValue(30.000000000000000)

        self.horizontalLayout_12.addWidget(self.max_pressure_input)

        self.pressure_unit_3 = QLabel(self.horizontalLayoutWidget_3)
        self.pressure_unit_3.setObjectName(u"pressure_unit_3")
        self.pressure_unit_3.setFont(font1)
        self.pressure_unit_3.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_12.addWidget(self.pressure_unit_3)

        self.horizontalLayout_12.setStretch(0, 4)
        self.horizontalLayout_12.setStretch(1, 1)
        self.horizontalLayout_12.setStretch(2, 1)
        self.horizontalLayout_12.setStretch(3, 1)
        self.horizontalLayout_12.setStretch(4, 1)
        self.color_frame_force_monitoring = QFrame(self.centralwidget)
        self.color_frame_force_monitoring.setObjectName(u"color_frame_force_monitoring")
        self.color_frame_force_monitoring.setGeometry(QRect(830, 11, 411, 411))
        self.color_frame_force_monitoring.setStyleSheet(u"background: qlineargradient(spread:pad, x1:0.5, y1:0, x2:0.5, y2:1, stop:0 rgba(179, 229, 252, 0.35), stop:0.1 rgba(227, 242, 253, 0),stop:0.35 rgba(179, 229, 252, 0.35), stop:0.45 rgba(179, 229, 252, 0.35), stop:0.6 rgba(227, 242, 253, 0), stop:0.8 rgba(255, 192, 192, 0.35), stop:1 rgba(227, 242, 253, 0));")
        self.color_frame_force_monitoring.setFrameShape(QFrame.NoFrame)
        self.color_frame_force_monitoring.setFrameShadow(QFrame.Raised)
        self.color_frame_tactile_monitoring_panel = QFrame(self.centralwidget)
        self.color_frame_tactile_monitoring_panel.setObjectName(u"color_frame_tactile_monitoring_panel")
        self.color_frame_tactile_monitoring_panel.setGeometry(QRect(-1, 11, 281, 411))
        self.color_frame_tactile_monitoring_panel.setStyleSheet(u"background: qlineargradient(spread:pad, x1:0.5, y1:0, x2:0.5, y2:1, stop:0 rgba(179, 229, 252, 0.35), stop:0.1 rgba(227, 242, 253, 0), stop:0.35 rgba(179, 229, 252, 0.35), stop:0.45 rgba(179, 229, 252, 0.35), stop:0.6 rgba(227, 242, 253, 0), stop:0.8 rgba(255, 192, 192, 0.35), stop:1 rgba(227, 242, 253, 0));")
        self.color_frame_tactile_monitoring_panel.setFrameShape(QFrame.NoFrame)
        self.color_frame_tactile_monitoring_panel.setFrameShadow(QFrame.Raised)
        self.color_frame_automatic_control_panels = QFrame(self.centralwidget)
        self.color_frame_automatic_control_panels.setObjectName(u"color_frame_automatic_control_panels")
        self.color_frame_automatic_control_panels.setGeometry(QRect(410, 430, 351, 431))
        self.color_frame_automatic_control_panels.setStyleSheet(u"background: qradialgradient(spread:pad, cx:0.5, cy:0.5, radius:1, stop:0 rgba(179, 229, 252, 0.35), stop:1 rgba(179, 229, 252, 0));")
        self.color_frame_automatic_control_panels.setFrameShape(QFrame.NoFrame)
        self.color_frame_automatic_control_panels.setFrameShadow(QFrame.Plain)
        self.color_frame_automatic_control_panels.setLineWidth(0)
        self.color_frame_sensing_signal_monitoring_panel = QFrame(self.centralwidget)
        self.color_frame_sensing_signal_monitoring_panel.setObjectName(u"color_frame_sensing_signal_monitoring_panel")
        self.color_frame_sensing_signal_monitoring_panel.setGeometry(QRect(760, 430, 921, 431))
        self.color_frame_sensing_signal_monitoring_panel.setStyleSheet(u"background: qlineargradient(spread:pad, x1:0.5, y1:0, x2:0.5, y2:1, stop:0 rgba(255, 192, 192, 0.35), stop:0.35 rgba(179, 229, 252, 0.35), stop:0.6 rgba(227, 242, 253, 0), stop:0.8 rgba(255, 192, 192, 0.35), stop:1 rgba(227, 242, 253, 0));")
        self.color_frame_sensing_signal_monitoring_panel.setFrameShape(QFrame.NoFrame)
        self.color_frame_sensing_signal_monitoring_panel.setFrameShadow(QFrame.Raised)
        self.frame_5 = QFrame(self.centralwidget)
        self.frame_5.setObjectName(u"frame_5")
        self.frame_5.setGeometry(QRect(-1, 425, 411, 8))
        self.frame_5.setStyleSheet(u"background: qlineargradient(spread:pad, x1:0.5, y1:0, x2:0.5, y2:1, stop:0 rgba(255, 192, 192, 0.1),  stop:1 rgba(255, 192, 192, 0.35));")
        self.frame_5.setFrameShape(QFrame.NoFrame)
        self.frame_5.setFrameShadow(QFrame.Raised)
        self.frame_10 = QFrame(self.centralwidget)
        self.frame_10.setObjectName(u"frame_10")
        self.frame_10.setGeometry(QRect(0, 6, 281, 8))
        self.frame_10.setStyleSheet(u"background: qlineargradient(spread:pad, x1:0.5, y1:0, x2:0.5, y2:1, stop:0 rgba(179, 229, 252, 0.1),  stop:1 rgba(179, 229, 252, 0.4));")
        self.frame_10.setFrameShape(QFrame.NoFrame)
        self.frame_10.setFrameShadow(QFrame.Raised)
        self.color_frame_testing_control_panel = QFrame(self.centralwidget)
        self.color_frame_testing_control_panel.setObjectName(u"color_frame_testing_control_panel")
        self.color_frame_testing_control_panel.setGeometry(QRect(0, 430, 411, 431))
        self.color_frame_testing_control_panel.setStyleSheet(u"background: qradialgradient(spread:pad, cx:0.5, cy:0.5, radius:1, stop:0 rgba(255, 192, 192, 0.35), stop:1 rgba(179, 229, 252, 0));")
        self.color_frame_testing_control_panel.setFrameShape(QFrame.NoFrame)
        self.color_frame_testing_control_panel.setFrameShadow(QFrame.Plain)
        self.color_frame_testing_control_panel.setLineWidth(0)
        self.frame_6 = QFrame(self.centralwidget)
        self.frame_6.setObjectName(u"frame_6")
        self.frame_6.setGeometry(QRect(410, 426, 351, 8))
        self.frame_6.setStyleSheet(u"background: qlineargradient(spread:pad, x1:0.5, y1:0, x2:0.5, y2:1, stop:0 rgba(179, 229, 252, 0.1),  stop:1 rgba(179, 229, 252, 0.4));")
        self.frame_6.setFrameShape(QFrame.NoFrame)
        self.frame_6.setFrameShadow(QFrame.Raised)
        self.frame_8 = QFrame(self.centralwidget)
        self.frame_8.setObjectName(u"frame_8")
        self.frame_8.setGeometry(QRect(760, 426, 921, 8))
        self.frame_8.setStyleSheet(u"background: qlineargradient(spread:pad, x1:0.5, y1:0, x2:0.5, y2:1, stop:0 rgba(255, 192, 192, 0.1),  stop:1 rgba(255, 192, 192, 0.35));")
        self.frame_8.setFrameShape(QFrame.NoFrame)
        self.frame_8.setFrameShadow(QFrame.Raised)
        self.frame_11 = QFrame(self.centralwidget)
        self.frame_11.setObjectName(u"frame_11")
        self.frame_11.setGeometry(QRect(830, 6, 411, 8))
        self.frame_11.setStyleSheet(u"background: qlineargradient(spread:pad, x1:0.5, y1:0, x2:0.5, y2:1, stop:0 rgba(179, 229, 252, 0.1),  stop:1 rgba(179, 229, 252, 0.4));")
        self.frame_11.setFrameShape(QFrame.NoFrame)
        self.frame_11.setFrameShadow(QFrame.Raised)
        self.sbrp_logo_label_1_1_39 = QLabel(self.centralwidget)
        self.sbrp_logo_label_1_1_39.setObjectName(u"sbrp_logo_label_1_1_39")
        self.sbrp_logo_label_1_1_39.setGeometry(QRect(1612, -1, 60, 43))
        sizePolicy2.setHeightForWidth(self.sbrp_logo_label_1_1_39.sizePolicy().hasHeightForWidth())
        self.sbrp_logo_label_1_1_39.setSizePolicy(sizePolicy2)
        self.sbrp_logo_label_1_1_39.setStyleSheet(u"")
        self.sbrp_logo_label_1_1_39.setPixmap(QPixmap(u"SBRPLogo1.png"))
        self.sbrp_logo_label_1_1_39.setScaledContents(True)
        self.sbrp_logo_label_1_1_39.setMargin(0)
        self.camera_panel = QGroupBox(self.centralwidget)
        self.camera_panel.setObjectName(u"camera_panel")
        self.camera_panel.setGeometry(QRect(290, 20, 531, 401))
        self.horizontalLayoutWidget_6 = QWidget(self.camera_panel)
        self.horizontalLayoutWidget_6.setObjectName(u"horizontalLayoutWidget_6")
        self.horizontalLayoutWidget_6.setGeometry(QRect(10, 30, 511, 361))
        self.horizontalLayout_20 = QHBoxLayout(self.horizontalLayoutWidget_6)
        self.horizontalLayout_20.setObjectName(u"horizontalLayout_20")
        self.horizontalLayout_20.setContentsMargins(0, 0, 0, 0)
        self.camera_openGLWidget = QOpenGLWidget(self.horizontalLayoutWidget_6)
        self.camera_openGLWidget.setObjectName(u"camera_openGLWidget")

        self.horizontalLayout_20.addWidget(self.camera_openGLWidget)

        self.color_frame_camera = QFrame(self.centralwidget)
        self.color_frame_camera.setObjectName(u"color_frame_camera")
        self.color_frame_camera.setGeometry(QRect(281, 10, 551, 411))
        self.color_frame_camera.setStyleSheet(u"background: qlineargradient(spread:pad, x1:0.5, y1:0, x2:0.5, y2:1, stop:0 rgba(255, 192, 192, 0.35), stop:0.1 rgba(227, 242, 253, 0),stop:0.35 rgba(179, 229, 252, 0.35), stop:0.45 rgba(179, 229, 252, 0.35), stop:0.6 rgba(227, 242, 253, 0), stop:0.8 rgba(255, 192, 192, 0.35), stop:1 rgba(227, 242, 253, 0));")
        self.color_frame_camera.setFrameShape(QFrame.NoFrame)
        self.color_frame_camera.setFrameShadow(QFrame.Raised)
        self.frame_12 = QFrame(self.centralwidget)
        self.frame_12.setObjectName(u"frame_12")
        self.frame_12.setGeometry(QRect(281, 6, 551, 8))
        self.frame_12.setStyleSheet(u"background: qlineargradient(spread:pad, x1:0.5, y1:0, x2:0.5, y2:1, stop:0 rgba(227, 242, 253, 0), stop:0.1 rgba(255, 192, 192, 0.35));")
        self.frame_12.setFrameShape(QFrame.NoFrame)
        self.frame_12.setFrameShadow(QFrame.Raised)
        self.simulation_panel = QGroupBox(self.centralwidget)
        self.simulation_panel.setObjectName(u"simulation_panel")
        self.simulation_panel.setGeometry(QRect(1249, 20, 421, 401))
        self.sofa_simulation_button = QPushButton(self.simulation_panel)
        self.sofa_simulation_button.setObjectName(u"sofa_simulation_button")
        self.sofa_simulation_button.setGeometry(QRect(10, 10, 102, 31))
        self.sofa_simulation_button.setStyleSheet(u"background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:1, stop:0 rgba(255, 192, 192, 0.7), stop:1 rgba(179, 229, 252, 0.7));\n"
"    border: none;\n"
"    border-radius: 5px;\n"
"    padding: 5px;")
        self.sofa_openGLWidget = QOpenGLWidget(self.simulation_panel)
        self.sofa_openGLWidget.setObjectName(u"sofa_openGLWidget")
        self.sofa_openGLWidget.setGeometry(QRect(10, 46, 399, 349))
        self.sofa_openGLWidget.setStyleSheet(u"background-color: rgba(0, 0, 0, 0); /* Fully transparent background */\n"
"    border: none; /* No border */")
        self.color_frame_simulation_panel = QFrame(self.centralwidget)
        self.color_frame_simulation_panel.setObjectName(u"color_frame_simulation_panel")
        self.color_frame_simulation_panel.setGeometry(QRect(1240, 10, 441, 421))
        self.color_frame_simulation_panel.setStyleSheet(u"background: qlineargradient(spread:pad, x1:0.5, y1:0, x2:0.5, y2:1, stop:0 rgba(255, 192, 192, 0.35), stop:0.1 rgba(227, 242, 253, 0),stop:0.35 rgba(179, 229, 252, 0.35), stop:0.45 rgba(179, 229, 252, 0.35), stop:0.6 rgba(227, 242, 253, 0), stop:0.8 rgba(255, 192, 192, 0.35), stop:1 rgba(227, 242, 253, 0));")
        self.color_frame_simulation_panel.setFrameShape(QFrame.NoFrame)
        self.color_frame_simulation_panel.setFrameShadow(QFrame.Raised)
        self.line_20 = QFrame(self.color_frame_simulation_panel)
        self.line_20.setObjectName(u"line_20")
        self.line_20.setGeometry(QRect(0, 0, 2, 421))
        self.line_20.setStyleSheet(u"    border: none;\n"
"    background: qlineargradient(spread:pad, y1:0, y2:1,stop:0 rgba(0, 0, 0, 0), stop:0.25 rgba(179, 229, 252, 1), stop:0.5 rgba(0, 0, 0, 0), stop:0.75 rgba(255, 192, 192, 1), stop:1 rgba(0, 0, 0, 0));")
        self.line_20.setFrameShape(QFrame.VLine)
        self.line_20.setFrameShadow(QFrame.Sunken)
        self.frame_13 = QFrame(self.centralwidget)
        self.frame_13.setObjectName(u"frame_13")
        self.frame_13.setGeometry(QRect(1240, 6, 361, 8))
        self.frame_13.setStyleSheet(u"background: qlineargradient(spread:pad, x1:0.5, y1:0, x2:0.5, y2:1, stop:0 rgba(227, 242, 253, 0), stop:0.1 rgba(255, 192, 192, 0.35));")
        self.frame_13.setFrameShape(QFrame.NoFrame)
        self.frame_13.setFrameShadow(QFrame.Raised)
        self.line_21 = QFrame(self.centralwidget)
        self.line_21.setObjectName(u"line_21")
        self.line_21.setGeometry(QRect(830, 10, 2, 421))
        self.line_21.setStyleSheet(u"    border: none;\n"
"    background: qlineargradient(spread:pad, y1:0, y2:1,stop:0 rgba(0, 0, 0, 0), stop:0.25 rgba(179, 229, 252, 1), stop:0.5 rgba(0, 0, 0, 0), stop:0.75 rgba(255, 192, 192, 1), stop:1 rgba(0, 0, 0, 0));")
        self.line_21.setFrameShape(QFrame.VLine)
        self.line_21.setFrameShadow(QFrame.Sunken)
        self.line_22 = QFrame(self.centralwidget)
        self.line_22.setObjectName(u"line_22")
        self.line_22.setGeometry(QRect(280, 10, 2, 411))
        self.line_22.setStyleSheet(u"    border: none;\n"
"    background: qlineargradient(spread:pad, y1:0, y2:1,stop:0 rgba(0, 0, 0, 0), stop:0.25 rgba(179, 229, 252, 1), stop:0.5 rgba(0, 0, 0, 0), stop:0.75 rgba(255, 192, 192, 1), stop:1 rgba(0, 0, 0, 0));")
        self.line_22.setFrameShape(QFrame.VLine)
        self.line_22.setFrameShadow(QFrame.Sunken)
        self.line_23 = QFrame(self.centralwidget)
        self.line_23.setObjectName(u"line_23")
        self.line_23.setGeometry(QRect(759, 460, 2, 391))
        self.line_23.setStyleSheet(u"    border: none;\n"
"    background: qlineargradient(spread:pad, y1:0, y2:1,stop:0 rgba(0, 0, 0, 0), stop:0.25 rgba(179, 229, 252, 1), stop:0.5 rgba(0, 0, 0, 0), stop:0.75 rgba(255, 192, 192, 1), stop:1 rgba(0, 0, 0, 0));")
        self.line_23.setFrameShape(QFrame.VLine)
        self.line_23.setFrameShadow(QFrame.Sunken)
        self.line_24 = QFrame(self.centralwidget)
        self.line_24.setObjectName(u"line_24")
        self.line_24.setGeometry(QRect(410, 460, 2, 391))
        self.line_24.setStyleSheet(u"    border: none;\n"
"    background: qlineargradient(spread:pad, y1:0, y2:1,stop:0 rgba(0, 0, 0, 0), stop:0.25 rgba(179, 229, 252, 1), stop:0.5 rgba(0, 0, 0, 0), stop:0.75 rgba(255, 192, 192, 1), stop:1 rgba(0, 0, 0, 0));")
        self.line_24.setFrameShape(QFrame.VLine)
        self.line_24.setFrameShadow(QFrame.Sunken)
        MainWindow.setCentralWidget(self.centralwidget)
        self.color_frame_tactile_monitoring_panel.raise_()
        self.color_frame_simulation_panel.raise_()
        self.frame_13.raise_()
        self.color_frame_camera.raise_()
        self.frame_10.raise_()
        self.color_frame_force_monitoring.raise_()
        self.color_frame_automatic_control_panels.raise_()
        self.color_frame_testing_control_panel.raise_()
        self.color_frame_sensing_signal_monitoring_panel.raise_()
        self.sensing_signal_monitoring_panel.raise_()
        self.tactile_monitoring_panel.raise_()
        self.force_monitoring_panel.raise_()
        self.automatic_grasping_control_panel.raise_()
        self.status_monitoring_panel_2.raise_()
        self.testing_control_panel.raise_()
        self.global_setting_panel.raise_()
        self.frame_5.raise_()
        self.frame_6.raise_()
        self.frame_8.raise_()
        self.frame_11.raise_()
        self.camera_panel.raise_()
        self.frame_12.raise_()
        self.simulation_panel.raise_()
        self.sbrp_logo_label_1_1_39.raise_()
        self.line_21.raise_()
        self.line_23.raise_()
        self.line_24.raise_()
        self.line_22.raise_()
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 1692, 23))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.toolBar = QToolBar(MainWindow)
        self.toolBar.setObjectName(u"toolBar")
        MainWindow.addToolBar(Qt.TopToolBarArea, self.toolBar)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.automatic_grasping_control_panel.setTitle(QCoreApplication.translate("MainWindow", u"  Grasping Control Panel", None))
        self.restart_button.setText(QCoreApplication.translate("MainWindow", u"Start", None))
        self.release_button.setText(QCoreApplication.translate("MainWindow", u"Release", None))
        self.stop_all_button.setText(QCoreApplication.translate("MainWindow", u"! STOP ALL !", None))
        self.force_monitoring_panel.setTitle(QCoreApplication.translate("MainWindow", u"  Force Monitoring Panel", None))
        self.tactile_monitoring_panel.setTitle(QCoreApplication.translate("MainWindow", u"  Tactile Monitoring Panel", None))
        self.S1_tactile_monitoring_label.setText(QCoreApplication.translate("MainWindow", u"S1 Tactile Monitoring", None))
        self.force_value_1_x.setText(QCoreApplication.translate("MainWindow", u"F<sub>X</sub> :", None))
        self.force_value_1_y.setText(QCoreApplication.translate("MainWindow", u"F<sub>Y</sub> :", None))
        self.force_value_1_z.setText(QCoreApplication.translate("MainWindow", u"F<sub>Z</sub> :", None))
        self.S2_tactile_monitoring_label.setText(QCoreApplication.translate("MainWindow", u"S2 Tactile Monitoring", None))
        self.force_value_2_x.setText(QCoreApplication.translate("MainWindow", u"F<sub>X</sub> :", None))
        self.force_value_2_y.setText(QCoreApplication.translate("MainWindow", u"F<sub>Y</sub> :", None))
        self.force_value_2_z.setText(QCoreApplication.translate("MainWindow", u"F<sub>Z</sub> :", None))
        self.sensing_signal_monitoring_panel.setTitle(QCoreApplication.translate("MainWindow", u"  Sensing Signal Monitoring Panel", None))
        self.status_monitoring_panel_2.setTitle(QCoreApplication.translate("MainWindow", u"Status Monitoring Panel 2", None))
        self.motor_pos_reading_label.setText(QCoreApplication.translate("MainWindow", u"<html><head/><body><p>Pos<span style=\" vertical-align:sub;\">motor</span></p></body></html>", None))
        self.data_stable_flag_indicator.setText(QCoreApplication.translate("MainWindow", u"Data Processing", None))
        self.contact_detect_indicator.setText(QCoreApplication.translate("MainWindow", u"Contact Detect", None))
        self.grasp_stable_flag_indicator.setText(QCoreApplication.translate("MainWindow", u"Grasp Stable", None))
        self.motor_stop_indicator.setText(QCoreApplication.translate("MainWindow", u"Motor Stop", None))
        self.elapsed_time_label.setText(QCoreApplication.translate("MainWindow", u"Elapse Time:", None))
        self.time_reset_button.setText(QCoreApplication.translate("MainWindow", u"Reset", None))
        self.testing_control_panel.setTitle(QCoreApplication.translate("MainWindow", u"Testing Control Panel", None))
        self.motor_speed_ctrl_label.setText(QCoreApplication.translate("MainWindow", u"RPM<sub>motor", None))
        self.motor_zero_button.setText(QCoreApplication.translate("MainWindow", u"Motor At Zero", None))
        self.sensor_zero_button.setText(QCoreApplication.translate("MainWindow", u"Zero Sensor", None))
        self.parasitic_decoupling_enable.setText(QCoreApplication.translate("MainWindow", u"Parasitic Decoupling", None))
        self.force_feedback_ctrl_enable.setText(QCoreApplication.translate("MainWindow", u"Feedback Control", None))
        self.pressure_scrollbar_label.setText(QCoreApplication.translate("MainWindow", u"P<sub>input", None))
        self.pressure_type_input_label.setText(QCoreApplication.translate("MainWindow", u"P<sub>input", None))
        self.pressure_type_input.setSuffix("")
        self.pressure_unit.setText(QCoreApplication.translate("MainWindow", u"kPa", None))
        self.coefficient_label.setText(QCoreApplication.translate("MainWindow", u"P<sub>1</sub>/P<sub>2", None))
        self.record_data_button.setText(QCoreApplication.translate("MainWindow", u"Recording", None))
        self.probing_button.setText(QCoreApplication.translate("MainWindow", u"Probing", None))
        self.firmness_value_label.setText(QCoreApplication.translate("MainWindow", u"F<sub>obj", None))
        self.global_setting_panel.setTitle(QCoreApplication.translate("MainWindow", u"  Global Setting Panel", None))
        self.max_pressure_label_2.setText(QCoreApplication.translate("MainWindow", u"P<sub>max", None))
        self.max_pressure_input.setSuffix("")
        self.pressure_unit_3.setText(QCoreApplication.translate("MainWindow", u"kPa", None))
        self.sbrp_logo_label_1_1_39.setText("")
        self.camera_panel.setTitle(QCoreApplication.translate("MainWindow", u"  Camera", None))
        self.simulation_panel.setTitle("")
        self.sofa_simulation_button.setText(QCoreApplication.translate("MainWindow", u"Simulation", None))
        self.toolBar.setWindowTitle(QCoreApplication.translate("MainWindow", u"toolBar", None))
    # retranslateUi

