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
from PySide6.QtWidgets import (QApplication, QFrame, QGroupBox, QHBoxLayout,
    QLCDNumber, QLabel, QMainWindow, QMenuBar,
    QPushButton, QSizePolicy, QSlider, QStatusBar,
    QToolBar, QVBoxLayout, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(1338, 754)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.control_panel = QGroupBox(self.centralwidget)
        self.control_panel.setObjectName(u"control_panel")
        self.control_panel.setGeometry(QRect(10, 390, 351, 301))
        self.verticalLayoutWidget_2 = QWidget(self.control_panel)
        self.verticalLayoutWidget_2.setObjectName(u"verticalLayoutWidget_2")
        self.verticalLayoutWidget_2.setGeometry(QRect(10, 30, 331, 181))
        self.verticalLayout_2 = QVBoxLayout(self.verticalLayoutWidget_2)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_2 = QHBoxLayout()
        self.horizontalLayout_2.setSpacing(9)
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.horizontalLayout_2.setContentsMargins(6, -1, 6, -1)
        self.release_button = QPushButton(self.verticalLayoutWidget_2)
        self.release_button.setObjectName(u"release_button")

        self.horizontalLayout_2.addWidget(self.release_button)

        self.restart_button = QPushButton(self.verticalLayoutWidget_2)
        self.restart_button.setObjectName(u"restart_button")

        self.horizontalLayout_2.addWidget(self.restart_button)


        self.verticalLayout_2.addLayout(self.horizontalLayout_2)

        self.horizontalLayout_5 = QHBoxLayout()
        self.horizontalLayout_5.setSpacing(9)
        self.horizontalLayout_5.setObjectName(u"horizontalLayout_5")
        self.motor_pos_bar_1 = QSlider(self.verticalLayoutWidget_2)
        self.motor_pos_bar_1.setObjectName(u"motor_pos_bar_1")
        self.motor_pos_bar_1.setOrientation(Qt.Horizontal)

        self.horizontalLayout_5.addWidget(self.motor_pos_bar_1)

        self.motor_position_label = QLabel(self.verticalLayoutWidget_2)
        self.motor_position_label.setObjectName(u"motor_position_label")

        self.horizontalLayout_5.addWidget(self.motor_position_label)

        self.motor_pos_bar_2 = QSlider(self.verticalLayoutWidget_2)
        self.motor_pos_bar_2.setObjectName(u"motor_pos_bar_2")
        self.motor_pos_bar_2.setOrientation(Qt.Horizontal)
        self.motor_pos_bar_2.setInvertedAppearance(True)

        self.horizontalLayout_5.addWidget(self.motor_pos_bar_2)


        self.verticalLayout_2.addLayout(self.horizontalLayout_5)

        self.horizontalLayout_3 = QHBoxLayout()
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.stop_all_button = QPushButton(self.verticalLayoutWidget_2)
        self.stop_all_button.setObjectName(u"stop_all_button")

        self.horizontalLayout_3.addWidget(self.stop_all_button)


        self.verticalLayout_2.addLayout(self.horizontalLayout_3)

        self.verticalLayout_2.setStretch(0, 4)
        self.verticalLayout_2.setStretch(1, 4)
        self.verticalLayout_2.setStretch(2, 1)
        self.verticalLayoutWidget_6 = QWidget(self.control_panel)
        self.verticalLayoutWidget_6.setObjectName(u"verticalLayoutWidget_6")
        self.verticalLayoutWidget_6.setGeometry(QRect(10, 210, 331, 95))
        self.verticalLayout_7 = QVBoxLayout(self.verticalLayoutWidget_6)
        self.verticalLayout_7.setObjectName(u"verticalLayout_7")
        self.verticalLayout_7.setContentsMargins(0, 0, 0, 0)
        self.hline3 = QFrame(self.verticalLayoutWidget_6)
        self.hline3.setObjectName(u"hline3")
        self.hline3.setFrameShape(QFrame.HLine)
        self.hline3.setFrameShadow(QFrame.Raised)

        self.verticalLayout_7.addWidget(self.hline3)

        self.horizontalLayout_6 = QHBoxLayout()
        self.horizontalLayout_6.setSpacing(0)
        self.horizontalLayout_6.setObjectName(u"horizontalLayout_6")
        self.horizontalLayout_6.setContentsMargins(0, 0, 0, 0)
        self.stable_flag_indicator = QLabel(self.verticalLayoutWidget_6)
        self.stable_flag_indicator.setObjectName(u"stable_flag_indicator")
        self.stable_flag_indicator.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_6.addWidget(self.stable_flag_indicator)

        self.contact_detect_indicator = QLabel(self.verticalLayoutWidget_6)
        self.contact_detect_indicator.setObjectName(u"contact_detect_indicator")
        self.contact_detect_indicator.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_6.addWidget(self.contact_detect_indicator)


        self.verticalLayout_7.addLayout(self.horizontalLayout_6)

        self.horizontalLayout_7 = QHBoxLayout()
        self.horizontalLayout_7.setSpacing(0)
        self.horizontalLayout_7.setObjectName(u"horizontalLayout_7")
        self.horizontalLayout_7.setContentsMargins(0, 0, 0, 0)
        self.grasp_stable_flag_indicator = QLabel(self.verticalLayoutWidget_6)
        self.grasp_stable_flag_indicator.setObjectName(u"grasp_stable_flag_indicator")
        self.grasp_stable_flag_indicator.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_7.addWidget(self.grasp_stable_flag_indicator)

        self.motor_stop_indicator = QLabel(self.verticalLayoutWidget_6)
        self.motor_stop_indicator.setObjectName(u"motor_stop_indicator")
        self.motor_stop_indicator.setAutoFillBackground(False)
        self.motor_stop_indicator.setScaledContents(False)
        self.motor_stop_indicator.setAlignment(Qt.AlignCenter)
        self.motor_stop_indicator.setMargin(0)

        self.horizontalLayout_7.addWidget(self.motor_stop_indicator)


        self.verticalLayout_7.addLayout(self.horizontalLayout_7)

        self.sensor_data_panel = QGroupBox(self.centralwidget)
        self.sensor_data_panel.setObjectName(u"sensor_data_panel")
        self.sensor_data_panel.setGeometry(QRect(360, 390, 971, 301))
        self.sensor_data_panel.setFlat(False)
        self.sensor_data_panel.setCheckable(False)
        self.verticalLayoutWidget = QWidget(self.sensor_data_panel)
        self.verticalLayoutWidget.setObjectName(u"verticalLayoutWidget")
        self.verticalLayoutWidget.setGeometry(QRect(9, 29, 471, 271))
        self.verticalLayout = QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.S1_graph = QFrame(self.verticalLayoutWidget)
        self.S1_graph.setObjectName(u"S1_graph")
        self.S1_graph.setFrameShape(QFrame.StyledPanel)
        self.S1_graph.setFrameShadow(QFrame.Raised)

        self.verticalLayout.addWidget(self.S1_graph)

        self.verticalLayout.setStretch(0, 10)
        self.verticalLayoutWidget_3 = QWidget(self.sensor_data_panel)
        self.verticalLayoutWidget_3.setObjectName(u"verticalLayoutWidget_3")
        self.verticalLayoutWidget_3.setGeometry(QRect(489, 30, 481, 271))
        self.verticalLayout_4 = QVBoxLayout(self.verticalLayoutWidget_3)
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.verticalLayout_4.setContentsMargins(0, 0, 0, 0)
        self.S2_graph = QFrame(self.verticalLayoutWidget_3)
        self.S2_graph.setObjectName(u"S2_graph")
        self.S2_graph.setFrameShape(QFrame.StyledPanel)
        self.S2_graph.setFrameShadow(QFrame.Raised)

        self.verticalLayout_4.addWidget(self.S2_graph)

        self.groupBox = QGroupBox(self.centralwidget)
        self.groupBox.setObjectName(u"groupBox")
        self.groupBox.setGeometry(QRect(10, 10, 601, 371))
        self.horizontalLayoutWidget = QWidget(self.groupBox)
        self.horizontalLayoutWidget.setObjectName(u"horizontalLayoutWidget")
        self.horizontalLayoutWidget.setGeometry(QRect(10, 30, 581, 331))
        self.horizontalLayout = QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_3 = QVBoxLayout()
        self.verticalLayout_3.setSpacing(6)
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.hline1 = QFrame(self.horizontalLayoutWidget)
        self.hline1.setObjectName(u"hline1")
        self.hline1.setFrameShape(QFrame.HLine)
        self.hline1.setFrameShadow(QFrame.Raised)

        self.verticalLayout_3.addWidget(self.hline1)

        self.S1_tactile_info_label = QLabel(self.horizontalLayoutWidget)
        self.S1_tactile_info_label.setObjectName(u"S1_tactile_info_label")
        self.S1_tactile_info_label.setAlignment(Qt.AlignBottom|Qt.AlignHCenter)
        self.S1_tactile_info_label.setMargin(0)

        self.verticalLayout_3.addWidget(self.S1_tactile_info_label)

        self.force_val_1 = QLCDNumber(self.horizontalLayoutWidget)
        self.force_val_1.setObjectName(u"force_val_1")
        font = QFont()
        font.setFamilies([u"Ubuntu"])
        font.setBold(False)
        font.setItalic(False)
        self.force_val_1.setFont(font)
        self.force_val_1.setSmallDecimalPoint(True)
        self.force_val_1.setProperty("value", 0.000000000000000)

        self.verticalLayout_3.addWidget(self.force_val_1)

        self.quadrant_3 = QWidget(self.horizontalLayoutWidget)
        self.quadrant_3.setObjectName(u"quadrant_3")
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.quadrant_3.sizePolicy().hasHeightForWidth())
        self.quadrant_3.setSizePolicy(sizePolicy)
        self.quadrant_1 = QWidget(self.quadrant_3)
        self.quadrant_1.setObjectName(u"quadrant_1")
        self.quadrant_1.setGeometry(QRect(-20, -25, 151, 151))

        self.verticalLayout_3.addWidget(self.quadrant_3)

        self.S2_tactile_info_label = QLabel(self.horizontalLayoutWidget)
        self.S2_tactile_info_label.setObjectName(u"S2_tactile_info_label")
        self.S2_tactile_info_label.setAlignment(Qt.AlignCenter)

        self.verticalLayout_3.addWidget(self.S2_tactile_info_label)

        self.force_val_2 = QLCDNumber(self.horizontalLayoutWidget)
        self.force_val_2.setObjectName(u"force_val_2")
        font1 = QFont()
        font1.setBold(True)
        font1.setStrikeOut(False)
        self.force_val_2.setFont(font1)

        self.verticalLayout_3.addWidget(self.force_val_2)

        self.quadrant_4 = QWidget(self.horizontalLayoutWidget)
        self.quadrant_4.setObjectName(u"quadrant_4")
        sizePolicy.setHeightForWidth(self.quadrant_4.sizePolicy().hasHeightForWidth())
        self.quadrant_4.setSizePolicy(sizePolicy)
        self.quadrant_2 = QWidget(self.quadrant_4)
        self.quadrant_2.setObjectName(u"quadrant_2")
        self.quadrant_2.setGeometry(QRect(-20, -30, 151, 151))

        self.verticalLayout_3.addWidget(self.quadrant_4)

        self.verticalLayout_3.setStretch(2, 1)
        self.verticalLayout_3.setStretch(3, 3)
        self.verticalLayout_3.setStretch(5, 1)
        self.verticalLayout_3.setStretch(6, 3)

        self.horizontalLayout.addLayout(self.verticalLayout_3)

        self.hline2 = QFrame(self.horizontalLayoutWidget)
        self.hline2.setObjectName(u"hline2")
        self.hline2.setFrameShape(QFrame.VLine)
        self.hline2.setFrameShadow(QFrame.Raised)

        self.horizontalLayout.addWidget(self.hline2)

        self.openGLWidget = QOpenGLWidget(self.horizontalLayoutWidget)
        self.openGLWidget.setObjectName(u"openGLWidget")

        self.horizontalLayout.addWidget(self.openGLWidget)

        self.horizontalLayout.setStretch(0, 1)
        self.horizontalLayout.setStretch(1, 1)
        self.horizontalLayout.setStretch(2, 4)
        self.groupBox_2 = QGroupBox(self.centralwidget)
        self.groupBox_2.setObjectName(u"groupBox_2")
        self.groupBox_2.setGeometry(QRect(619, 9, 711, 371))
        self.verticalLayoutWidget_5 = QWidget(self.groupBox_2)
        self.verticalLayoutWidget_5.setObjectName(u"verticalLayoutWidget_5")
        self.verticalLayoutWidget_5.setGeometry(QRect(10, 30, 339, 329))
        self.verticalLayout_6 = QVBoxLayout(self.verticalLayoutWidget_5)
        self.verticalLayout_6.setObjectName(u"verticalLayout_6")
        self.verticalLayout_6.setContentsMargins(0, 0, 0, 0)
        self.force_val_graph = QFrame(self.verticalLayoutWidget_5)
        self.force_val_graph.setObjectName(u"force_val_graph")
        self.force_val_graph.setFrameShape(QFrame.StyledPanel)
        self.force_val_graph.setFrameShadow(QFrame.Raised)

        self.verticalLayout_6.addWidget(self.force_val_graph)

        self.verticalLayout_6.setStretch(0, 10)
        self.verticalLayoutWidget_4 = QWidget(self.groupBox_2)
        self.verticalLayoutWidget_4.setObjectName(u"verticalLayoutWidget_4")
        self.verticalLayoutWidget_4.setGeometry(QRect(360, 30, 349, 329))
        self.verticalLayout_5 = QVBoxLayout(self.verticalLayoutWidget_4)
        self.verticalLayout_5.setObjectName(u"verticalLayout_5")
        self.verticalLayout_5.setContentsMargins(0, 0, 0, 0)
        self.pressure_graph = QFrame(self.verticalLayoutWidget_4)
        self.pressure_graph.setObjectName(u"pressure_graph")
        self.pressure_graph.setFrameShape(QFrame.StyledPanel)
        self.pressure_graph.setFrameShadow(QFrame.Raised)

        self.verticalLayout_5.addWidget(self.pressure_graph)

        self.verticalLayout_5.setStretch(0, 10)
        MainWindow.setCentralWidget(self.centralwidget)
        self.groupBox_2.raise_()
        self.groupBox.raise_()
        self.sensor_data_panel.raise_()
        self.control_panel.raise_()
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 1338, 22))
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
        self.control_panel.setTitle(QCoreApplication.translate("MainWindow", u"Control Panel", None))
        self.release_button.setText(QCoreApplication.translate("MainWindow", u"Release", None))
        self.restart_button.setText(QCoreApplication.translate("MainWindow", u"Restart", None))
        self.motor_position_label.setText(QCoreApplication.translate("MainWindow", u"Motor Position", None))
        self.stop_all_button.setText(QCoreApplication.translate("MainWindow", u"STOP ALL", None))
        self.stable_flag_indicator.setText(QCoreApplication.translate("MainWindow", u"Dataprocess stable", None))
        self.contact_detect_indicator.setText(QCoreApplication.translate("MainWindow", u"Contact detect", None))
        self.grasp_stable_flag_indicator.setText(QCoreApplication.translate("MainWindow", u"Grasp stable", None))
        self.motor_stop_indicator.setText(QCoreApplication.translate("MainWindow", u"Motor stop", None))
        self.sensor_data_panel.setTitle(QCoreApplication.translate("MainWindow", u"Sensor signals", None))
        self.groupBox.setTitle(QCoreApplication.translate("MainWindow", u"Gripper status", None))
        self.S1_tactile_info_label.setText(QCoreApplication.translate("MainWindow", u"S1 Tactile Info", None))
        self.S2_tactile_info_label.setText(QCoreApplication.translate("MainWindow", u"S2 Tactile Info", None))
        self.groupBox_2.setTitle(QCoreApplication.translate("MainWindow", u"Monitoring Panel", None))
        self.toolBar.setWindowTitle(QCoreApplication.translate("MainWindow", u"toolBar", None))
    # retranslateUi

