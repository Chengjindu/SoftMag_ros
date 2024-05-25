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
from PySide6.QtWidgets import (QApplication, QCheckBox, QComboBox, QFrame,
    QGroupBox, QHBoxLayout, QLCDNumber, QLabel,
    QMainWindow, QMenuBar, QPushButton, QScrollBar,
    QSizePolicy, QSlider, QStatusBar, QToolBar,
    QVBoxLayout, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(1558, 844)
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Preferred)
        sizePolicy.setHorizontalStretch(20)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(MainWindow.sizePolicy().hasHeightForWidth())
        MainWindow.setSizePolicy(sizePolicy)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.automatic_grasping_control_panel = QGroupBox(self.centralwidget)
        self.automatic_grasping_control_panel.setObjectName(u"automatic_grasping_control_panel")
        self.automatic_grasping_control_panel.setGeometry(QRect(370, 390, 351, 141))
        self.verticalLayoutWidget_2 = QWidget(self.automatic_grasping_control_panel)
        self.verticalLayoutWidget_2.setObjectName(u"verticalLayoutWidget_2")
        self.verticalLayoutWidget_2.setGeometry(QRect(10, 30, 331, 101))
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

        self.hline1_6 = QFrame(self.verticalLayoutWidget_2)
        self.hline1_6.setObjectName(u"hline1_6")
        self.hline1_6.setFrameShape(QFrame.HLine)
        self.hline1_6.setFrameShadow(QFrame.Raised)

        self.verticalLayout_2.addWidget(self.hline1_6)

        self.horizontalLayout_3 = QHBoxLayout()
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.stop_all_button_grasping = QPushButton(self.verticalLayoutWidget_2)
        self.stop_all_button_grasping.setObjectName(u"stop_all_button_grasping")

        self.horizontalLayout_3.addWidget(self.stop_all_button_grasping)


        self.verticalLayout_2.addLayout(self.horizontalLayout_3)

        self.verticalLayout_2.setStretch(0, 3)
        self.verticalLayout_2.setStretch(1, 1)
        self.verticalLayout_2.setStretch(2, 3)
        self.other_signal_monitoring_panel = QGroupBox(self.centralwidget)
        self.other_signal_monitoring_panel.setObjectName(u"other_signal_monitoring_panel")
        self.other_signal_monitoring_panel.setGeometry(QRect(730, 390, 821, 391))
        self.other_signal_monitoring_panel.setFlat(False)
        self.other_signal_monitoring_panel.setCheckable(False)
        self.verticalLayoutWidget_5 = QWidget(self.other_signal_monitoring_panel)
        self.verticalLayoutWidget_5.setObjectName(u"verticalLayoutWidget_5")
        self.verticalLayoutWidget_5.setGeometry(QRect(10, 30, 401, 351))
        self.verticalLayout_6 = QVBoxLayout(self.verticalLayoutWidget_5)
        self.verticalLayout_6.setObjectName(u"verticalLayout_6")
        self.verticalLayout_6.setContentsMargins(0, 0, 0, 0)
        self.force_val_graph = QFrame(self.verticalLayoutWidget_5)
        self.force_val_graph.setObjectName(u"force_val_graph")
        self.force_val_graph.setFrameShape(QFrame.StyledPanel)
        self.force_val_graph.setFrameShadow(QFrame.Raised)

        self.verticalLayout_6.addWidget(self.force_val_graph)

        self.verticalLayoutWidget_4 = QWidget(self.other_signal_monitoring_panel)
        self.verticalLayoutWidget_4.setObjectName(u"verticalLayoutWidget_4")
        self.verticalLayoutWidget_4.setGeometry(QRect(410, 30, 401, 351))
        self.verticalLayout_5 = QVBoxLayout(self.verticalLayoutWidget_4)
        self.verticalLayout_5.setObjectName(u"verticalLayout_5")
        self.verticalLayout_5.setContentsMargins(0, 0, 0, 0)
        self.pressure_graph = QFrame(self.verticalLayoutWidget_4)
        self.pressure_graph.setObjectName(u"pressure_graph")
        self.pressure_graph.setFrameShape(QFrame.StyledPanel)
        self.pressure_graph.setFrameShadow(QFrame.Raised)

        self.verticalLayout_5.addWidget(self.pressure_graph)

        self.status_monitoring_panel_1 = QGroupBox(self.centralwidget)
        self.status_monitoring_panel_1.setObjectName(u"status_monitoring_panel_1")
        self.status_monitoring_panel_1.setGeometry(QRect(10, 10, 611, 371))
        self.horizontalLayoutWidget = QWidget(self.status_monitoring_panel_1)
        self.horizontalLayoutWidget.setObjectName(u"horizontalLayoutWidget")
        self.horizontalLayoutWidget.setGeometry(QRect(10, 30, 591, 331))
        self.horizontalLayout = QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_3 = QVBoxLayout()
        self.verticalLayout_3.setSpacing(6)
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.S1_tactile_info_label = QLabel(self.horizontalLayoutWidget)
        self.S1_tactile_info_label.setObjectName(u"S1_tactile_info_label")
        self.S1_tactile_info_label.setAlignment(Qt.AlignBottom|Qt.AlignHCenter)
        self.S1_tactile_info_label.setMargin(0)

        self.verticalLayout_3.addWidget(self.S1_tactile_info_label)

        self.horizontalLayout_11 = QHBoxLayout()
        self.horizontalLayout_11.setObjectName(u"horizontalLayout_11")
        self.widget_11 = QWidget(self.horizontalLayoutWidget)
        self.widget_11.setObjectName(u"widget_11")

        self.horizontalLayout_11.addWidget(self.widget_11)

        self.force_val_1 = QLCDNumber(self.horizontalLayoutWidget)
        self.force_val_1.setObjectName(u"force_val_1")
        font = QFont()
        font.setFamilies([u"Ubuntu"])
        font.setBold(True)
        font.setItalic(False)
        self.force_val_1.setFont(font)
        self.force_val_1.setAutoFillBackground(False)
        self.force_val_1.setStyleSheet(u"QLCDNumber {\n"
"    background-color: black;\n"
"    color: green;\n"
"    font-weight: bold;\n"
"}\n"
"")
        self.force_val_1.setFrameShape(QFrame.Box)
        self.force_val_1.setFrameShadow(QFrame.Raised)
        self.force_val_1.setLineWidth(1)
        self.force_val_1.setMidLineWidth(0)
        self.force_val_1.setSmallDecimalPoint(True)
        self.force_val_1.setSegmentStyle(QLCDNumber.Filled)
        self.force_val_1.setProperty("value", 0.000000000000000)

        self.horizontalLayout_11.addWidget(self.force_val_1)

        self.force_unit_1 = QLabel(self.horizontalLayoutWidget)
        self.force_unit_1.setObjectName(u"force_unit_1")
        self.force_unit_1.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_11.addWidget(self.force_unit_1)

        self.horizontalLayout_11.setStretch(1, 6)
        self.horizontalLayout_11.setStretch(2, 1)

        self.verticalLayout_3.addLayout(self.horizontalLayout_11)

        self.quadrant_1 = QWidget(self.horizontalLayoutWidget)
        self.quadrant_1.setObjectName(u"quadrant_1")
        sizePolicy1 = QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.quadrant_1.sizePolicy().hasHeightForWidth())
        self.quadrant_1.setSizePolicy(sizePolicy1)
        self.quadrant_1_1 = QWidget(self.quadrant_1)
        self.quadrant_1_1.setObjectName(u"quadrant_1_1")
        self.quadrant_1_1.setGeometry(QRect(-20, -30, 151, 151))

        self.verticalLayout_3.addWidget(self.quadrant_1)

        self.hline1 = QFrame(self.horizontalLayoutWidget)
        self.hline1.setObjectName(u"hline1")
        self.hline1.setFrameShape(QFrame.HLine)
        self.hline1.setFrameShadow(QFrame.Raised)

        self.verticalLayout_3.addWidget(self.hline1)

        self.S2_tactile_info_label = QLabel(self.horizontalLayoutWidget)
        self.S2_tactile_info_label.setObjectName(u"S2_tactile_info_label")
        self.S2_tactile_info_label.setAlignment(Qt.AlignCenter)

        self.verticalLayout_3.addWidget(self.S2_tactile_info_label)

        self.horizontalLayout_13 = QHBoxLayout()
        self.horizontalLayout_13.setObjectName(u"horizontalLayout_13")
        self.widget_10 = QWidget(self.horizontalLayoutWidget)
        self.widget_10.setObjectName(u"widget_10")

        self.horizontalLayout_13.addWidget(self.widget_10)

        self.force_val_2 = QLCDNumber(self.horizontalLayoutWidget)
        self.force_val_2.setObjectName(u"force_val_2")
        self.force_val_2.setFont(font)
        self.force_val_2.setStyleSheet(u"QLCDNumber {\n"
"    background-color: black;\n"
"    color: green;\n"
"    font-weight: bold;\n"
"}\n"
"")
        self.force_val_2.setFrameShape(QFrame.Box)
        self.force_val_2.setSmallDecimalPoint(True)
        self.force_val_2.setProperty("value", 0.000000000000000)

        self.horizontalLayout_13.addWidget(self.force_val_2)

        self.force_unit_2 = QLabel(self.horizontalLayoutWidget)
        self.force_unit_2.setObjectName(u"force_unit_2")
        self.force_unit_2.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_13.addWidget(self.force_unit_2)

        self.horizontalLayout_13.setStretch(1, 6)
        self.horizontalLayout_13.setStretch(2, 1)

        self.verticalLayout_3.addLayout(self.horizontalLayout_13)

        self.quadrant_2 = QWidget(self.horizontalLayoutWidget)
        self.quadrant_2.setObjectName(u"quadrant_2")
        sizePolicy1.setHeightForWidth(self.quadrant_2.sizePolicy().hasHeightForWidth())
        self.quadrant_2.setSizePolicy(sizePolicy1)
        self.quadrant_2_2 = QWidget(self.quadrant_2)
        self.quadrant_2_2.setObjectName(u"quadrant_2_2")
        self.quadrant_2_2.setGeometry(QRect(-20, -30, 151, 151))

        self.verticalLayout_3.addWidget(self.quadrant_2)

        self.verticalLayout_3.setStretch(2, 3)
        self.verticalLayout_3.setStretch(3, 1)
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
        self.sensing_signal_monitoring_panel = QGroupBox(self.centralwidget)
        self.sensing_signal_monitoring_panel.setObjectName(u"sensing_signal_monitoring_panel")
        self.sensing_signal_monitoring_panel.setGeometry(QRect(630, 10, 921, 371))
        self.verticalLayoutWidget = QWidget(self.sensing_signal_monitoring_panel)
        self.verticalLayoutWidget.setObjectName(u"verticalLayoutWidget")
        self.verticalLayoutWidget.setGeometry(QRect(10, 30, 451, 331))
        self.verticalLayout = QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.S1_graph = QFrame(self.verticalLayoutWidget)
        self.S1_graph.setObjectName(u"S1_graph")
        self.S1_graph.setFrameShape(QFrame.StyledPanel)
        self.S1_graph.setFrameShadow(QFrame.Raised)

        self.verticalLayout.addWidget(self.S1_graph)

        self.verticalLayoutWidget_3 = QWidget(self.sensing_signal_monitoring_panel)
        self.verticalLayoutWidget_3.setObjectName(u"verticalLayoutWidget_3")
        self.verticalLayoutWidget_3.setGeometry(QRect(460, 30, 451, 331))
        self.verticalLayout_4 = QVBoxLayout(self.verticalLayoutWidget_3)
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.verticalLayout_4.setContentsMargins(0, 0, 0, 0)
        self.S2_graph = QFrame(self.verticalLayoutWidget_3)
        self.S2_graph.setObjectName(u"S2_graph")
        self.S2_graph.setFrameShape(QFrame.StyledPanel)
        self.S2_graph.setFrameShadow(QFrame.Raised)

        self.verticalLayout_4.addWidget(self.S2_graph)

        self.status_monitoring_panel_2 = QGroupBox(self.centralwidget)
        self.status_monitoring_panel_2.setObjectName(u"status_monitoring_panel_2")
        self.status_monitoring_panel_2.setGeometry(QRect(370, 540, 351, 241))
        self.verticalLayoutWidget_6 = QWidget(self.status_monitoring_panel_2)
        self.verticalLayoutWidget_6.setObjectName(u"verticalLayoutWidget_6")
        self.verticalLayoutWidget_6.setGeometry(QRect(10, 30, 331, 201))
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

        self.verticalLayout_12 = QVBoxLayout()
        self.verticalLayout_12.setObjectName(u"verticalLayout_12")
        self.widget_3 = QWidget(self.verticalLayoutWidget_6)
        self.widget_3.setObjectName(u"widget_3")

        self.verticalLayout_12.addWidget(self.widget_3)


        self.verticalLayout_9.addLayout(self.verticalLayout_12)

        self.horizontalLayout_16 = QHBoxLayout()
        self.horizontalLayout_16.setSpacing(9)
        self.horizontalLayout_16.setObjectName(u"horizontalLayout_16")
        self.motor_pos_reading_bar_1 = QSlider(self.verticalLayoutWidget_6)
        self.motor_pos_reading_bar_1.setObjectName(u"motor_pos_reading_bar_1")
        self.motor_pos_reading_bar_1.setOrientation(Qt.Horizontal)

        self.horizontalLayout_16.addWidget(self.motor_pos_reading_bar_1)

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
        self.horizontalLayout_16.setStretch(1, 1)
        self.horizontalLayout_16.setStretch(2, 2)

        self.verticalLayout_9.addLayout(self.horizontalLayout_16)

        self.verticalLayout_13 = QVBoxLayout()
        self.verticalLayout_13.setObjectName(u"verticalLayout_13")
        self.widget_4 = QWidget(self.verticalLayoutWidget_6)
        self.widget_4.setObjectName(u"widget_4")

        self.verticalLayout_13.addWidget(self.widget_4)


        self.verticalLayout_9.addLayout(self.verticalLayout_13)

        self.verticalLayout_9.setStretch(0, 1)
        self.verticalLayout_9.setStretch(1, 1)
        self.verticalLayout_9.setStretch(2, 5)
        self.verticalLayout_9.setStretch(3, 1)

        self.verticalLayout_7.addLayout(self.verticalLayout_9)

        self.hline1_5 = QFrame(self.verticalLayoutWidget_6)
        self.hline1_5.setObjectName(u"hline1_5")
        self.hline1_5.setFrameShape(QFrame.HLine)
        self.hline1_5.setFrameShadow(QFrame.Raised)

        self.verticalLayout_7.addWidget(self.hline1_5)

        self.verticalLayout_17 = QVBoxLayout()
        self.verticalLayout_17.setObjectName(u"verticalLayout_17")
        self.horizontalLayout_17 = QHBoxLayout()
        self.horizontalLayout_17.setSpacing(0)
        self.horizontalLayout_17.setObjectName(u"horizontalLayout_17")
        self.horizontalLayout_17.setContentsMargins(0, 0, 0, 0)
        self.stable_flag_indicator = QLabel(self.verticalLayoutWidget_6)
        self.stable_flag_indicator.setObjectName(u"stable_flag_indicator")
        self.stable_flag_indicator.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_17.addWidget(self.stable_flag_indicator)

        self.contact_detect_indicator = QLabel(self.verticalLayoutWidget_6)
        self.contact_detect_indicator.setObjectName(u"contact_detect_indicator")
        self.contact_detect_indicator.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_17.addWidget(self.contact_detect_indicator)


        self.verticalLayout_17.addLayout(self.horizontalLayout_17)

        self.horizontalLayout_18 = QHBoxLayout()
        self.horizontalLayout_18.setSpacing(0)
        self.horizontalLayout_18.setObjectName(u"horizontalLayout_18")
        self.horizontalLayout_18.setContentsMargins(0, 0, 0, 0)
        self.grasp_stable_flag_indicator = QLabel(self.verticalLayoutWidget_6)
        self.grasp_stable_flag_indicator.setObjectName(u"grasp_stable_flag_indicator")
        self.grasp_stable_flag_indicator.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_18.addWidget(self.grasp_stable_flag_indicator)

        self.motor_stop_indicator = QLabel(self.verticalLayoutWidget_6)
        self.motor_stop_indicator.setObjectName(u"motor_stop_indicator")
        self.motor_stop_indicator.setAutoFillBackground(False)
        self.motor_stop_indicator.setScaledContents(False)
        self.motor_stop_indicator.setAlignment(Qt.AlignCenter)
        self.motor_stop_indicator.setMargin(0)

        self.horizontalLayout_18.addWidget(self.motor_stop_indicator)


        self.verticalLayout_17.addLayout(self.horizontalLayout_18)


        self.verticalLayout_7.addLayout(self.verticalLayout_17)

        self.hline1_4 = QFrame(self.verticalLayoutWidget_6)
        self.hline1_4.setObjectName(u"hline1_4")
        self.hline1_4.setFrameShape(QFrame.HLine)
        self.hline1_4.setFrameShadow(QFrame.Raised)

        self.verticalLayout_7.addWidget(self.hline1_4)

        self.record_data_button = QPushButton(self.verticalLayoutWidget_6)
        self.record_data_button.setObjectName(u"record_data_button")

        self.verticalLayout_7.addWidget(self.record_data_button)

        self.hline1_7 = QFrame(self.verticalLayoutWidget_6)
        self.hline1_7.setObjectName(u"hline1_7")
        sizePolicy2 = QSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)
        sizePolicy2.setHorizontalStretch(0)
        sizePolicy2.setVerticalStretch(1)
        sizePolicy2.setHeightForWidth(self.hline1_7.sizePolicy().hasHeightForWidth())
        self.hline1_7.setSizePolicy(sizePolicy2)
        self.hline1_7.setFrameShape(QFrame.HLine)
        self.hline1_7.setFrameShadow(QFrame.Raised)

        self.verticalLayout_7.addWidget(self.hline1_7)

        self.verticalLayout_7.setStretch(0, 1)
        self.verticalLayout_7.setStretch(1, 2)
        self.verticalLayout_7.setStretch(2, 1)
        self.verticalLayout_7.setStretch(3, 1)
        self.verticalLayout_7.setStretch(4, 1)
        self.verticalLayout_7.setStretch(5, 1)
        self.testing_control_panel = QGroupBox(self.centralwidget)
        self.testing_control_panel.setObjectName(u"testing_control_panel")
        self.testing_control_panel.setGeometry(QRect(10, 470, 351, 311))
        self.verticalLayoutWidget_9 = QWidget(self.testing_control_panel)
        self.verticalLayoutWidget_9.setObjectName(u"verticalLayoutWidget_9")
        self.verticalLayoutWidget_9.setGeometry(QRect(10, 30, 331, 271))
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

        self.verticalLayout_10 = QVBoxLayout()
        self.verticalLayout_10.setObjectName(u"verticalLayout_10")
        self.widget = QWidget(self.verticalLayoutWidget_9)
        self.widget.setObjectName(u"widget")

        self.verticalLayout_10.addWidget(self.widget)


        self.verticalLayout_18.addLayout(self.verticalLayout_10)

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
        self.motor_pos_ctrl_bar_1.setOrientation(Qt.Horizontal)

        self.horizontalLayout_15.addWidget(self.motor_pos_ctrl_bar_1)

        self.motor_pos_ctrl_label = QLabel(self.verticalLayoutWidget_9)
        self.motor_pos_ctrl_label.setObjectName(u"motor_pos_ctrl_label")

        self.horizontalLayout_15.addWidget(self.motor_pos_ctrl_label)

        self.motor_pos_ctrl_bar_2 = QSlider(self.verticalLayoutWidget_9)
        self.motor_pos_ctrl_bar_2.setObjectName(u"motor_pos_ctrl_bar_2")
        sizePolicy4.setHeightForWidth(self.motor_pos_ctrl_bar_2.sizePolicy().hasHeightForWidth())
        self.motor_pos_ctrl_bar_2.setSizePolicy(sizePolicy4)
        self.motor_pos_ctrl_bar_2.setOrientation(Qt.Horizontal)
        self.motor_pos_ctrl_bar_2.setInvertedAppearance(True)

        self.horizontalLayout_15.addWidget(self.motor_pos_ctrl_bar_2)

        self.horizontalLayout_15.setStretch(0, 1)
        self.horizontalLayout_15.setStretch(1, 1)
        self.horizontalLayout_15.setStretch(2, 1)

        self.verticalLayout_18.addLayout(self.horizontalLayout_15)

        self.verticalLayout_11 = QVBoxLayout()
        self.verticalLayout_11.setObjectName(u"verticalLayout_11")
        self.widget_2 = QWidget(self.verticalLayoutWidget_9)
        self.widget_2.setObjectName(u"widget_2")

        self.verticalLayout_11.addWidget(self.widget_2)


        self.verticalLayout_18.addLayout(self.verticalLayout_11)

        self.verticalLayout_18.setStretch(0, 1)
        self.verticalLayout_18.setStretch(1, 1)
        self.verticalLayout_18.setStretch(2, 4)
        self.verticalLayout_18.setStretch(3, 1)

        self.testing_control_panel_layout.addLayout(self.verticalLayout_18)

        self.hline1_15 = QFrame(self.verticalLayoutWidget_9)
        self.hline1_15.setObjectName(u"hline1_15")
        self.hline1_15.setFrameShape(QFrame.HLine)
        self.hline1_15.setFrameShadow(QFrame.Raised)

        self.testing_control_panel_layout.addWidget(self.hline1_15)

        self.horizontalLayout_4 = QHBoxLayout()
        self.horizontalLayout_4.setObjectName(u"horizontalLayout_4")
        self.pressure_control_label = QLabel(self.verticalLayoutWidget_9)
        self.pressure_control_label.setObjectName(u"pressure_control_label")
        self.pressure_control_label.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_4.addWidget(self.pressure_control_label)

        self.widget_5 = QWidget(self.verticalLayoutWidget_9)
        self.widget_5.setObjectName(u"widget_5")

        self.horizontalLayout_4.addWidget(self.widget_5)

        self.force_feedback_ctrl_enable = QCheckBox(self.verticalLayoutWidget_9)
        self.force_feedback_ctrl_enable.setObjectName(u"force_feedback_ctrl_enable")

        self.horizontalLayout_4.addWidget(self.force_feedback_ctrl_enable)

        self.horizontalLayout_4.setStretch(0, 3)
        self.horizontalLayout_4.setStretch(1, 1)
        self.horizontalLayout_4.setStretch(2, 2)

        self.testing_control_panel_layout.addLayout(self.horizontalLayout_4)

        self.Pressure_ScrollBar = QScrollBar(self.verticalLayoutWidget_9)
        self.Pressure_ScrollBar.setObjectName(u"Pressure_ScrollBar")
        sizePolicy5 = QSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Preferred)
        sizePolicy5.setHorizontalStretch(0)
        sizePolicy5.setVerticalStretch(1)
        sizePolicy5.setHeightForWidth(self.Pressure_ScrollBar.sizePolicy().hasHeightForWidth())
        self.Pressure_ScrollBar.setSizePolicy(sizePolicy5)
        self.Pressure_ScrollBar.setMaximum(35)
        self.Pressure_ScrollBar.setSingleStep(1)
        self.Pressure_ScrollBar.setOrientation(Qt.Horizontal)

        self.testing_control_panel_layout.addWidget(self.Pressure_ScrollBar)

        self.horizontalLayout_10 = QHBoxLayout()
        self.horizontalLayout_10.setObjectName(u"horizontalLayout_10")
        self.current_pressure_label = QLabel(self.verticalLayoutWidget_9)
        self.current_pressure_label.setObjectName(u"current_pressure_label")
        self.current_pressure_label.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_10.addWidget(self.current_pressure_label)

        self.widget_9 = QWidget(self.verticalLayoutWidget_9)
        self.widget_9.setObjectName(u"widget_9")

        self.horizontalLayout_10.addWidget(self.widget_9)

        self.ctrl_pressure_val = QLCDNumber(self.verticalLayoutWidget_9)
        self.ctrl_pressure_val.setObjectName(u"ctrl_pressure_val")

        self.horizontalLayout_10.addWidget(self.ctrl_pressure_val)

        self.pressure_unit = QLabel(self.verticalLayoutWidget_9)
        self.pressure_unit.setObjectName(u"pressure_unit")
        self.pressure_unit.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_10.addWidget(self.pressure_unit)

        self.horizontalLayout_10.setStretch(0, 3)
        self.horizontalLayout_10.setStretch(1, 1)
        self.horizontalLayout_10.setStretch(2, 3)

        self.testing_control_panel_layout.addLayout(self.horizontalLayout_10)

        self.hline1_3 = QFrame(self.verticalLayoutWidget_9)
        self.hline1_3.setObjectName(u"hline1_3")
        self.hline1_3.setFrameShape(QFrame.HLine)
        self.hline1_3.setFrameShadow(QFrame.Raised)

        self.testing_control_panel_layout.addWidget(self.hline1_3)

        self.horizontalLayout_5 = QHBoxLayout()
        self.horizontalLayout_5.setObjectName(u"horizontalLayout_5")
        self.contact_detect_enable = QCheckBox(self.verticalLayoutWidget_9)
        self.contact_detect_enable.setObjectName(u"contact_detect_enable")

        self.horizontalLayout_5.addWidget(self.contact_detect_enable)

        self.sofa_simulation_enable = QCheckBox(self.verticalLayoutWidget_9)
        self.sofa_simulation_enable.setObjectName(u"sofa_simulation_enable")

        self.horizontalLayout_5.addWidget(self.sofa_simulation_enable)


        self.testing_control_panel_layout.addLayout(self.horizontalLayout_5)

        self.hline1_8 = QFrame(self.verticalLayoutWidget_9)
        self.hline1_8.setObjectName(u"hline1_8")
        self.hline1_8.setFrameShape(QFrame.HLine)
        self.hline1_8.setFrameShadow(QFrame.Raised)

        self.testing_control_panel_layout.addWidget(self.hline1_8)

        self.horizontalLayout_7 = QHBoxLayout()
        self.horizontalLayout_7.setObjectName(u"horizontalLayout_7")
        self.stop_all_button_testing = QPushButton(self.verticalLayoutWidget_9)
        self.stop_all_button_testing.setObjectName(u"stop_all_button_testing")

        self.horizontalLayout_7.addWidget(self.stop_all_button_testing)


        self.testing_control_panel_layout.addLayout(self.horizontalLayout_7)

        self.testing_control_panel_layout.setStretch(0, 3)
        self.testing_control_panel_layout.setStretch(1, 1)
        self.testing_control_panel_layout.setStretch(2, 1)
        self.testing_control_panel_layout.setStretch(3, 1)
        self.testing_control_panel_layout.setStretch(4, 1)
        self.testing_control_panel_layout.setStretch(5, 1)
        self.testing_control_panel_layout.setStretch(6, 2)
        self.testing_control_panel_layout.setStretch(7, 1)
        self.testing_control_panel_layout.setStretch(8, 2)
        self.operating_mode_selection_panel = QGroupBox(self.centralwidget)
        self.operating_mode_selection_panel.setObjectName(u"operating_mode_selection_panel")
        self.operating_mode_selection_panel.setGeometry(QRect(10, 390, 351, 71))
        self.horizontalLayoutWidget_3 = QWidget(self.operating_mode_selection_panel)
        self.horizontalLayoutWidget_3.setObjectName(u"horizontalLayoutWidget_3")
        self.horizontalLayoutWidget_3.setGeometry(QRect(10, 30, 331, 31))
        self.horizontalLayout_12 = QHBoxLayout(self.horizontalLayoutWidget_3)
        self.horizontalLayout_12.setObjectName(u"horizontalLayout_12")
        self.horizontalLayout_12.setContentsMargins(0, 0, 0, 0)
        self.model_selection_comboBox = QComboBox(self.horizontalLayoutWidget_3)
        self.model_selection_comboBox.setObjectName(u"model_selection_comboBox")

        self.horizontalLayout_12.addWidget(self.model_selection_comboBox)

        MainWindow.setCentralWidget(self.centralwidget)
        self.sensing_signal_monitoring_panel.raise_()
        self.status_monitoring_panel_1.raise_()
        self.other_signal_monitoring_panel.raise_()
        self.automatic_grasping_control_panel.raise_()
        self.status_monitoring_panel_2.raise_()
        self.testing_control_panel.raise_()
        self.operating_mode_selection_panel.raise_()
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 1558, 23))
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
        self.automatic_grasping_control_panel.setTitle(QCoreApplication.translate("MainWindow", u"Automatic Grasping Control Panel", None))
        self.release_button.setText(QCoreApplication.translate("MainWindow", u"Release", None))
        self.restart_button.setText(QCoreApplication.translate("MainWindow", u"Restart", None))
        self.stop_all_button_grasping.setText(QCoreApplication.translate("MainWindow", u"STOP ALL", None))
        self.other_signal_monitoring_panel.setTitle(QCoreApplication.translate("MainWindow", u"Other Signal Monitoring Panel", None))
        self.status_monitoring_panel_1.setTitle(QCoreApplication.translate("MainWindow", u"Status Monitoring Panel 1", None))
        self.S1_tactile_info_label.setText(QCoreApplication.translate("MainWindow", u"S1 Tactile Info", None))
        self.force_unit_1.setText(QCoreApplication.translate("MainWindow", u"N", None))
        self.S2_tactile_info_label.setText(QCoreApplication.translate("MainWindow", u"S2 Tactile Info", None))
        self.force_unit_2.setText(QCoreApplication.translate("MainWindow", u"N", None))
        self.sensing_signal_monitoring_panel.setTitle(QCoreApplication.translate("MainWindow", u"Sensing Signal Monitoring Panel", None))
        self.status_monitoring_panel_2.setTitle(QCoreApplication.translate("MainWindow", u"Status Monitoring Panel 2", None))
        self.motor_pos_reading_label.setText(QCoreApplication.translate("MainWindow", u"Motor Position", None))
        self.stable_flag_indicator.setText(QCoreApplication.translate("MainWindow", u"Dataprocess Stable", None))
        self.contact_detect_indicator.setText(QCoreApplication.translate("MainWindow", u"Contact Detect", None))
        self.grasp_stable_flag_indicator.setText(QCoreApplication.translate("MainWindow", u"Grasp Stable", None))
        self.motor_stop_indicator.setText(QCoreApplication.translate("MainWindow", u"Motor Stop", None))
        self.record_data_button.setText(QCoreApplication.translate("MainWindow", u"Record Data", None))
        self.testing_control_panel.setTitle(QCoreApplication.translate("MainWindow", u"Testing Panel", None))
        self.motor_pos_ctrl_label.setText(QCoreApplication.translate("MainWindow", u"Motor Control", None))
        self.pressure_control_label.setText(QCoreApplication.translate("MainWindow", u"Pressure Control", None))
        self.force_feedback_ctrl_enable.setText(QCoreApplication.translate("MainWindow", u"Force Control", None))
        self.current_pressure_label.setText(QCoreApplication.translate("MainWindow", u"Input Pressure", None))
        self.pressure_unit.setText(QCoreApplication.translate("MainWindow", u"kPa", None))
        self.contact_detect_enable.setText(QCoreApplication.translate("MainWindow", u"Contact Detect", None))
        self.sofa_simulation_enable.setText(QCoreApplication.translate("MainWindow", u"SOFA Simulation", None))
        self.stop_all_button_testing.setText(QCoreApplication.translate("MainWindow", u"STOP ALL", None))
        self.operating_mode_selection_panel.setTitle(QCoreApplication.translate("MainWindow", u"Operating Mode Selection", None))
        self.toolBar.setWindowTitle(QCoreApplication.translate("MainWindow", u"toolBar", None))
    # retranslateUi

