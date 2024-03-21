# encoding: utf-8
# !/usr/bin/python3

from PySide6.QtOpenGLWidgets import QOpenGLWidget
from PySide6.QtCore import QTimer
import Sofa
import SofaRuntime
import Sofa.Gui

class SofaQtViewer(QOpenGLWidget):
    def __init__(self, parent=None):
        super(SofaQtViewer, self).__init__(parent)
        # self.timer = QTimer(self)
        # self.timer.timeout.connect(self.stepSimulation)
        self.root = None
        self.createScene()
        Sofa.Simulation.init(self.root)
        # self.timer.start(16)  # Adjust the interval to control simulation speed

    def createScene(self):
        self.root = Sofa.Core.Node("root")
        self.root.addObject('RequiredPlugin', name='Sofa.Component.StateContainer')

        # Scene must now include a AnimationLoop
        self.root.addObject('DefaultAnimationLoop')

        # Add new nodes and objects in the scene
        node1 = self.root.addChild("Node1")
        node2 = self.root.addChild("Node2")

        node1.addObject("MechanicalObject", template="Rigid3d", position="0 0 0   0 0 0 1", showObject="1")

        node2.addObject("MechanicalObject", template="Rigid3d", position="1 1 1   0 0 0 1", showObject="1")

        return self.root

    def stepSimulation(self):
        if self.root is not None:
            Sofa.Simulation.animate(self.root, 0.016)  # 16 ms time step
            self.update()  # Trigger the repaint

    def paintGL(self):
        # This method is called when the widget needs to be painted
        # Render the SOFA scene here.
        pass

    def resizeGL(self, width, height):
        # Update the viewport and other settings when the widget is resized
        pass
