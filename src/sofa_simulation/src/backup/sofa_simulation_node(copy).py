#!/usr/bin/env python3

import os
import rospy
from std_msgs.msg import String  # Import ROS standard message types
import Sofa
import Sofa.Gui
import SofaRuntime

def createScene(root):
    root.addObject('RequiredPlugin', name='Sofa.Component.StateContainer')
    root.addObject('DefaultAnimationLoop')
    root.addObject('RequiredPlugin', name='SofaOpenglVisual')

    node1 = root.addChild("Node1")
    node2 = root.addChild("Node2")
    node1.addObject("MechanicalObject", template="Rigid3d", position="0 0 0   0 0 0 1", showObject="1")
    node2.addObject("MechanicalObject", template="Rigid3d", position="1 1 1   0 0 0 1", showObject="1")

def main():
    # Initialize the ROS node
    rospy.init_node('sofa_simulation_node', anonymous=True)

    # Make sure to load all necessary libraries
    SofaRuntime.importPlugin("Sofa.Component.StateContainer")

    root = Sofa.Core.Node("root")   # Initialize the root node for the simulation.
    createScene(root)   # Set up the simulation scene.

    Sofa.Simulation.init(root)  # Once defined, initialization of the scene graph

    # Check supported GUIs
    supported_gui = Sofa.Gui.GUIManager.ListSupportedGUI(",")
    print("Supported GUIs are: " + supported_gui)

    # Initialize and run the GUI
    Sofa.Gui.GUIManager.Init("simple_scene", "qt") # it can also be changed to "qglviewer"
    Sofa.Gui.GUIManager.createGUI(root, __file__)
    Sofa.Gui.GUIManager.MainLoop(root)
    Sofa.Gui.GUIManager.closeGUI()

    rospy.loginfo("Starting SOFA simulation with simplified model")
    while not rospy.is_shutdown():
        # Placeholder for simulation loop.
        rospy.sleep(1)  # Adjust based on simulation's needs.

    rospy.loginfo("SOFA simulation ended")

if __name__ == '__main__':
    main()
