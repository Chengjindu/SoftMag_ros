#!/usr/bin/env python3
# coding: utf8

import os
import rospy
from std_msgs.msg import Float32MultiArray  # wrapper for ROS primitive types, see : https://github.com/ros2/common_interfaces/tree/master/std_msgs

import Sofa
import Sofa.Gui
import SofaRuntime
import math
from controller import WholeGripperController
# from splib3.loaders import loadPointListFromFile
import sofaros

import signal
import sys


# Get the directory of your script
script_directory = os.path.dirname(os.path.abspath(__file__))


# Fingers information
youngModulusFingers = 70 #kPa
youngModulusFabric = 7500000 #kPa
youngModulusFoam = 130 #kPa
poissonRatioFingers = 0.4
poissonRatioFabric = 0.2
poissonRatioFoam = 0.4
foamthickness = '8' #millimeters
fingerMass = 0.030 #kg
fabricMass = 0.005 #kg
foamMass = 0.005 #kg
translateFinger1 = [0, 0, 0]
translateFinger2 = [0, 0, -75]
translations = [translateFinger1, translateFinger2]
rotationFinger1 = [0, 0, -90]
rotationFinger2 = [0, 180, -90]
rotations = [rotationFinger1, rotationFinger2]


def createScene(rootNode):
    rootNode.addObject('RequiredPlugin',
                       pluginName='SoftRobots SofaPython3 SofaLoader SofaSimpleFem SofaEngine SofaDeformable SofaImplicitOdeSolver SofaConstraint SofaSparseSolver SofaMeshCollision SofaRigid SofaOpenglVisual Sofa.Component.Collision.Detection.Algorithm')
    rootNode.addObject('VisualStyle',
                       displayFlags='showVisualModels hideBehaviorModels hideCollisionModels hideBoundingCollisionModels hideForceFields showInteractionForceFields hideWireframe')
    rootNode.dt = 0.01
    rootNode.gravity.value = [-9810, 0, 0]
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', tolerance=1e-12, maxIterations=10000)
    rootNode.addObject('DefaultPipeline')
    rootNode.addObject('BruteForceBroadPhase')
    rootNode.addObject('BVHNarrowPhase')
    rootNode.addObject('DefaultContactManager', response='FrictionContactConstraint', responseParams='mu=0.99')
    rootNode.addObject('LocalMinDistance', name='Proximity', alarmDistance=10, contactDistance=1, angleCone=0.0)

    rootNode.addObject('BackgroundSetting', color=[0.2, 0.22, 0.25, 1.])
    rootNode.addObject('OglSceneFrame', style='Arrows', alignment='TopRight')
    
    for i in range(2):
        ####################################sofaros.init()######
        # Finger Model	 						 #
        ##########################################
        finger = rootNode.addChild('finger' + str(i + 1))
        finger.addObject('EulerImplicitSolver', name='odesolver', rayleighStiffness=0.1, rayleighMass=0.1)
        finger.addObject('SparseLDLSolver', name='preconditioner')

        ##########################################
        # Mechanics             				  #
        ##########################################
        # Use it to construct the absolute path to the mesh file
        finger_mesh_file_path = os.path.join(script_directory, 'data', 'mesh', 'border8.msh')
        finger.addObject('MeshGmshLoader', name='loader', filename=finger_mesh_file_path, rotation=rotations[i], translation=translations[i])
        finger.addObject('MeshTopology', src='@loader', name='container')
        finger.addObject('MechanicalObject', name='tetras', template='Vec3d')
        finger.addObject('UniformMass', totalMass=fingerMass)
        finger.addObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio=poissonRatioFingers,
                         youngModulus=youngModulusFingers)
        finger.addObject('LinearSolverConstraintCorrection', solverName='preconditioner')

        if (i+1) == 1:
            ff = rootNode.finger1.loader.position
        else:
            ff = rootNode.finger2.loader.position
        xmin = min(ff[:,0])
        xmax = max(ff[:,0])
        ymin = min(ff[:,1])
        ymax = max(ff[:,1])  
        zmin = min(ff[:,2])
        zmax = max(ff[:,2])
        
        finger.addObject('BoxROI', name='boxROI', box=[xmin, ymin, zmin, xmin+10, ymax, zmax], drawBoxes=False, doUpdate=False)
        finger.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e12, angularStiffness=1e12)

        
        ##########################################
        # Fabric Sub topology			   #
        ########################################## 
        fabric = finger.addChild('fabric' + str(i + 1))
        fabric_mesh_file_path = os.path.join(script_directory, 'data', 'mesh', 'fabric2.msh')
        fabric.addObject('MeshGmshLoader', name='loader', filename=fabric_mesh_file_path, rotation=rotations[i], translation=translations[i])
        fabric.addObject('MeshTopology', src='@loader', name='topocontainer')
        fabric.addObject('MechanicalObject', name='tetras', template='Vec3d')
        fabric.addObject('UniformMass', totalMass=fabricMass)
        fabric.addObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio=poissonRatioFabric,
                        youngModulus=youngModulusFabric)
        fabric.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False)

        ##########################################
        # Foam Sub topology			   #
        ##########################################
        foam = finger.addChild('foam' + str(i + 1))
        foam_mesh_file_path = os.path.join(script_directory, 'data', 'mesh', 'foam'+foamthickness+'.msh')
        foam.addObject('MeshGmshLoader', name='loader', filename=foam_mesh_file_path, rotation=rotations[i], translation=translations[i])
        foam.addObject('MeshTopology', src='@loader', name='container')
        foam.addObject('MechanicalObject', name='tetras', template='Vec3d')
        foam.addObject('UniformMass', totalMass=foamMass)
        foam.addObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio=poissonRatioFoam,
                         youngModulus=youngModulusFoam)
        foam.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False)
        
        ##########################################
        # Cavity Constraint						 #
        ##########################################
        cavity = finger.addChild('cavity')
        cavity_file_path = os.path.join(script_directory, 'data', 'mesh', 'tchambers.stl')
        cavity.addObject('MeshSTLLoader', name='loader', filename=cavity_file_path, translation=translations[i], rotation=rotations[i])
        cavity.addObject('MeshTopology', src='@loader', name='topo')
        cavity.addObject('MechanicalObject', name='cavity')
        cavity.addObject('SurfacePressureConstraint', name='SurfacePressureConstraint', template='Vec3d', value=0.0001,
                          triangles='@topo.triangles', valueType='pressure')
        cavity.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False)

        ##########################################
        # Collision							  #
        ##########################################
        collisionFinger = finger.addChild('collisionFinger')
        collisionFinger_file_path = os.path.join(script_directory, 'data', 'mesh', 'border8.stl')
        collisionFinger.addObject('MeshSTLLoader', name='loader', filename=collisionFinger_file_path, translation=translations[i], rotation=rotations[i])
        collisionFinger.addObject('MeshTopology', src='@loader', name='topo')
        collisionFinger.addObject('MechanicalObject', name='collisMech')
        collisionFinger.addObject('TriangleCollisionModel', selfCollision=False)
        collisionFinger.addObject('LineCollisionModel', selfCollision=False)
        collisionFinger.addObject('PointCollisionModel', selfCollision=False)
        collisionFinger.addObject('BarycentricMapping')

        # collisionFabric = fabric.addChild('collisionFabric')
        # collisionFabric.addObject('MeshSTLLoader', name='loader', filename='data/mesh/fabric2.stl', translation=translations[i], rotation=rotations[i])
        # collisionFabric.addObject('MeshTopology', src='@loader', name='topo')
        # collisionFabric.addObject('MechanicalObject', name='collisMech')
        # collisionFabric.addObject('TriangleCollisionModel', selfCollision=False)
        # collisionFabric.addObject('LineCollisionModel', selfCollision=False)
        # collisionFabric.addObject('PointCollisionModel', selfCollision=False)
        # collisionFabric.addObject('BarycentricMapping')

        # collisionFoam = foam.addChild('collisionFoam')
        # collisionFoam.addObject('MeshSTLLoader', name='loader', filename='data/mesh/foam8.stl', translation=translations[i], rotation=rotations[i])
        # collisionFoam.addObject('MeshTopology', src='@loader', name='topo')
        # collisionFoam.addObject('MechanicalObject', name='collisMech')
        # collisionFoam.addObject('TriangleCollisionModel', selfCollision=False)
        # collisionFoam.addObject('LineCollisionModel', selfCollision=False)
        # collisionFoam.addObject('PointCollisionModel', selfCollision=False)
        # collisionFoam.addObject('BarycentricMapping')

        ##########################################
        # Visualizations					  #
        ##########################################
        visuFinger = finger.addChild('visuFinger')
        visuFinger_file_path = os.path.join(script_directory, 'data', 'mesh', 'conic.stl')
        visuFinger.addObject('MeshSTLLoader', name='loader', filename=visuFinger_file_path, translation=translations[i], rotation=rotations[i])
        visuFinger.addObject('OglModel', src='@loader', color=[0.7, 0.7, 0.8, 0.7])
        visuFinger.addObject('BarycentricMapping')
        
        visuFabric = fabric.addChild('visuFabric')
        visuFabric_file_path = os.path.join(script_directory, 'data', 'mesh', 'fabric2.stl')
        visuFabric.addObject('MeshSTLLoader', name='loader', filename=visuFabric_file_path, translation=translations[i], rotation=rotations[i])
        visuFabric.addObject('OglModel', src='@loader', color=[0.5, 0.3, 0.6, 0.95])
        visuFabric.addObject('BarycentricMapping')
        
        visuFoam = foam.addChild('visuFoam')
        visuFoam_file_path = os.path.join(script_directory, 'data', 'mesh', 'foam8.stl')
        visuFoam.addObject('MeshSTLLoader', name='loader', filename=visuFoam_file_path, translation=translations[i], rotation=rotations[i])
        visuFoam.addObject('OglModel', src='@loader', color=[0.9, 0.9, 0.4, 0.6])
        visuFoam.addObject('BarycentricMapping')


    rootNode.addObject(WholeGripperController(node=rootNode))

    rosNode = sofaros.init("SofaNode")
    rootNode.addObject(sofaros.RosReceiver(rosNode, Float32MultiArray, node=rootNode))

    return rootNode

def signal_handler(sig, frame):
    rospy.loginfo("SIGINT received. Shutting down gracefully...")
    Sofa.Gui.GUIManager.closeGUI()
    rospy.signal_shutdown("SIGINT received")
    sys.exit(0)

def main():
    # Initialize the ROS node
    # rospy.init_node('sofa_simulation_node', anonymous=True)

    # Signal handling for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)

    # Make sure to load all necessary libraries
    SofaRuntime.importPlugin("Sofa.Component.StateContainer")

    # This now references the createScene method defined above.
    root = Sofa.Core.Node("root")   # Initialize the root node for the simulation.
    createScene(root)   # Set up the simulation scene.

    # Once defined, initialization of the scene graph
    Sofa.Simulation.init(root)

    # Check supported GUIs
    supported_gui = Sofa.Gui.GUIManager.ListSupportedGUI(",")
    rospy.loginfo("Supported GUIs are: " + supported_gui)

    # Initialize and run the GUI
    Sofa.Gui.GUIManager.Init("simple_scene", "qglviewer") # Initialize with qglviewer or qt
    Sofa.Gui.GUIManager.createGUI(root, __file__)
    Sofa.Gui.GUIManager.MainLoop(root)
    Sofa.Gui.GUIManager.closeGUI()

    rospy.loginfo("Starting SOFA simulation with simplified model")
    while not rospy.is_shutdown():
        # Placeholder for simulation loop.
        rospy.sleep(1)  # Adjust based on simulation's needs.

    rospy.loginfo("SOFA simulation ended")

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    main()
