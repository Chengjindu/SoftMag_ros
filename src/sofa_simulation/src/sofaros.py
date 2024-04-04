#!/usr/bin/env python3
# coding: utf8
import Sofa.Core
import rospy
from std_msgs.msg import Int32, Float32  # or the appropriate message type you are using


def precb(data):
    p = data.data
    # rospy.loginfo(f'the received pressure is {p}')
    return p


def poscb(data):
    d = data.data
    # rospy.loginfo(f'the received position is {d}')
    return d


def moveRestPos(rest_pos, dx, dy, dz):
    out = []
    for i in range(len(rest_pos)):
        out.append([rest_pos[i][0] + dx, rest_pos[i][1] + dy, rest_pos[i][2] + dz])
    return out

class RosSender(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        # to reduce the latency in TCP, we can disable Nagle's algo with tcp_nodelay=False in ROS1
        # (https://en.wikipedia.org/wiki/Nagle%27s_algorithm)
        # Todo: find the equivalent in ROS2
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.name = "RosSender"

        # Args
        self.node = rospy.Publisher(args[1], Float32, queue_size=10)  # rosname
        self.datafield = args[2]
        self.sendingcb = args[4]

    def onAnimateEndEvent(self, event):
        data = Float32()
        data.data = self.sendingcb(self.datafield)
        self.node.publish(data)

class RosReceiver(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        # Assuming the SOFA node is passed as a keyword argument:
        self.snode = kwargs.get('node')
        if self.snode is None:
            raise ValueError("SOFA node is required for RosReceiver initialization")

        self.constraints = []
        self.dofs = []

        for i in range(1, 3):
            self.dofs.append(self.snode.getChild('finger' + str(i)).tetras)
            self.constraints.append(self.snode.getChild('finger' + str(i)).cavity.SurfacePressureConstraint)

            # fingerNode = self.snode.getChild('finger' + str(i))
            # if not fingerNode:
            #     print(f"Error: finger {i} node not found in RosReceiver.")
            #     continue
            # tetrasNode = fingerNode.getObject('tetras')
            # if not tetrasNode:
            #     print(f"Error: tetras node not found in finger {i}.")
            #     continue
            #
            # self.dofs.append(tetrasNode)
            #
            # constraint = fingerNode.cavity.getObject('SurfacePressureConstraint')
            # if not constraint:
            #     print(f"Error: SurfacePressureConstraint node not found in finger {i} in RosReceiver.")
            #     continue
            # self.constraints.append(constraint)

        self.name = "RosReceiver"

        # rospy.Subscriber("/animation/receiver/pressure", Float32, self.pressureCB)
        rospy.Subscriber("/pressure", Float32, self.pressureCB)
        # rospy.Subscriber("/animation/receiver/position", Float32, self.positionCB)
        rospy.Subscriber("/motor_pos", Int32, self.positionCB)

        self.predata = None
        self.posdata = None

    def pressureCB(self, data):
        self.predata = data.data

    def positionCB(self, data):
        self.posdata = data.data

    def onAnimateBeginEvent(self, event):
        if self.predata is not None:
            p = precb(Float32(self.predata))
            for i in range(2):
                self.constraints[i].value[0] = p * self.snode.dt.value
                rospy.loginfo(f'Current Current pressure = {p} kPa')
                # new_positions = [[x, y, 50-80] for x, y, z in self.dofs[1].rest_position.value]
                # self.dofs[1].rest_position.value = new_positions
                # rospy.loginfo(f'Current Z position = {self.dofs[1].rest_position.value[0][2]} mm')
            self.predata = None

        if self.posdata is not None:
            d = poscb(Float32(self.posdata))
            rospy.loginfo(f'Current Z = {self.dofs[1].rest_position.value[0][2]} mm')
            new_positions = [[x, y, d * 1.0638 -75] for x, y, z in self.dofs[1].rest_position.value]
            self.dofs[1].rest_position.value = new_positions
            # self.dofs[1].rest_position.value[0][2] = d * 1.0638 -75
            rospy.loginfo(f'Current Z position = {d * -1.0638 + 80} mm')
            self.posdata = None

def init(nodeName="Sofa"):
    rospy.init_node(nodeName, anonymous=True)
    rospy.loginfo('Created node')
    # In ROS1, the node doesn't have to be returned as rospy handles this internally