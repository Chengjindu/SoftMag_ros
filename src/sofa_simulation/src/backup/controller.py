#!/usr/bin/env python
# -*- coding: utf-8 -*-
import Sofa.Core
from Sofa.constants import *
import math


def moveRestPos(rest_pos, dx, dy, dz):
    out = []
    for i in range(0,len(rest_pos)) :
        out += [[rest_pos[i][0]+dx, rest_pos[i][1]+dy, rest_pos[i][2]+dz]]
    return out


def rotateRestPos(rest_pos,rx,centerPosY,centerPosZ):
    out = []
    for i in range(0,len(rest_pos)) :
        newRestPosY = (rest_pos[i][1] - centerPosY)*math.cos(rx) - (rest_pos[i][2] - centerPosZ)*math.sin(rx) +  centerPosY
        newRestPosZ = (rest_pos[i][1] - centerPosY)*math.sin(rx) + (rest_pos[i][2] - centerPosZ)*math.cos(rx) +  centerPosZ
        out += [[rest_pos[i][0], newRestPosY, newRestPosZ]]
    return out


class WholeGripperController(Sofa.Core.Controller):

    def __init__(self, *a, **kw):

        Sofa.Core.Controller.__init__(self, *a, **kw)
        self.node = kw["node"]
        self.constraints = []
        self.dofs = []
        for i in range(1,3):
            self.dofs.append(self.node.getChild('finger' + str(i)).tetras)
            self.constraints.append(self.node.getChild('finger'+str(i)).cavity.SurfacePressureConstraint)

        self.centerPosY = 0
        self.centerPosZ = -20
        self.rotAngle = 0

        return

    def onKeypressedEvent(self,e):

        increment = 1*self.node.dt.value # X[kPa]*dt

        if e["key"] == Sofa.constants.Key.plus:
            for i in range(2):
                pressureValue = self.constraints[i].value[0] + increment
                if pressureValue > 200:
                    pressureValue = 200
                self.constraints[i].value[0] = pressureValue
                print (f'Value = {pressureValue/self.node.dt.value} kPa')

        if e["key"] == Sofa.constants.Key.minus:
            for i in range(2):
                pressureValue = self.constraints[i].value[0] - increment
                if pressureValue < 0:
                    pressureValue = 0
                self.constraints[i].value[0] = pressureValue
                print (f'Value = {pressureValue/self.node.dt.value} kPa')

        elif e["key"] == Sofa.constants.Key.uparrow:
            for i in range(2):
                results = moveRestPos(self.dofs[i].rest_position.value, 3.0, 0.0, 0.0)
                self.dofs[i].rest_position.value = results

        elif e["key"] == Sofa.constants.Key.downarrow:
            for i in range(2):
                results = moveRestPos(self.dofs[i].rest_position.value, -3.0, 0.0, 0.0)
                self.dofs[i].rest_position.value = results

        elif e["key"] == Sofa.constants.Key.leftarrow:
            dy = 3.0*math.cos(self.rotAngle)
            dz = 3.0*math.sin(self.rotAngle)
            for i in range(2):
                results = moveRestPos(self.dofs[i].rest_position.value, 0.0, dy, dz)
                self.dofs[i].rest_position.value = results
            self.centerPosY = self.centerPosY + dy
            self.centerPosZ = self.centerPosZ + dz

        elif e["key"] == Sofa.constants.Key.rightarrow:
            dy = -3.0*math.cos(self.rotAngle)
            dz = -3.0*math.sin(self.rotAngle)
            for i in range(2):
                results = moveRestPos(self.dofs[i].rest_position.value, 0.0, dy, dz)
                self.dofs[i].rest_position.value = results
            self.centerPosY = self.centerPosY + dy
            self.centerPosZ = self.centerPosZ + dz

        # Direct rotation
        elif e["key"] == "A":
            for i in range(2):
                results = rotateRestPos(self.dofs[i].rest_position.value, math.pi / 16, self.centerPosY, self.centerPosZ)
                self.dofs[i].rest_position.value = results
            self.rotAngle = self.rotAngle + math.pi/16

        # Indirect rotation
        elif e["key"] == "Q":
            for i in range(2):
                results = rotateRestPos(self.dofs[i].rest_position.value, -math.pi / 16, self.centerPosY, self.centerPosZ)
                self.dofs[i].rest_position.value = results
            self.rotAngle = self.rotAngle - math.pi/16
