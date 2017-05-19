"""Welcome to Google tech talk series for PyCon 2017!

    PyBullet: Physics Simulation for Robotics and Machine Learning
        Basic tutorial

Questions, comments? Tweet me @hellojas :)
"""


import pybullet as p
import time
import math
import os
import sys
from data_config import *
import numpy as np

POSITION_INDEX = 0
VELOCITY_INDEX = 1
MOTOR_TORQUES_INDEX = 2
JOINT_NAME_INDEX = 1
JOINT_INDEX = 0
NUM_MOTORS = 8

class Minitaur:
  """Initialize URDF filepath."""
  def __init__(self, urdfRootPath=''):
    self.urdfRootPath = urdfRootPath
    self.reset()

  """Maps the joint name to equivalent indexes."""
  def buildJointNameToIdDict(self):
    nJoints = p.getNumJoints(self.quadruped)
    self.jointNameToId = {}
    for i in range(nJoints):
      jointInfo = p.getJointInfo(self.quadruped, i)
      self.jointNameToId[
        jointInfo[JOINT_NAME_INDEX].decode('UTF-8')] = jointInfo[JOINT_INDEX]
    self.resetPose()

  """Adds all the motor joint IDs to a list."""
  def buildMotorIdList(self):
    self.motorIdList.append(self.jointNameToId['motor_front_leftL_joint'])
    self.motorIdList.append(self.jointNameToId['motor_front_leftR_joint'])
    self.motorIdList.append(self.jointNameToId['motor_back_leftL_joint'])
    self.motorIdList.append(self.jointNameToId['motor_back_leftR_joint'])
    self.motorIdList.append(self.jointNameToId['motor_front_rightL_joint'])
    self.motorIdList.append(self.jointNameToId['motor_front_rightR_joint'])
    self.motorIdList.append(self.jointNameToId['motor_back_rightL_joint'])
    self.motorIdList.append(self.jointNameToId['motor_back_rightR_joint'])

  """Sets configuation values."""
  def reset(self):
    self.quadruped = p.loadURDF("%s/quadruped/minitaur.urdf" % self.urdfRootPath,0,0,.2)
    self.kp = 1
    self.kd = 0.1
    self.maxForce = 3.5
    self.nMotors = 8
    self.motorIdList = []
    self.motorDir = [-1, -1, -1, -1, 1, 1, 1, 1]
    self.buildJointNameToIdDict()
    self.buildMotorIdList()

  """Sets the motor joints to desired angles."""
  def setMotorAngleById(self, motorId, desiredAngle):
    p.setJointMotorControl2(bodyIndex=self.quadruped, jointIndex=motorId,
        controlMode=p.POSITION_CONTROL, targetPosition=desiredAngle,
        positionGain=self.kp, velocityGain=self.kd, force=self.maxForce)

  """Sets the motor joints to desired angles (by name)."""
  def setMotorAngleByName(self, motorName, desiredAngle):
    self.setMotorAngleById(self.jointNameToId[motorName], desiredAngle)

  """Gets the base position of the minitaur."""
  def getBasePosition(self):
    position, orientation = p.getBasePositionAndOrientation(self.quadruped)
    return position

  """Gets the base orientation of the minitaur."""
  def getBaseOrientation(self):
    position, orientation = p.getBasePositionAndOrientation(self.quadruped)
    return orientation

  """Gets the motor angles."""
  def getMotorAngles(self):
    motorAngles = []
    for i in range(self.nMotors):
      jointState = p.getJointState(self.quadruped, self.motorIdList[i])
      motorAngles.append(jointState[POSITION_INDEX])
    motorAngles = np.multiply(motorAngles, self.motorDir)
    return motorAngles

  """Gets the motor velocites."""
  def getMotorVelocities(self):
    motorVelocities = []
    for i in range(self.nMotors):
      jointState = p.getJointState(self.quadruped, self.motorIdList[i])
      motorVelocities.append(jointState[VELOCITY_INDEX])
    motorVelocities = np.multiply(motorVelocities, self.motorDir)
    return motorVelocities

  """Gets the motor torques."""
  def getMotorTorques(self):
    motorTorques = []
    for i in range(self.nMotors):
      jointState = p.getJointState(self.quadruped, self.motorIdList[i])
      motorTorques.append(jointState[MOTOR_TORQUES_INDEX])
    motorTorques = np.multiply(motorTorques, self.motorDir)
    return motorTorques

  """Applys the motor commands."""
  def applyAction(self, motorCommands):
    motorCommandsWithDir = np.multiply(motorCommands, self.motorDir)
    for i in range(self.nMotors):
      self.setMotorAngleById(self.motorIdList[i], motorCommandsWithDir[i])

  """Resets the pose of the minitaur."""
  def resetPose(self):
    # constants
    kneeFrictionForce = 0
    halfpi = 1.57079632679
    kneeangle = -2.1834 #halfpi - acos(upper_leg_length / lower_leg_length)

    def resetLeftFrontLeg():
        # left front leg
        p.resetJointState(self.quadruped,self.jointNameToId['motor_front_leftL_joint'],self.motorDir[0]*halfpi)
        p.resetJointState(self.quadruped,self.jointNameToId['knee_front_leftL_link'],self.motorDir[0]*kneeangle)
        p.resetJointState(self.quadruped,self.jointNameToId['motor_front_leftR_joint'],self.motorDir[1]*halfpi)
        p.resetJointState(self.quadruped,self.jointNameToId['knee_front_leftR_link'],self.motorDir[1]*kneeangle)

        # add the constraints
        p.createConstraint(
            self.quadruped,self.jointNameToId['knee_front_leftR_link'],  # parent body
            self.quadruped,self.jointNameToId['knee_front_leftL_link'],  # child body
            p.JOINT_POINT2POINT, [0,0,0], [0,0.005,0.2], [0,0.01,0.2]) # joint axis, parentFramePosition, childFramePosition

        # set motors
        self.setMotorAngleByName('motor_front_leftL_joint', self.motorDir[0]*halfpi)
        self.setMotorAngleByName('motor_front_leftR_joint', self.motorDir[1]*halfpi)

        p.setJointMotorControl2(
            bodyIndex=self.quadruped,
            jointIndex=self.jointNameToId['knee_front_leftL_link'],
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=0, force=kneeFrictionForce)
        p.setJointMotorControl2(
            bodyIndex=self.quadruped,
            jointIndex=self.jointNameToId['knee_front_leftR_link'],
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=0, force=kneeFrictionForce)
    def resetLeftBackLeg():
        #left back leg
        p.resetJointState(self.quadruped,self.jointNameToId['motor_back_leftL_joint'],self.motorDir[2]*halfpi)
        p.resetJointState(self.quadruped,self.jointNameToId['knee_back_leftL_link'],self.motorDir[2]*kneeangle)
        p.resetJointState(self.quadruped,self.jointNameToId['motor_back_leftR_joint'],self.motorDir[3]*halfpi)
        p.resetJointState(self.quadruped,self.jointNameToId['knee_back_leftR_link'],self.motorDir[3]*kneeangle)
        p.createConstraint(self.quadruped,self.jointNameToId['knee_back_leftR_link'],self.quadruped,self.jointNameToId['knee_back_leftL_link'],p.JOINT_POINT2POINT,[0,0,0],[0,0.005,0.2],[0,0.01,0.2])
        self.setMotorAngleByName('motor_back_leftL_joint',self.motorDir[2]*halfpi)
        self.setMotorAngleByName('motor_back_leftR_joint',self.motorDir[3]*halfpi)
        p.setJointMotorControl2(bodyIndex=self.quadruped,jointIndex=self.jointNameToId['knee_back_leftL_link'],controlMode=p.VELOCITY_CONTROL,targetVelocity=0,force=kneeFrictionForce)
        p.setJointMotorControl2(bodyIndex=self.quadruped,jointIndex=self.jointNameToId['knee_back_leftR_link'],controlMode=p.VELOCITY_CONTROL,targetVelocity=0,force=kneeFrictionForce)
    def resetRightFrontLeg():
        #right front leg
        p.resetJointState(self.quadruped,self.jointNameToId['motor_front_rightL_joint'],self.motorDir[4]*halfpi)
        p.resetJointState(self.quadruped,self.jointNameToId['knee_front_rightL_link'],self.motorDir[4]*kneeangle)
        p.resetJointState(self.quadruped,self.jointNameToId['motor_front_rightR_joint'],self.motorDir[5]*halfpi)
        p.resetJointState(self.quadruped,self.jointNameToId['knee_front_rightR_link'],self.motorDir[5]*kneeangle)
        p.createConstraint(self.quadruped,self.jointNameToId['knee_front_rightR_link'],self.quadruped,self.jointNameToId['knee_front_rightL_link'],p.JOINT_POINT2POINT,[0,0,0],[0,0.005,0.2],[0,0.01,0.2])
        self.setMotorAngleByName('motor_front_rightL_joint',self.motorDir[4]*halfpi)
        self.setMotorAngleByName('motor_front_rightR_joint',self.motorDir[5]*halfpi)
        p.setJointMotorControl2(bodyIndex=self.quadruped,jointIndex=self.jointNameToId['knee_front_rightL_link'],controlMode=p.VELOCITY_CONTROL,targetVelocity=0,force=kneeFrictionForce)
        p.setJointMotorControl2(bodyIndex=self.quadruped,jointIndex=self.jointNameToId['knee_front_rightR_link'],controlMode=p.VELOCITY_CONTROL,targetVelocity=0,force=kneeFrictionForce)
    def resetRightBackLeg():
        #right back leg
        p.resetJointState(self.quadruped,self.jointNameToId['motor_back_rightL_joint'],self.motorDir[6]*halfpi)
        p.resetJointState(self.quadruped,self.jointNameToId['knee_back_rightL_link'],self.motorDir[6]*kneeangle)
        p.resetJointState(self.quadruped,self.jointNameToId['motor_back_rightR_joint'],self.motorDir[7]*halfpi)
        p.resetJointState(self.quadruped,self.jointNameToId['knee_back_rightR_link'],self.motorDir[7]*kneeangle)
        p.createConstraint(self.quadruped,self.jointNameToId['knee_back_rightR_link'],self.quadruped,self.jointNameToId['knee_back_rightL_link'],p.JOINT_POINT2POINT,[0,0,0],[0,0.005,0.2],[0,0.01,0.2])
        self.setMotorAngleByName('motor_back_rightL_joint',self.motorDir[6]*halfpi)
        self.setMotorAngleByName('motor_back_rightR_joint',self.motorDir[7]*halfpi)
        p.setJointMotorControl2(bodyIndex=self.quadruped,jointIndex=self.jointNameToId['knee_back_rightL_link'],controlMode=p.VELOCITY_CONTROL,targetVelocity=0,force=kneeFrictionForce)
        p.setJointMotorControl2(bodyIndex=self.quadruped,jointIndex=self.jointNameToId['knee_back_rightR_link'],controlMode=p.VELOCITY_CONTROL,targetVelocity=0,force=kneeFrictionForce)

    resetLeftFrontLeg()
    resetLeftBackLeg()
    resetRightFrontLeg()
    resetRightBackLeg()

def run_simulation():
    p.connect(p.GUI)
    p.setGravity(0,0,-10)

    p.loadURDF(os.path.join(MODELS_DIR, PLANE_URDF))
    minitaur = Minitaur(MODELS_DIR)

    t = 0.0

    while (True):
        t = t + fixedTimeStep
        target = math.sin(t*speed)*jump_amp+1.57
        minitaur.applyAction(np.ones(NUM_MOTORS)*target)

        p.stepSimulation()
        time.sleep(0.01)
