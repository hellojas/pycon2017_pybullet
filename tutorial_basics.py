"""Welcome to Google tech talk series for PyCon 2017!

    PyBullet: Physics Simulation for Robotics and Machine Learning
        Basic tutorial

Questions, comments? Tweet me @hellojas :)
"""
import pybullet as p
import os
import time

# all my constants
# importing all.. forgive me. :)
from data_config import *

def basics():
    p.connect(p.GUI) # see connection_types()

    basic_world_control()

    r2d2 = load_objects()

    basic_object_states(r2d2)

    # p.resetBasePositionAndOrientation(r2d2, [1,1,1], [0,0,0,1])
    # basic_object_states(r2d2)

    # basic_object_control(r2d2)

    # step(5000)

"""Explains basic world control."""
def basic_world_control():

    p.setGravity(0,0,-9.8)

    # p.stepSimulation()
    # or use p.setRealTimeSimulation(1)

    # p.resetSimulation()
    # p.disconnect()

"""Steps the simulation."""
def step(steps):
    for _ in xrange(steps):
        p.stepSimulation()
        time.sleep(.001)

"""Loads the objects for the scene."""
def load_objects():
    _ = p.loadURDF(os.path.join(MODELS_DIR, PLANE_URDF))
    r2d2 = p.loadURDF(os.path.join(MODELS_DIR, R2D2_URDF),0,0,0.5)
    return r2d2

"""Explains basic object states."""
def basic_object_states(r2d2):
    print('number of joints:', p.getNumJoints(r2d2))
    print('base position and orientation', p.getBasePositionAndOrientation(r2d2))
    for joint in p.getJointStates(r2d2, R2D2_WHEEL_JOINTS):
        print 'joint:', joint
    # (jointPosition, jointVelocity, jointReactionForces, appliedJointMotorTorque)

"""Explains basic object control."""
def basic_object_control(r2d2):
   p.setJointMotorControlArray(
    r2d2, R2D2_WHEEL_JOINTS, p.VELOCITY_CONTROL, # or p.POSITION_CONTROL or p.TORQUE_CONTROL
    targetVelocities=[1,1,1,1], forces=[1,1,1,1])

def connection_types():
    p.connect(p.DIRECT)  # non-graphical
    p.connect(p.GUI, options="--opengl2")  # using open gl2
    p.connect(p.SHARED_MEMORY, 1234)  # i.e used for VR
    p.connect(p.UDP, "192.168.0.1")  # connect over machines
    p.connect(p.UDP, "localhost", 1234)
    p.connect(p.TCP, "localhost", 6667)

def full_code():
    p.connect(p.GUI)
    p.setGravity(0,0,-9.8)
    _ = p.loadURDF(os.path.join(MODELS_DIR, PLANE_URDF))
    r2d2 = p.loadURDF(os.path.join(MODELS_DIR, R2D2_URDF),0,0,0.5)
    p.setJointMotorControlArray(r2d2, R2D2_WHEEL_JOINTS,
        p.VELOCITY_CONTROL, [1,1,1,1],[1,1,1,1])
    for ts in xrange(10000):
        print(ts)
        for i in xrange(p.getNumJoints(r2d2)):
            print(i, p.getJointState(r2d2, i))
        p.stepSimulation()
        time.sleep(.0001)
