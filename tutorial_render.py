"""Welcome to Google tech talk series for PyCon 2017!

    PyBullet: Physics Simulation for Robotics and Machine Learning
        Rendering tutorial

Questions, comments? Tweet me @hellojas :)
"""
from data_config import *  # Importing all.. forgive me. :)
from camera_config import *

import matplotlib.pyplot as plt
import numpy as np
import os
import pybullet as p
import time

# all my constants
from data_config import *  # Importing all.. forgive me. :)
from camera_config import *

"""Runs a simulation and either render an image or view the GUI."""
def run_simulation(render=None):
    if render:
        p.connect(p.DIRECT)
    else:
        p.connect(p.GUI)

    if render == 'simplified':
        setup_scene()
        render_simple()
    elif render == 'advance':
        setup_scene()
        render_advance()
    else:
        setup_scene(plane=True)
        debug_with_gui()

    p.disconnect()

"""Views the simulation with a GUI."""
def debug_with_gui():
    p.setGravity(0, 0, -9.8)

    for _ in xrange(500):
        p.stepSimulation()
        time.sleep(0.01)

"""Loads the URDFs into the simulation."""
def setup_scene(plane=False):
    if plane:
        p.loadURDF(os.path.join(MODELS_DIR, PLANE_URDF), 0,0,-.2)
    p.loadURDF(os.path.join(MODELS_DIR, R2D2_URDF), 0,0,.2)

"""Renders an image of the simulation with minimal parameters."""
def render_simple():
    img_obj = p.getCameraImage(pixelWidth, pixelHeight) # (w, h, rgb, depth, segmentation)
    w, h, rgb, _, _ = img_obj
    # preprocess for matplotlib
    np_img_arr = np.reshape(rgb, (h, w, 4))
    np_img_arr = np_img_arr / 255.
    plt.imshow(np_img_arr)
    plt.show()

"""Renders an image of the simulation with custom view and projection matrices."""
def render_advance():
    main_start = time.time()
    for pitch in range (90,180,10) :
        start = time.time()
        viewMatrix = p.computeViewMatrixFromYawPitchRoll(camTargetPos, camDistance, yaw, pitch, roll, upAxisIndex)
        aspect = pixelWidth / pixelHeight;
        projectionMatrix = p.computeProjectionMatrixFOV(fov, aspect, nearPlane, farPlane);
        img_arr = p.getCameraImage(pixelWidth, pixelHeight,
            viewMatrix, projectionMatrix,
            lightDirection=[0,1,0],renderer=p.ER_TINY_RENDERER)
        stop = time.time()
        print ("renderImage %f" % (stop - start))

        w, h, rgb, _, _ = img_arr
        print ('width = %d height = %d' % (w,h))

        np_img_arr = np.reshape(rgb, (h, w, 4))
        np_img_arr = np_img_arr / 255.

        plt.imshow(np_img_arr,interpolation='none')
        # plt.show()
        plt.pause(0.001)
