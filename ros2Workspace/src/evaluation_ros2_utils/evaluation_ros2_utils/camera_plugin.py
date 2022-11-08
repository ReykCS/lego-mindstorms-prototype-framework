import os
import rclpy
import sys
from rclpy.node import Node

from std_msgs.msg import Empty

from datetime import datetime

import numpy as np

NOW = datetime.now().strftime("%m_%d__%H_%M_%S")
DIR = os.environ.get("LM_FRAMEWORK_PATH") + "recording/simulation/"

EVAL_SCENARIO = "distance_sensor"
ROBOT_NAME = "Eval_DistanceSensor"

DIR += ROBOT_NAME + "_" + EVAL_SCENARIO + "_" + NOW

RECORDING = True

class CameraPlugin():
    def init(self, webots_node, properties={}):
        try: 
            rclpy.init(args=None)
        except:
            pass

        self.node = Node("CameraNode")

        self.robot = webots_node.robot
        self.device = self.robot.getDevice("top_camera")
        self.timestep = int(self.robot.getBasicTimeStep())

        self.device.enable(33)

        self.current_timestep = 0

        self.node.create_subscription(
            Empty, "/eval/has_finished", self.has_finished_callback, 10
        )

        self.is_recording = RECORDING

        # Create directory for recordings
        os.mkdir(DIR)

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)

        if not self.is_recording:
            return

        # image = self.device.getImageArray()
        self.device.saveImage(DIR + "/" + str(self.current_timestep) + ".png", 90)

        print("Image", self.current_timestep) # , image[:1])

        self.current_timestep += self.timestep

    def has_finished_callback(self, _):
        # Turn off camera
        self.is_recording = False

        print("Stop recording")
