import argparse
import os
import yaml
import rclpy
import numpy as np

from rclpy.node import Node, Publisher

def parse_arguments(args):
    parser = argparse.ArgumentParser()

    parser.add_argument("--robot_name")
    parser.add_argument("--configuration_file_path")

    known_args, _ = parser.parse_known_args(args)

    return known_args
    

def set_robot_environment(robot_name):
    os.environ["WEBOTS_ROBOT_NAME"] = robot_name


def read_configuration_file(filepath):
    with open(filepath, "r") as file:
        robot_config = yaml.safe_load(file)

        name = robot_config["name"]
        devices = robot_config["devices"]

        return name, devices


class Limit:
    def __init__(self, min, max):
        self.min = min
        self.max = max

        self.real_min = min
        self.real_max = max

    def is_in_range(self, value):
        if value > self.max or value < self.min:
            print(f"[WARNING]\tValue {value} is not in range [{self.min}, {self.max}]")

            return False

        return True

    def is_in_real_range(self, value):
        return value <= self.real_max and value >= self.real_min

    def norm_value_to_range(self, value):
        return Limit.norm_value(value, self.min, self.max)

    def modulate_value_to_range(self, value):
        is_negative = value < 0

        return (abs(value) % self.max) * (-1 if is_negative else 1)

    def norm_value_to_real_range(self, value):
        return Limit.norm_value(value, self.real_min, self.real_max)

    def set_new_limit(self, new_limits):
        """
            Sets the upper and lower bound of the limit
        """
        self.min = self.norm_value_to_real_range(new_limits[0])
        self.max = self.norm_value_to_real_range(new_limits[1])

    @staticmethod
    def norm_value(value, min, max):
        return np.minimum(max, np.maximum(min, value))