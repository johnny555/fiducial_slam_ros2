#!/usr/bin/env python3

import sys
import time
import unittest
import math
import rclpy
from rclpy.node import Node
from fiducial_msgs.msg import FiducialMapEntryArray
from geometry_msgs.msg import PoseWithCovarianceStamped

NAME = 'map_test_node'
EPSILON = 0.1

'''
A node to verify the map created by fiducial_slam. It will verify
that the specified map file contains at least min_lines lines,
and if the expect parameter is also given, it will check that
the position of the specfied fiducial. In addition, the pose
specified in expected_pose is verified
'''

def rad2deg(rad):
    return rad * 180.0 / math.pi

class MapTestNode(Node):
    def __init__(self):
        super().__init__(NAME)
        self.mapEntries = -1
        self.map = {}
        self.pose = None
        self.min_lines = self.declare_parameter('min_lines', 1).get_parameter_value().integer_value
        self.expect = self.declare_parameter('expect', '').get_parameter_value().string_value
        self.expected_pose = self.declare_parameter('expected_pose', '').get_parameter_value().string_value

        self.map_subscription = self.create_subscription(
            FiducialMapEntryArray,
            '/fiducial_map',
            self.mapCallback,
            10)
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/fiducial_pose',
            self.poseCallback,
            10)
        self.get_logger().info("map_test_node started")

    def mapCallback(self, msg):
        self.mapEntries = len(msg.fiducials)
        for fid in msg.fiducials:
            self.map[fid.fiducial_id] = (fid.x, fid.y, fid.z,
                                       rad2deg(fid.rx), rad2deg(fid.ry), rad2deg(fid.rz))
        # self.get_logger().info(f'Received map with {self.mapEntries} entries')

    def poseCallback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.pose = (position.x, position.y, position.z,
                     orientation.x, orientation.y, orientation.z, orientation.w)
        # self.get_logger().info(f'Received pose: {self.pose}')

class TestMap(unittest.TestCase):
    def test_map_and_pose(self):
        rclpy.init()
        node = MapTestNode()
        
        self.get_logger().info(f"min_lines: {node.min_lines}")
        self.get_logger().info(f"expect: {node.expect}")
        self.get_logger().info(f"expected_pose: {node.expected_pose}")

        start_time = time.time()
        timeout_seconds = 60  # Timeout after 60 seconds

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.mapEntries >= node.min_lines and node.pose is not None:
                if node.expected_pose != "":
                    ep = node.expected_pose.split()
                    self.assertEqual(len(ep), len(node.pose), 
                                     f"Expected pose has {len(ep)} elements, got {len(node.pose)}")
                    for i in range(len(ep)):
                        self.assertAlmostEqual(float(ep[i]), float(node.pose[i]), delta=EPSILON,
                                             msg=f"Pose mismatch at index {i}: expected {ep[i]}, got {node.pose[i]}. Full pose: {node.pose}")
                
                if node.expect != "":
                    lines = node.expect.split(',')
                    for line in lines:
                        ex = line.split()
                        fid = int(ex[0])
                        ex_values = [float(v) for v in ex[1:]]
                        
                        self.assertIn(fid, node.map, f"Fiducial {fid} not in map. Map keys: {list(node.map.keys())}")
                        fiducial_data = node.map[fid]
                        self.assertEqual(len(ex_values), len(fiducial_data),
                                         f"Fiducial {fid} expected data has {len(ex_values)} elements, got {len(fiducial_data)}")
                        for i in range(len(ex_values)):
                            self.assertAlmostEqual(ex_values[i], float(fiducial_data[i]), delta=EPSILON,
                                                 msg=f"Fiducial {fid} data mismatch at index {i}: expected {ex_values[i]}, got {fiducial_data[i]}. Full data: {fiducial_data}")
                node.destroy_node()
                rclpy.shutdown()
                return # Test passed

            if time.time() - start_time > timeout_seconds:
                node.destroy_node()
                rclpy.shutdown()
                self.fail(f"Timeout after {timeout_seconds} seconds. mapEntries: {node.mapEntries}, pose: {node.pose}")
            
            time.sleep(0.5)
        
        # Should not be reached if test passes or times out correctly
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        self.fail("Test loop exited unexpectedly.")
