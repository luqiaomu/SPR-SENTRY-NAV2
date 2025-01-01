#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import numpy as np
import math

import cv2
import yaml
import time
import os
from pathlib import Path
import sys

class CloudtoPgm(Node):
    def __init__(self):
        super().__init__('cloud_to_pgm')

        self.declare_parameter('lasermap_topic', '/Laser_map')             
        self.lasermap_topic = self.get_parameter(                     
            'lasermap_topic').get_parameter_value().string_value
        
        self.subscription = self.create_subscription(
            PointCloud2,
            self.lasermap_topic,
            self.pointcloud_callback,
            10)
        self.subscription  # 防止Python清理时取消订阅

        self.save_flag = False

        self.declare_parameter('parent_path', '/home/spr/Sentry_ws/src/SPR-RM-Sentry/src/rm_map/pointcloud_to_pgm/pgm/')             
        self.parent_path = self.get_parameter(                     
            'parent_path').get_parameter_value().string_value
        
        
        
        self.declare_parameter('senser_range_min', -0.2)             
        self.senser_range_min = self.get_parameter(                    
            'senser_range_min').get_parameter_value().double_value
        
        self.declare_parameter('senser_range_max', 0.1)             
        self.senser_range_max = self.get_parameter(                    
            'senser_range_max').get_parameter_value().double_value
        
        self.declare_parameter('down_sample_every_k_points', 20)             
        self.down_sample_every_k_points = self.get_parameter(                    
            'down_sample_every_k_points').get_parameter_value().integer_value
        
        self.declare_parameter('num_poins', 2)             
        self.num_poins = self.get_parameter(                    
            'num_poins').get_parameter_value().integer_value
        
        self.declare_parameter('radius', 0.05)             
        self.radius = self.get_parameter(                    
            'radius').get_parameter_value().double_value

    def pointcloud_callback(self, msg):
        if not self.save_flag:
            self.get_logger().info('Received a PointCloud2 message')

            pc_msg = msg

            pc = self.get_point_cloud(pc_msg)

            self.convert_to_pgm(pc) 

            self.save_flag = True
        else:
            self.destroy_node()
            rclpy.shutdown()
 

    def get_point_cloud(self, pc_msg):
        # Convert ROS PointCloud2 message to Open3D PointCloud
        points = np.frombuffer(pc_msg.data, dtype=np.float32).reshape(-1, pc_msg.point_step // 4)[:,:3]
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(points)
        return pc

    def convert_to_pgm(self, cloud):
        now = time.time()
        name = 'map' + str(int(now))
        pgm_path = self.parent_path + name + '.pgm'
        yaml_path = self.parent_path + name + '.yaml'

        
        downmap = cloud.uniform_down_sample(every_k_points=self.down_sample_every_k_points)

        points = np.asarray(downmap.points)
        filter_points = points[(points[:, 2] >= self.senser_range_min) & (points[:, 2] <= self.senser_range_max)]

        filter_cloud = o3d.geometry.PointCloud()
        filter_cloud.points = o3d.utility.Vector3dVector(filter_points)

        filter_cloud, ind = filter_cloud.remove_radius_outlier(self.num_poins, self.radius)

        self.get_logger().info(f"Point cloud size:{filter_cloud}")

        min_x = 0.0
        min_y = 0.0
        max_x = 0.0
        max_y = 0.0

        for point in filter_cloud.points:
            if point[0] > max_x:
                max_x = point[0]
            if point[0] < min_x:
                min_x = point[0]

            if point[1] > max_y:
                max_y = point[1]
            if point[1] < min_y:
                min_y = point[1]

        map_width = int((max_x - min_x) / 0.05)
        map_height = int((max_y - min_y) / 0.05)

        self.get_logger().info(f'Map size(pixel): {map_width}, {map_height}')

        origin_x = int(min_x / 0.05)
        origin_y = int(max_y / 0.05)

        pgm_map = np.ones((map_height, map_width, 3), np.uint8) * 254

        for point in filter_cloud.points:
            x = int((point[0] - min_x) / 0.05)
            y = int((max_y - point[1]) / 0.05)
            if y < map_height and x < map_width:
                pgm_map[y, x] = [0, 0, 0]

        # cv2.imshow('map', pgm_map)

        gray_image = cv2.cvtColor(pgm_map, cv2.COLOR_BGR2GRAY)
        cv2.imwrite(pgm_path, gray_image)

        map_yaml = {}
        map_yaml['free_thresh'] = 0.25
        map_yaml['image'] = name + '.pgm'
        map_yaml['mode'] = 'trinary'
        map_yaml['negate'] = 0
        map_yaml['occupied_thresh'] = 0.65
        map_yaml['origin'] = [float(round(min_x, 2)), float(round(min_y, 2)), 0]

        map_yaml['resolution'] = 0.05
        with open(yaml_path, 'w') as file:
            yaml.dump(map_yaml, file, default_flow_style=False, encoding='utf-8', allow_unicode=True)

        self.get_logger().info('Save map to: ' + yaml_path)
    
def main(args=None):
    rclpy.init(args=args)

    cloud_to_pgm_node = CloudtoPgm()

    rclpy.spin(cloud_to_pgm_node)
    cloud_to_pgm_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

