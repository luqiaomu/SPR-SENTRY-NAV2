# 0,205,254
import open3d as o3d
import copy
import numpy as np
import cv2
import yaml
import time
import os
from pathlib import Path
import sys

FILE = Path(__file__).resolve()
ROOT = FILE.parents[1]
ROOT = str(ROOT)

now = time.time()
name = 'map' + str(int(now))
pgm_path = ROOT + '/pgm/' + name + '.pgm'
yaml_path = ROOT + '/pgm/' + name + '.yaml'

pcd_path = '/home/spr/Sentry_ws/src/SPR-RM-Sentry/src/rm_localization/fast_lio/PCD/result.pcd'
pcd_cloud = o3d.io.read_point_cloud(pcd_path)
downmap = pcd_cloud.uniform_down_sample(every_k_points=20)

mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=3.0,origin=[0,0,0])

points = np.asarray(downmap.points)
filter_points = points[(points[:, 2] >= -0.1) & (points[:, 2] <= 0.1)]

filter_cloud = o3d.geometry.PointCloud()
filter_cloud.points = o3d.utility.Vector3dVector(filter_points)

num_poins = 2
radius = 0.05

filter_cloud, ind = filter_cloud.remove_radius_outlier(num_poins, radius)

print(f"Point cloud size:{filter_cloud}")

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

print(f'Map size(pixel): {map_width}, {map_height}')

origin_x = int(min_x / 0.05)
origin_y = int(max_y / 0.05)

pgm_map = np.ones((map_height, map_width, 3), np.uint8) * 254

for point in filter_cloud.points:
    x = int((point[0] - min_x) / 0.05)
    y = int((max_y - point[1]) / 0.05)
    if y < map_height and x < map_width:
        pgm_map[y, x] = [0, 0, 0]

cv2.imshow('map', pgm_map)

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

print('Save map to: ' + yaml_path)

# o3d.visualization.draw_geometries([filter_cloud, mesh_frame])

cv2.waitKey(0)
