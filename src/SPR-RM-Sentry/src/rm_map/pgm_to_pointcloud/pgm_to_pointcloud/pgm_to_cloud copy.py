import cv2
import numpy as np
import open3d as o3d
import yaml

# 读取PGM格式的代价地图
costmap = cv2.imread('/home/spr/Sentry_ws/src/SPR-RM-Sentry/src/rm_map/pgm_to_pointcloud/pgm/RMUC.pgm', cv2.IMREAD_GRAYSCALE)

with open('/home/spr/Sentry_ws/src/SPR-RM-Sentry/src/rm_map/pgm_to_pointcloud/pgm/RMUC.yaml', 'r') as file:
    data = file.read()
    map_yaml = yaml.load(data, Loader=yaml.FullLoader)

step = 1
height, width = costmap.shape
# print(height,width)
point_cloud = o3d.geometry.PointCloud()


for i in range(0, height, step):
    for j in range(width):
        if costmap[i, j] == 0:  # 假设非零值代表障碍物
            # 计算点云中每个点的坐标
            real_x = j * 0.05 + map_yaml['origin'][0]
            k = height - i
            real_y = k * 0.05 + map_yaml['origin'][1]
            point_cloud.points.append([real_x, real_y, 0])
            # point_cloud.points.append([real_x-0.025, real_y, 0])
            # point_cloud.points.append([real_x, real_y-0.025, 0])
            # point_cloud.points.append([real_x+0.025, real_y, 0])
            # point_cloud.points.append([real_x, real_y+0.025, 0])

            # point_cloud.points.append([real_x-0.025, real_y+0.025, 0])
            # point_cloud.points.append([real_x+0.025, real_y-0.025, 0])
            # point_cloud.points.append([real_x+0.025, real_y+0.025, 0])
            # point_cloud.points.append([real_x-0.025, real_y-0.025, 0])

mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=2.0,origin=[0,0,0])
print(point_cloud)
o3d.io.write_point_cloud("/home/spr/Sentry_ws/src/SPR-RM-Sentry/src/rm_map/pgm_to_pointcloud/pcd/write.pcd", point_cloud)

o3d.visualization.draw_geometries([point_cloud, mesh_frame])
