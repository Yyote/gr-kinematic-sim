import pygame as pg
import pygame.draw as d
import math as m
import numpy as np
import os
import rclpy
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header
from gr_kinematic_sim.custom_utils.object_tools import Sprite, SMALL_IMAGE_SIZE, DEFAULT_IMAGE_SIZE, VERY_SMALL_IMAGE_SIZE, WORLD_SCALE
from gr_kinematic_sim.custom_utils.collisions import find_lidar_collisions
from ament_index_python.packages import get_package_share_directory

pkg_dir = f"{get_package_share_directory('gr_kinematic_sim')}/../../../../src/gr_kinematic_sim/"

class Lidar(Sprite):
    def __init__(self, x, y, image, screen, offset_x, offset_y, num_rays, ray_length_px, robot_name, node, enable_pointcloud):
        super().__init__(x, y, image, screen, offset_x, offset_y, VERY_SMALL_IMAGE_SIZE)
        self.curr_offset_x = offset_x
        self.curr_offset_y = offset_y
        self.num_rays = num_rays
        self.ray_length_px = ray_length_px
        self.node = node
        self.lidar_pub = node.create_publisher(LaserScan, f"{robot_name}/scan", 10)
        self.pointcloud_pub = node.create_publisher(PointCloud2, f"{robot_name}/pointcloud", 10)
        self.robot_name = robot_name
        self.enable_pointcloud = enable_pointcloud
        
        self.angle_min = - m.pi
        self.angle_max = m.pi
        self.angle_increment = m.pi / 180.0 * (360 / self.num_rays)
        self.range_min = 0.0
        self.range_max = ray_length_px / WORLD_SCALE
    
    def _draw_line(self, screen, x1, y1, x2, y2):
        d.aaline(screen, (0, 0, 255 ), (x1 + self.curr_offset_x, y1 + self.curr_offset_y), (x2 + self.curr_offset_x, y2 + self.curr_offset_y), 1)
        return x1, y1, x2, y2
    
    def get_lidar_lines_around_point(self, screen, draw_lines=False):
        lines = []
        x0 = self.rect.x + self.rect.width / 2
        y0 = self.rect.y + self.rect.height / 2
        current_orientation = (self._current_rotation - 90) * m.pi / 180
        for i in range(self.num_rays):
            angle = i * m.pi / 180.0 * (360 / self.num_rays)
            x2 = m.cos(angle - current_orientation) * self.ray_length_px + x0
            y2 = m.sin(angle - current_orientation) * self.ray_length_px + y0
            if draw_lines:
                self._draw_line(screen, x0, y0, x2, y2)
                if i == 0:
                    pg.draw.line(screen, (255, 255, 0), (x0 + self.curr_offset_x, y0 + self.curr_offset_y), (x2 + self.curr_offset_x, y2 + self.curr_offset_y))
            lines.append([(x0, y0), (x2, y2)])
        
        return lines

    def logic(self, tilemap, spritelist, name):
        lines = self.get_lidar_lines_around_point(self.screen, False)
        collisions = find_lidar_collisions(tilemap, spritelist, lines, name, self.range_max * WORLD_SCALE)
        map_collisions = collisions
        msg = LaserScan()
        msg.angle_increment = self.angle_increment
        msg.angle_max = self.angle_max
        msg.angle_min = self.angle_min
        msg.range_max = self.range_max
        msg.range_min = self.range_min
        msg.scan_time = 0.0
        msg.time_increment = msg.scan_time / self.num_rays
        
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = f"{self.robot_name}/laser"

        for i in range(len(collisions)):
            if collisions[-i] is not None:
                x1 = self.rect.centerx
                y1 = self.rect.centery
                
                x2 = collisions[-i][0]
                y2 = collisions[-i][1]
                
                if not m.isinf(x2):
                    dr = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5 
                    msg.ranges.append(dr / WORLD_SCALE)
                    # pg.draw.circle(self.screen, (255, 0, 0), map_collisions[-i], 3, 1)
                if m.isinf(x2):
                    msg.ranges.append(float('inf'))
                    x2 = m.cos(-i * msg.angle_increment - (self._current_rotation * m.pi / 180) + m.pi / 2) * msg.range_max * WORLD_SCALE + self.rect.centerx
                    y2 = m.sin(-i * msg.angle_increment - (self._current_rotation * m.pi / 180) + m.pi / 2) * msg.range_max * WORLD_SCALE + self.rect.centery
                    map_collisions[-i] = (x2, y2)
        
        tilemap.unfog_map(map_collisions, self.rect.center, msg.range_max * WORLD_SCALE)

        if self.enable_pointcloud:
            ros_dtype = PointField.FLOAT32
            dtype = np.float32
            itemsize = np.dtype(dtype).itemsize # A 32-bit float takes 4 bytes.

            points = []

            for i in range(len(msg.ranges)):
                x = msg.ranges[i] * m.cos(i * msg.angle_increment + m.pi)
                y = msg.ranges[i] * m.sin(i * msg.angle_increment + m.pi)
                z = 0
                points.append([x, y, z])
            
            points = np.array(points)
            data = points.astype(dtype).tobytes()

            fields = [PointField(
                name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
                for i, n in enumerate('xyz')]

            pc = PointCloud2()
            pc.is_dense = False
            pc.is_bigendian = False

            header = Header()
            header.stamp = self.node.get_clock().now().to_msg()
            header.frame_id = f"{self.robot_name}/laser"
            pc.header = header

            pc.fields = fields
            pc.height = 1
            pc.width = points.shape[0]
            pc.point_step = (itemsize * 3)
            pc.row_step = (itemsize * 3 * points.shape[0])
            pc.data = data

            self.pointcloud_pub.publish(pc)

        self.lidar_pub.publish(msg)
        return collisions
    
    def set_center_position(self, center_x, center_y, rotation_deg):
        self.rect.centerx = center_x
        self.rect.centery = center_y 
        self.set_rotation(rotation_deg)



class LidarB1(Lidar):
    def __init__(self, robot_name, screen, node, enable_pointcloud=False):
        super().__init__(200, 200, pg.image.load(f'{pkg_dir}gr_kinematic_sim/sprites/LidarBig.png'), screen, 0, 0, 40, 180, robot_name, node, enable_pointcloud=enable_pointcloud)


