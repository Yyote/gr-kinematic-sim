import pygame as pg
import pytmx
import math
import rclpy
from copy import copy

from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, TwistStamped, Twist, PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Odometry
from tf2_ros.transform_broadcaster import TransformBroadcaster, TransformStamped

from gr_kinematic_sim.custom_utils.object_tools import PhysicalObject, DEFAULT_IMAGE_SIZE, SMALL_IMAGE_SIZE, VERY_SMALL_IMAGE_SIZE, pkg_dir, WORLD_SCALE
from gr_kinematic_sim.custom_utils.mathtools import normalise_in_range, sgn_wo_zero, rotation_matrix, EulerAngles
from gr_kinematic_sim.custom_utils.collisions import check_kinematic_collision_between_tilemap_and_rect
from gr_kinematic_sim.custom_utils.gametools import tick_rate
from gr_kinematic_sim.custom_utils.sensors import LidarB1





class Robot(PhysicalObject):
    def __init__(self, node, name, tilemap, robot_factory, x, y, image, screen, offset_x, offset_y, mass=1, friction_multiplier=0.95, image_size=DEFAULT_IMAGE_SIZE, dynamic_model=True):
        super().__init__(name, tilemap, robot_factory, x, y, image, screen, offset_x, offset_y, mass, friction_multiplier, image_size=image_size, dynamic_model=dynamic_model)
        self.sensors = None
        self.node = node
        # self.node = rclpy.node.Node()
        self.pose_pub = self.node.create_publisher(PoseStamped, f"{name}/pose", 10)
        self.odom_pub = self.node.create_publisher(Odometry, f"{name}/odom", 10)
        self.transform_broadcaster = TransformBroadcaster(self.node)
        self.previous_pose = None
        
        self.lidar_transform = TransformStamped()

        self.lidar_transform.header.stamp = self.node.get_clock().now().to_msg()
        self.lidar_transform.header.frame_id = f"{self.name}/base_link"
        self.lidar_transform.child_frame_id = f"{self.name}/laser"
        
        self.lidar_transform.transform.translation.x = 0.0
        self.lidar_transform.transform.translation.y = 0.0
        self.lidar_transform.transform.translation.z = 0.0
        
        self.lidar_transform.transform.rotation.x = 0.0
        self.lidar_transform.transform.rotation.y = 0.0
        self.lidar_transform.transform.rotation.z = 0.0
        self.lidar_transform.transform.rotation.w = 1.0
        self.transform_broadcaster.sendTransform(self.lidar_transform)

    
    # def cmd_vel_cb(self, msg):
    #     # msg = TwistStamped()
    #     # msg.twist = Twist()
    #     self.set_local_velocity(msg.twist.linear.x * WORLD_SCALE / tick_rate, msg.twist.linear.y * WORLD_SCALE / tick_rate, msg.twist.angular.z * WORLD_SCALE / tick_rate)
    
    def call_sensors(self):
        for sensor in self.sensors:
            sensor.update_offset(self.curr_offset_x, self.curr_offset_y)
            sensor.set_center_position(self.rect.x + self.rect.width / 2, self.rect.y + self.rect.height / 2, self._current_rotation)
            sensor.draw()
            sensor.logic(self.tilemap)
    
    def draw(self, offset_x, offset_y):
        super().draw(offset_x, offset_y)
        self.call_sensors()
        
        transform = TransformStamped()
        pose_msg = PoseStamped()
        odom_msg = Odometry()

        pose_msg.header.stamp = self.node.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"

        odom_msg.header = pose_msg.header
        odom_msg.child_frame_id = f"{self.name}/laser"


        pose_msg.pose.position.x = (- self.rect.centery) / WORLD_SCALE
        pose_msg.pose.position.y = (- self.rect.centerx) / WORLD_SCALE
        pose_msg.pose.position.z = 0.0

        angles = EulerAngles()
        orientation = copy(self._current_rotation)
        if orientation > 180:
            orientation - 360
        print(f"self._current_rotation: {orientation}")
        pose_msg.pose.orientation = angles.setRPY_of_quaternion(0, 0, orientation * math.pi / 180)
        self.pose_pub.publish(pose_msg)
        
        transform.header.stamp = self.node.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = f"{self.name}/base_link"

        transform.transform.translation.x = pose_msg.pose.position.x
        transform.transform.translation.y = pose_msg.pose.position.y
        transform.transform.translation.z = pose_msg.pose.position.z

        q = angles.setRPY_of_quaternion(0, 0, self._current_rotation * math.pi / 180)
        transform.transform.rotation.x = q.x
        transform.transform.rotation.y = q.y
        transform.transform.rotation.z = q.z
        transform.transform.rotation.w = q.w
        
        self.transform_broadcaster.sendTransform(transform)
        self.lidar_transform.header.stamp = self.node.get_clock().now().to_msg()
        self.transform_broadcaster.sendTransform(self.lidar_transform)
        
        odom_msg.pose.pose = pose_msg.pose
        if self.previous_pose != None:
            t0 = (self.previous_pose.header.stamp.sec, self.previous_pose.header.stamp.nanosec)
            t1 = (pose_msg.header.stamp.sec, pose_msg.header.stamp.nanosec)
            
            t0 = float(t0[0]) + float(t0[1] / (10 ** 9))
            t1 = float(t1[0]) + float(t1[1] / (10 ** 9))
            
            dt = t1 - t0 
            twist_wc = TwistWithCovariance()
            twist = Twist()
            twist.linear.x = (pose_msg.pose.position.x - self.previous_pose.pose.position.x) / dt
            twist.linear.y = (pose_msg.pose.position.y - self.previous_pose.pose.position.y) / dt
            twist.linear.z = (pose_msg.pose.position.z - self.previous_pose.pose.position.z) / dt
            
            angles.getRPY_from_quaternion(self.previous_pose.pose.orientation)
            
            curr_rotation = self._current_rotation * math.pi / 180.0
            if curr_rotation > math.pi:
                curr_rotation -= 2 * math.pi
            twist.angular.z = (curr_rotation - angles._yaw) / dt
            
            curr_rotation = self._current_rotation * math.pi / 180.0
            if curr_rotation > math.pi:
                curr_rotation -= 2 * math.pi
            twist.angular.z = (curr_rotation - angles._yaw) / dt
            
            twist_wc.twist = twist
            odom_msg.twist = twist_wc
            self.odom_pub.publish(odom_msg)
        self.previous_pose = pose_msg
        # self.lin_vel_x = 0.0
        # self.lin_vel_y = 0.0
        self.ang_vel = 0.0

    
    def set_sensors(self, sensors=[]):
        if type(sensors) != type([]):
            print('TypeError: `sensors` must be an array!')
            raise TypeError('`sensors` must be an array!')
            quit()
        self.sensors = sensors


class AckermanRobot(Robot):
    def __init__(self, node, name, tilemap, robot_factory, x, y, image, screen, offset_x, offset_y, mass=1, friction_multiplier=0.95, image_size=DEFAULT_IMAGE_SIZE):
        super().__init__(node, name, tilemap, robot_factory, x, y, image, screen, offset_x, offset_y, mass, friction_multiplier, image_size=image_size, dynamic_model=False)
        self.cmd_vel_sub = self.node.create_subscription(TwistStamped, f"{name}/cmd_vel", self.cmd_vel_cb, 10)
        self.linear_max = 1.0
        self.min_rotation_radius = self.linear_max ** 2 / (self.mass * 9.81 * self.friction_multiplier)
    
    def cmd_vel_cb(self, msg):
        # msg = TwistStamped()
        # msg.twist = Twist()
        if abs(msg.twist.linear.x) > self.linear_max:
            msg.twist.linear.x = self.linear_max * sgn_wo_zero(msg.twist.linear.x)
        
        acv = msg.twist.linear.x ** 2 / self.min_rotation_radius
        acw = msg.twist.angular.z ** 2 * self.min_rotation_radius
        
        if acw > acv:
            msg.twist.angular.z = acv / self.min_rotation_radius
        
        self.set_local_velocity(msg.twist.linear.x * WORLD_SCALE / tick_rate, (msg.twist.angular.z / tick_rate) * 180 / math.pi)
    
    def set_local_velocity(self, vel, ang_vel):
        self.lin_vel_x = vel * math.cos(-self._current_rotation * math.pi / 180 - math.pi / 2)
        self.lin_vel_y = vel * math.sin(-self._current_rotation * math.pi / 180 - math.pi / 2)
        self.ang_vel = ang_vel
        if abs(vel) -0.2 < 0:
            self.ang_vel = 0
    
    def apply_force_now(self, lin_force_x, lin_force_y, ang_force=0):
        if self.dynamic_model == False:
            raise Exception('Dynamic modeling is turned off for this model, so this function should not be called. Please, check your code')
            quit()
        
    def apply_force_now_local(self, lin_force_x, lin_force_y, ang_force=0):
        if self.dynamic_model == False:
            raise Exception('Dynamic modeling is turned off for this model, so this function should not be called. Please, check your code')
            quit()




class OmniRobot(Robot):
    def __init__(self, node, name, tilemap, robot_factory, x, y, image, screen, offset_x, offset_y, mass=1, friction_multiplier=0.95, image_size=DEFAULT_IMAGE_SIZE):
        super().__init__(node, name, tilemap, robot_factory, x, y, image, screen, offset_x, offset_y, mass, friction_multiplier, image_size=image_size, dynamic_model=False)
        self.cmd_vel_sub = self.node.create_subscription(TwistStamped, f"{name}/cmd_vel", self.cmd_vel_cb, 10)
        self.linear_max = 1
    
    def cmd_vel_cb(self, msg):
        # msg = TwistStamped()
        # msg.twist = Twist()
        vx = -msg.twist.linear.y 
        vy = -msg.twist.linear.x 

        if ((vx ** 2 + vy ** 2) > self.linear_max ** 2):
            alpha = math.atan2(vy, vx)
            vx = math.cos(alpha) * self.linear_max
            vy = math.sin(alpha) * self.linear_max
            
        print(f"self.linear_max: {self.linear_max}")
        print(f"vx: {vx}")
        print(f"vy: {vy}")
        print(f"vx ** 2 + vy ** 2: {vx ** 2 + vy ** 2}")

        self.set_local_velocity(vx * WORLD_SCALE / tick_rate, vy * WORLD_SCALE / tick_rate, (msg.twist.angular.z / tick_rate) * 180 / math.pi)
    
    def apply_force_now(self, lin_force_x, lin_force_y, ang_force=0):
        if self.dynamic_model == False:
            raise Exception('Dynamic modeling is turned off for this model, so this function should not be called. Please, check your code')
            quit()
        
    def apply_force_now_local(self, lin_force_x, lin_force_y, ang_force=0):
        if self.dynamic_model == False:
            raise Exception('Dynamic modeling is turned off for this model, so this function should not be called. Please, check your code')
            quit()




class TrackedRobot(Robot):
    def __init__(self, node, name, tilemap, robot_factory, x, y, image, screen, offset_x, offset_y, mass=1, friction_multiplier=0.95, image_size=DEFAULT_IMAGE_SIZE):
        super().__init__(node, name, tilemap, robot_factory, x, y, image, screen, offset_x, offset_y, mass, friction_multiplier, image_size=image_size, dynamic_model=False)
        self.cmd_vel_sub = self.node.create_subscription(TwistStamped, f"{name}/cmd_vel", self.cmd_vel_cb, 10)
        self.linear_max = 1.0
        self.ang_max = math.pi 

    def cmd_vel_cb(self, msg):
        if (msg != Twist()):
            if abs(msg.twist.linear.x) > self.linear_max:
                msg.twist.linear.x = self.linear_max * sgn_wo_zero(msg.twist.linear.x)

            if abs(msg.twist.angular.z) > self.ang_max:
                msg.twist.angular.z = self.ang_max * sgn_wo_zero(msg.twist.angular.z)
            print(msg)
            self.set_local_velocity(msg.twist.linear.x * WORLD_SCALE / tick_rate, (msg.twist.angular.z / tick_rate) * 180 / math.pi)
    
    def set_local_velocity(self, vel, ang_vel):
        self.lin_vel_x = vel * math.cos(-self._current_rotation * math.pi / 180 - math.pi / 2)
        self.lin_vel_y = vel * math.sin(-self._current_rotation * math.pi / 180 - math.pi / 2)
        
        self.ang_vel = ang_vel
        # if abs(vel) -0.2 < 0:
        #     self.ang_vel = 0
    
    def apply_force_now(self, lin_force_x, lin_force_y, ang_force=0):
        if self.dynamic_model == False:
            raise Exception('Dynamic modeling is turned off for this model, so this function should not be called. Please, check your code')
            quit()
        
    def apply_force_now_local(self, lin_force_x, lin_force_y, ang_force=0):
        if self.dynamic_model == False:
            raise Exception('Dynamic modeling is turned off for this model, so this function should not be called. Please, check your code')
            quit()







class RobotFactory:
    def __init__(self, node, tilemap, screen, default_robot_name="/robot"):
        self.node = node
        self.tilemap = tilemap
        self.default_robot_name = default_robot_name
        self.screen = screen
        self.robot_counter = 1
        self.spritelist = []
        
    def create_ackerman_with_lidar(self, pos_x, pos_y):
        sensors = [LidarB1(f"robot{self.robot_counter}", self.screen, self.node)]
        robot = AckermanRobot(self.node, f"robot{self.robot_counter}", self.tilemap, self,  pos_x, pos_y, pg.image.load(f'{pkg_dir}gr_kinematic_sim/sprites/robots/wheeled2.png'), self.screen, 0, 0, 1, 0.9, (25, 25))
        robot.set_sensors(sensors)
        self.spritelist.append(robot)
        if len(self.spritelist) > 1:
            for i in range(len(self.spritelist)):
                self.spritelist[self.robot_counter - 1].robot_factory.spritelist.append(robot)
        self.robot_counter += 1
        return robot
        
    def create_omni_with_lidar(self, pos_x, pos_y):
        sensors = [LidarB1(f"robot{self.robot_counter}", self.screen, self.node)]
        robot = OmniRobot(self.node, f"robot{self.robot_counter}", self.tilemap, self,  pos_x, pos_y, pg.image.load(f'{pkg_dir}gr_kinematic_sim/sprites/robots/omniwheeled.png'), self.screen, 0, 0, 1, 0.9, (25, 25))
        robot.set_sensors(sensors)
        self.spritelist.append(robot)
        if len(self.spritelist) > 1:
            for i in range(len(self.spritelist)):
                self.spritelist[self.robot_counter - 1].robot_factory.spritelist.append(robot)
        self.robot_counter += 1
        return robot
    
    def create_tracked_with_lidar(self, pos_x, pos_y):
        sensors = [LidarB1(f"robot{self.robot_counter}", self.screen, self.node)]
        robot = TrackedRobot(self.node, f"robot{self.robot_counter}", self.tilemap, self,  pos_x, pos_y, pg.image.load(f'{pkg_dir}gr_kinematic_sim/sprites/robots/tracked.png'), self.screen, 0, 0, 1, 0.9, (25, 25))
        robot.set_sensors(sensors)
        self.spritelist.append(robot)
        if len(self.spritelist) > 1:
            for i in range(len(self.spritelist)):
                self.spritelist[self.robot_counter - 1].robot_factory.spritelist.append(robot)
        self.robot_counter += 1
        return robot