import pygame as pg
import rclpy
import os

from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist

from ament_index_python.packages import get_package_share_directory
from gr_kinematic_sim.custom_utils.sensors import Lidar, LidarB1
from gr_kinematic_sim.custom_utils.collisions import check_dynamic_collisions_between_tilemap_and_spritelist, check_collisions_in_spritelist, check_collisions_between_tilemap_and_lines
from gr_kinematic_sim.custom_utils.object_tools import WORLD_SCALE, Sprite, FoggedMap, PhysicalObject
from gr_kinematic_sim.custom_utils.robots import Robot, AckermanRobot, RobotFactory
from gr_kinematic_sim.custom_utils.rostools import try_get

from gr_kinematic_sim.custom_utils.gametools import handle_key_events, handle_offset_change, handle_keypresses_through_force, move_every_sprite_in_list, move_every_sprite_in_list, draw_every_sprite_in_list, scroll_screen_with_mouse, handle_keypresses_through_velocity, handle_keypresses_through_velocity_omni
from gr_kinematic_sim.custom_utils.gametools import tick_rate

import faulthandler
faulthandler.enable()

pkg_dir = f"{get_package_share_directory('gr_kinematic_sim')}/../../../../src/gr_kinematic_sim/"


class SimNode(Node):
    def __init__(self):
        super().__init__(node_name='gr_kinematic_sim', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, "/robot1/cmd_vel", 10)
        
        self.robot_types = try_get(lambda: self.get_parameter("robot_types"), None) # Этот параметр отвечает за спавн роботов в сценариях. Если он None, используется спавн через функцию мэйн
        self.robot_coords = try_get(lambda: self.get_parameter("robot_coords"), None) # Этот параметр отвечает за спавн роботов в сценариях. Если он None, используется спавн через функцию мэйн
        self.robots = None
        
        if self.robot_types is not None and self.robot_coords is not None and len(self.robot_coords) == len(self.robot_types) * 2:
            self.robots = []
            for i in range(len(self.robot_types)):
                robot = {
                    'type' : self.robot_types[i],
                    'x_coords' : self.robot_coords[i*2],
                    'y_coords' : self.robot_coords[i*2 + 1],
                }
                
                self.robots.append(robot)
        elif self.robot_types is None or self.robot_coords is None:
            self.get_logger().warn('SIM WARNING: No scenario launched, proceeding with the default one')
        else:
            self.get_logger().warn(f"\nself.robot_types = {self.robot_types}\nself.robot_coords = {self.robot_coords}")
            self.get_logger().fatal("SIM ERROR: Robot types' size and coords' size do not match. Please check the scenario launch file.")
            self.get_logger().fatal("Shutting down...")
            rclpy.shutdown()
        
        self.map_name = try_get(lambda: self.get_parameter("map_name"), None) 

    def parse_robots(self, robot_factory : RobotFactory, all_sprites_list : list):
        """Парсит роботов из параметра robots и создает их в заданных координатах.
        На текущий момент работает только с роботами, создаваемыми через RobotFactory
        
        Структура robots:
            robots (array):
                robot (dict):\n
                    type (String)\n
                    x_coords (float [метры])\n
                    y_coords (float [метры])
        
        Args:
            robot_factory (RobotFactory): объект фабрики роботов, использующийся для создания роботов
        """

        if self.robots is not None:
            for robot in self.robots:
                if robot['type'] == 'ackerman':
                    new_robot = robot_factory.create_ackerman_with_lidar(robot['x_coords'] * WORLD_SCALE, robot['y_coords'] * WORLD_SCALE)
                    all_sprites_list.append(new_robot)
                if robot['type'] == 'tracked':
                    new_robot = robot_factory.create_tracked_with_lidar(robot['x_coords'] * WORLD_SCALE, robot['y_coords'] * WORLD_SCALE)
                    all_sprites_list.append(new_robot)
                if robot['type'] == 'omni':
                    new_robot = robot_factory.create_omni_with_lidar(robot['x_coords'] * WORLD_SCALE, robot['y_coords'] * WORLD_SCALE)
                    all_sprites_list.append(new_robot)
        else:
            print("No robots provided for scenario. Proceeding with default robot")










def main():    
    rclpy.init()
    node = SimNode()
    
    counter = 0
    ck = tick_rate / 10
    
    pg.init()
    # screen = pg.display.set_mode((1, 1))
    width = 1280
    height = 720

    global_offset_x = 0
    global_offset_y = 0

    screen = pg.display.set_mode((width, height))
    pg.display.set_caption('My game')

    clock = pg.time.Clock()
    
    gmap = None
    
    if node.map_name is None:
        node.map_name = 'map_test.tmx'
    try:
        gmap = FoggedMap(f'{pkg_dir}gr_kinematic_sim/maps/{node.map_name}', node)
    except Exception as e:
        print(e)
        print('Reverting to default map.')
        gmap = FoggedMap(f'{pkg_dir}gr_kinematic_sim/maps/Empty_49x49.tmx', node)
    map_img = gmap.make_map()
    map_rect = map_img.get_rect()




    all_sprites = []
    factory = RobotFactory(node, gmap, screen)

    node.parse_robots(robot_factory=factory, all_sprites_list=all_sprites)

    if node.robots is None:
        robot1 = factory.create_tracked_with_lidar(200, 200)
        # robot3 = factory.create_tracked_with_lidar(150, 150)
        # robot4 = factory.create_omni_with_lidar(150, 200)
        # robot2 = factory.create_ackerman_with_lidar(100, 150)
        # robot5 = factory.create_tracked_with_lidar(200, 150)

        all_sprites.append(robot1)
        # all_sprites.append(robot3)
        # all_sprites.append(robot4)
        # all_sprites.append(robot2)
        # all_sprites.append(robot5)

    running = True
    while running:
        global_offset_x, global_offset_y = handle_offset_change(global_offset_x, global_offset_y)
        global_offset_x, global_offset_y = scroll_screen_with_mouse(width, height, global_offset_x, global_offset_y)
        
        screen.fill((0,0,0))
        gmap.update_counter(counter)
        gmap.render(screen=screen, offset_x=global_offset_x, offset_y=global_offset_y)
        
        for event in pg.event.get():
            if event.type == pg.QUIT:
                running = False
        
        # check_kinematic_collisions_between_tilemap_and_spritelist(gmap, all_sprites)
        # check_collisions_in_spritelist(all_sprites)
        
        handle_keypresses_through_velocity(all_sprites[0], node)
        
        handle_key_events()
        move_every_sprite_in_list(all_sprites)
        draw_every_sprite_in_list(all_sprites, global_offset_x, global_offset_y)
        # check_dynamic_collisions_between_tilemap_and_spritelist(gmap, spritelist=all_sprites)
        
        # gmap.gameMap_to_OpenCv()
        ##
        gmap.OpenCV_to_OccupancyGRID(gmap.gameMap_to_OpenCv())
        ##
        # for roboti in all_sprites:
        #     roboti.draw_mask()
        pg.display.update()
        clock.tick(tick_rate)
        
        # print(f"FPS = {clock.get_fps()}")
        
        rclpy.spin_once(node, timeout_sec=1 / (2 * tick_rate))
        counter += 1
    
