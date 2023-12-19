import pygame as pg
import rclpy
import os

from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist

from ament_index_python.packages import get_package_share_directory
from gr_kinematic_sim.custom_utils.sensors import Lidar, LidarB1
from gr_kinematic_sim.custom_utils.collisions import check_dynamic_collisions_between_tilemap_and_spritelist, check_collisions_in_spritelist, check_collisions_between_tilemap_and_lines
from gr_kinematic_sim.custom_utils.object_tools import Sprite, FoggedMap, PhysicalObject
from gr_kinematic_sim.custom_utils.robots import Robot, AckermanRobot, RobotFactory

from gr_kinematic_sim.custom_utils.gametools import handle_key_events, handle_offset_change, handle_keypresses_through_force, draw_every_sprite_in_list, scroll_screen_with_mouse, handle_keypresses_through_velocity, handle_keypresses_through_velocity_omni
from gr_kinematic_sim.custom_utils.gametools import tick_rate

import faulthandler
faulthandler.enable()

pkg_dir = f"{get_package_share_directory('gr_kinematic_sim')}/../../../../src/gr_kinematic_sim/"


class SimNode(Node):
    def __init__(self):
        super().__init__('gr_kinematic_sim')
        self.cmd_vel_pub = self.create_publisher(TwistStamped, "/robot1/cmd_vel", 10)





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
    gmap = FoggedMap(f'{pkg_dir}gr_kinematic_sim/maps/map_test.tmx', node)
    map_img = gmap.make_map()
    map_rect = map_img.get_rect()




    all_sprites = []

    factory = RobotFactory(node, gmap, screen)

    robot1 = factory.create_ackerman_with_lidar(200, 200)
    robot3 = factory.create_tracked_with_lidar(150, 150)
    robot4 = factory.create_omni_with_lidar(150, 200)
    robot2 = factory.create_tracked_with_lidar(100, 150)



    all_sprites.append(robot1)
    all_sprites.append(robot3)
    all_sprites.append(robot4)
    all_sprites.append(robot2)

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
        
        handle_keypresses_through_velocity(robot1, node)
        
        handle_key_events()
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
        
        print(clock.get_fps())
        
        rclpy.spin_once(node, timeout_sec=1 / (2 * tick_rate))
        counter += 1
    

if __name__ == '__main__':
    main()
    rclpy.shutdown()