import pygame as pg
import rclpy
import os

from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from gr_kinematic_sim.custom_utils.sensors import Lidar, LidarB1
from gr_kinematic_sim.custom_utils.collisions import check_dynamic_collisions_between_tilemap_and_spritelist, check_collisions_in_spritelist, check_collisions_between_tilemap_and_lines
from gr_kinematic_sim.custom_utils.object_tools import Sprite, TiledMap, PhysicalObject, Robot, AckermanRobot

from gr_kinematic_sim.custom_utils.gametools import handle_key_events, handle_offset_change, handle_keypresses_through_force, draw_every_sprite_in_list, scroll_screen_with_mouse, handle_keypresses_through_velocity

pkg_dir = f"{get_package_share_directory('gr_kinematic_sim')}/../../../../src/gr_kinematic_sim/"


class SimNode(Node):
    def __init__(self):
        super().__init__('gr_kinematic_sim')





def main():
    rclpy.init()
    node = SimNode()
    
    pg.init()
    screen = pg.display.set_mode((1, 1))

    clock = pg.time.Clock()
    gmap = TiledMap(f'{pkg_dir}gr_kinematic_sim/maps/map_test.tmx')
    map_img = gmap.make_map()
    map_rect = map_img.get_rect()


    width = 1280
    height = 720

    global_offset_x = 0
    global_offset_y = 0

    screen = pg.display.set_mode((width, height))
    pg.display.set_caption('My game')

    all_sprites = []

    sensors1 = [LidarB1("/robot1", screen, node)]
    robot1 = AckermanRobot("/robot1", gmap, 200, 200, pg.image.load(f'{pkg_dir}gr_kinematic_sim/sprites/robots/wheeled.png'), screen, global_offset_x, global_offset_y, 2, 0.9, (25, 25))
    robot1.set_sensors(sensors1)



    all_sprites.append(robot1)

    running = True
    while running:
        global_offset_x, global_offset_y = handle_offset_change(global_offset_x, global_offset_y)
        global_offset_x, global_offset_y = scroll_screen_with_mouse(width, height, global_offset_x, global_offset_y)
        
        screen.fill((0,0,0))
        screen.blit(map_img, (0 + global_offset_x, 0 + global_offset_y))
        
        for event in pg.event.get():
            if event.type == pg.QUIT:
                running = False
        
        # check_kinematic_collisions_between_tilemap_and_spritelist(gmap, all_sprites)
        check_collisions_in_spritelist(all_sprites)
        
        handle_keypresses_through_velocity(robot1)
        
        handle_key_events()
        
        draw_every_sprite_in_list(all_sprites, global_offset_x, global_offset_y)
        
        pg.display.update()
        clock.tick(20)
    

if __name__ == '__main__':
    main()