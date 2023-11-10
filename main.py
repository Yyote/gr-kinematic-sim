import pygame as pg

from custom_utils.sensors import Lidar, LidarB1
from custom_utils.collisions import check_collisions_between_tilemap_and_spritelist, check_collisions_in_spritelist, check_collisions_between_tilemap_and_lines
from custom_utils.object_tools import Sprite, TiledMap, PhysicalObject, Robot
from custom_utils.gametools import handle_key_events, handle_offset_change, handle_keypresses_through_force, draw_every_sprite_in_list, scroll_screen_with_mouse

pg.init()

screen = pg.display.set_mode((1, 1))

clock = pg.time.Clock()
gmap = TiledMap('maps/map_test.tmx')
map_img = gmap.make_map()
map_rect = map_img.get_rect()

# width = map_img.get_width()
# height = map_img.get_height()

width = 1280
height = 720

global_offset_x = 0
global_offset_y = 0

screen = pg.display.set_mode((width, height))
pg.display.set_caption('My game')

all_sprites = []

sensors1 = [LidarB1(screen)]
robot1 = Robot(200, 200, pg.image.load('sprites/robots/wheeled.png'), screen, global_offset_x, global_offset_y, 2, 0.9, (25, 25))
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
    
    check_collisions_between_tilemap_and_spritelist(gmap, all_sprites)
    check_collisions_in_spritelist(all_sprites)
    
    handle_keypresses_through_force(robot1)
    
    handle_key_events()
    
    draw_every_sprite_in_list(all_sprites, global_offset_x, global_offset_y, gmap)
    
    pg.display.update()
    clock.tick(20)
    
