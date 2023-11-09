import pygame as pg

from custom_utils.sensors import get_lidar_lines_around_point, check_collisions_between_tilemap_and_lines
from custom_utils.physics import check_collisions_between_tilemap_and_spritelist, check_collisions_in_spritelist
from custom_utils.object_tools import Sprite, TiledMap, PhysicalObject
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

sprite1 = PhysicalObject(200, 200, pg.image.load('sprites/Arrow_east.svg.png'), screen, 1, 0.9)
sprite2 = PhysicalObject(201, 201, pg.image.load('sprites/Arrow_east.svg.png'), screen, 1, 0.9)
sprite3 = PhysicalObject(301, 301, pg.image.load('sprites/Arrow_east.svg.png'), screen, 1, 0.9)

all_sprites.append(sprite1)
all_sprites.append(sprite2)
all_sprites.append(sprite3)

running = True
while running:
    screen.fill((0,0,0))
    screen.blit(map_img, (0 + global_offset_x, 0 + global_offset_y))
    
    for event in pg.event.get():
        if event.type == pg.QUIT:
            running = False
    
    check_collisions_between_tilemap_and_spritelist(gmap, all_sprites)
    check_collisions_in_spritelist(all_sprites)
    
    handle_keypresses_through_force(sprite1)
    handle_keypresses_through_force(sprite2)
    handle_keypresses_through_force(sprite3)
    
    handle_key_events()
    
    lines = get_lidar_lines_around_point(screen, sprite1.rect.x + sprite1.rect.width / 2, sprite1.rect.y + sprite1.rect.height / 2, 40, 200, sprite1._current_rotation, offset_x=global_offset_x, offset_y=global_offset_y, draw_lines=True)
    collisions = check_collisions_between_tilemap_and_lines(screen, gmap, lines)
    
    for i in range(len(collisions)):
        if collisions[i] is not None:
            p = (collisions[i][0] + global_offset_x, collisions[i][1] + global_offset_y)
            pg.draw.circle(screen, (255, 0, 0), p, 3, 3)
    
    global_offset_x, global_offset_y = handle_offset_change(global_offset_x, global_offset_y)
    global_offset_x, global_offset_y = scroll_screen_with_mouse(width, height, global_offset_x, global_offset_y)
    
    draw_every_sprite_in_list(all_sprites, global_offset_x, global_offset_y)
    
    pg.display.update()
    clock.tick(20)
    
