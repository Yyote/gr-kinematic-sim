import pygame as pg

from custom_utils.object_tools import Sprite, TiledMap, PhysicalObject
from custom_utils.gametools import handle_keypresses, handle_keypresses_through_force, draw_every_sprite_in_list, check_collisions_between_tilemap_and_spritelist, check_collisions_in_spritelist

pg.init()

screen = pg.display.set_mode((1, 1))

clock = pg.time.Clock()
gmap = TiledMap('maps/map_test.tmx')
map_img = gmap.make_map()
map_rect = map_img.get_rect()

width = map_img.get_width()
height = map_img.get_height()



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
    screen.blit(map_img, (0, 0))
    
    for event in pg.event.get():
        if event.type == pg.QUIT:
            running = False
    
    check_collisions_between_tilemap_and_spritelist(gmap, all_sprites)
    check_collisions_in_spritelist(all_sprites)
    
    handle_keypresses_through_force(sprite1)
    handle_keypresses_through_force(sprite2)
    handle_keypresses_through_force(sprite3)
    
    draw_every_sprite_in_list(all_sprites)
    
    pg.display.update()
    clock.tick(20)
    
