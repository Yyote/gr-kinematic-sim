import pygame as pg

from custom_utils.object_tools import Sprite, TiledMap
from custom_utils.gametools import handle_keypresses

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


sprite1 = Sprite(200, 200, pg.image.load('1.jpg'), screen)

running = True
while running:
    screen.fill((0,0,0))
    screen.blit(map_img, (0, 0))
    
    for event in pg.event.get():
        if event.type == pg.QUIT:
            running = False
    
    handle_keypresses(sprite1)
    
    sprite1.draw()
    
    pg.display.update()
    clock.tick(20000)
    
