import pygame as pg
from custom_utils.object_tools import Sprite

width = 1280
height = 720


pg.init()
clock = pg.time.Clock()

screen = pg.display.set_mode((width, height))
pg.display.set_caption('My game')


sprite1 = Sprite(200, 200, pg.image.load('1.jpg'), screen)

running = True
while running:
    screen.fill((0,0,0))
    
    for event in pg.event.get():
        if event.type == pg.QUIT:
            running = False
        # if event.type == pg.KEYDOWN:
        #     if event.key == pg.K_DOWN:
        #         sprite1.move(0, 5)
        #     if event.key == pg.K_UP:
        #         sprite1.move(0, -5)
        #     if event.key == pg.K_LEFT:
        #         sprite1.move(-5, 0)
        #     if event.key == pg.K_RIGHT:
        #         sprite1.move(5, 0)
    
    keys = pg.key.get_pressed()
    if keys[pg.K_LEFT]:
        sprite1.move(-1, 0)
    if keys[pg.K_RIGHT]:
        sprite1.move(1, 0)
    if keys[pg.K_UP]:
        sprite1.move(0, -1)
    if keys[pg.K_DOWN]:
        sprite1.move(0, 1)
    if keys[pg.K_z]:
        sprite1.rotate(1)
    if keys[pg.K_x]:
        sprite1.rotate(-1)
    
    sprite1.draw()
    
    pg.display.update()
    clock.tick(200)
    
    if keys[pg.K_ESCAPE]:
        pg.quit()
