import pygame as pg

def handle_keypresses(sprite):
    """
    Brief: 
        Работает только с классом custom_utils.object_tools.Sprite
    """
    keys = pg.key.get_pressed()
    if keys[pg.K_LEFT]:
        sprite.move(-1, 0)
    if keys[pg.K_RIGHT]:
        sprite.move(1, 0)
    if keys[pg.K_UP]:
        sprite.move(0, -1)
    if keys[pg.K_DOWN]:
        sprite.move(0, 1)
    if keys[pg.K_z]:
        sprite.rotate(1)
    if keys[pg.K_x]:
        sprite.rotate(-1)
    if keys[pg.K_ESCAPE]:
        pg.quit()