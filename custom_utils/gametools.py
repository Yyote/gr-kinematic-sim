import pygame as pg
from custom_utils.mathtools import make_non_zero, limit


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


def handle_keypresses_through_force(sprite):
    """
    Brief: 
        Работает только с классом custom_utils.object_tools.Sprite
    """
    keys = pg.key.get_pressed()
    if keys[pg.K_LEFT]:
        sprite.apply_force_now_local(-0.05, 0)
    if keys[pg.K_RIGHT]:
        sprite.apply_force_now_local(0.05, 0)
    if keys[pg.K_UP]:
        sprite.apply_force_now_local(0, -0.05)
    if keys[pg.K_DOWN]:
        sprite.apply_force_now_local(0, 0.05)
    if keys[pg.K_z]:
        sprite.apply_force_now_local(0, 0, 0.1)
    if keys[pg.K_x]:
        sprite.apply_force_now_local(0, 0, -0.1)
    if keys[pg.K_ESCAPE]:
        pg.quit()
        

def draw_every_sprite_in_list(list_):
    for sprite in list_:
        sprite.draw()
        

def check_collisions_between_tilemap_and_spritelist(tilemap, spritelist):
    
    for i in range(len(tilemap.collider_list)):
        for j in range(len(spritelist)):
            if spritelist[j].rect.colliderect(tilemap.collider_list[i]):
                spritelist[j].lin_accel_x = 0
                spritelist[j].lin_accel_y = 0
                
                dx = spritelist[j].rect.x - tilemap.collider_list[i].x
                dy = spritelist[j].rect.y - tilemap.collider_list[i].y
                
                
                spritelist[j].apply_force_now(limit(8 / (make_non_zero(dx)), 3), limit(8 / (make_non_zero(dy)), 3))
                
                spritelist[j].ang_accel = 0
                spritelist[j].ang_vel = - spritelist[j].ang_vel


def check_collisions_in_spritelist(spritelist):
    for i in range(len(spritelist)):
        for j in range(len(spritelist)):
            if j > i:
                if spritelist[j].rect.colliderect(spritelist[i].rect):
                    spritelist[j].lin_accel_x = 0
                    spritelist[j].lin_accel_y = 0
                    
                    dx = spritelist[j].rect.x - spritelist[i].rect.x
                    dy = spritelist[j].rect.y - spritelist[i].rect.y
                    
                    spritelist[i].lin_accel_x = 0
                    spritelist[i].lin_accel_y = 0
                    
                    dx2 = spritelist[i].rect.x - spritelist[j].rect.x
                    dy2 = spritelist[i].rect.y - spritelist[j].rect.y
                    
                    spritelist[j].ang_accel = 0
                    spritelist[j].ang_vel = - spritelist[j].ang_vel
                    
                    spritelist[i].ang_accel = 0
                    spritelist[i].ang_vel = - spritelist[i].ang_vel
                    
                    spritelist[j].apply_force_now(limit(8 / (make_non_zero(dx)), 3), limit(8 / (make_non_zero(dy)), 3))
                    spritelist[i].apply_force_now(limit(8 / (make_non_zero(dx2)), 3), limit(8 / (make_non_zero(dy2)), 3))