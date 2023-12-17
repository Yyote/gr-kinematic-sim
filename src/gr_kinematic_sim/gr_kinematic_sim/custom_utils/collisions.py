from gr_kinematic_sim.custom_utils.mathtools import make_non_zero, limit
import math as m
import numpy as np
import pygame as pg

def get_distance_between_rect_centers(rect1, rect2):
    c1 = np.array(rect1.center)
    c2 = np.array(rect2.center)
    
    dc = c2 - c1
    # print(dc)
    dc ** 2
    # print(dc)
    dc = dc.sum() ** 0.5
    # print(dc)
    return dc

def rects_in_distance(rect1, rect2, distance):
    return get_distance_between_rect_centers(rect1, rect2) < distance

def check_dynamic_collisions_between_tilemap_and_spritelist(tilemap, spritelist):
    for j in range(len(spritelist)):
        local_colliders = []
        start_x1 = spritelist[j].rect.centerx
        start_y1 = spritelist[j].rect.centery
        # Фильтрация препятствий по расстоянию до спрайта
        for i in range(len(tilemap.collider_list)):
            dr = ((start_x1 - tilemap.collider_list[i].centerx) ** 2 + (start_y1 - tilemap.collider_list[i].centery) ** 2) ** 0.5
            if dr < spritelist[j].rect.width * 2 ** 0.5:
                local_colliders.append(tilemap.collider_list[i])
            
        for i in range(len(local_colliders)):
            if spritelist[j].rect.colliderect(local_colliders[i]):
                spritelist[j].lin_accel_x = 0
                spritelist[j].lin_accel_y = 0
                
                dx = spritelist[j].rect.x - local_colliders[i].x
                dy = spritelist[j].rect.y - local_colliders[i].y
                
                
                spritelist[j].apply_force_now(limit(8 / (make_non_zero(dx)), 3), limit(8 / (make_non_zero(dy)), 3))
                
                spritelist[j].ang_accel = 0
                spritelist[j].ang_vel = - spritelist[j].ang_vel


def check_kinematic_collision_between_tilemap_and_rect(tilemap, rect):
    for i in range(len(tilemap.collider_list)):
        if rect.colliderect(tilemap.collider_list[i]):
            return (True, tilemap.collider_list[i])
    return (False, None)


def check_mask_collision_between_tilemap_and_sprite(tilemap, sprite):
    for x in range(tilemap.gameMap.width):
        for y in range(tilemap.gameMap.height):
            dx = tilemap.map_dict[x][y].rect.x - sprite.rect.x
            dy = tilemap.map_dict[x][y].rect.y  - sprite.rect.y
            dr = m.sqrt(dx ** 2 + dy ** 2)
            if dr < m.sqrt(sprite.rect.height ** 2 + sprite.rect.width ** 2):
                if sprite.get_mask().overlap(tilemap.map_dict[x][y].get_mask(), (dx, dy)) and tilemap.map_dict[x][y].real_gid == 1:
                    return (True, tilemap.map_dict[x][y].rect)
    return (False, None)

def check_kinematic_collision_between_spritelis_and_rect(spritelist, sprite):
    for i in range(len(spritelist)):
        if sprite.rect.colliderect(spritelist[i].rect) and spritelist[i].name != sprite.name:
            return (True, spritelist[i].rect)
    return (False, None)


def check_mask_collision_between_spritelis_and_rect(spritelist, sprite):
    for i in range(len(spritelist)):
        if spritelist[i].name != sprite.name:
            print(f"2.: {sprite.get_mask().overlap(spritelist[i].get_mask(), (spritelist[i].rect.x, spritelist[i].rect.y))}")
            if sprite.get_mask().overlap(spritelist[i].get_mask(), (spritelist[i].rect.x - sprite.rect.x, spritelist[i].rect.y  - sprite.rect.y)):
                return (True, spritelist[i].rect)
    return (False, None)

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
                    

def check_collisions_between_tilemap_and_lines(tilemap, lines, line_length):
    collisions = {}
    col_points = []

    start_x1 = None
    start_y1 = None
    x1 = lines[0][0][0]
    y1 = lines[0][0][1]
    
    if start_x1 == None:
        start_x1 = x1
        start_y1 = y1
    
    local_colliders = []
    
    # Фильтрация препятствий по расстоянию до лидара
    for i in range(len(tilemap.collider_list)):
        dr = ((start_x1 - tilemap.collider_list[i].centerx) ** 2 + (start_y1 - tilemap.collider_list[i].centery) ** 2) ** 0.5
        if dr < line_length:
            local_colliders.append(tilemap.collider_list[i])
    
    # Поиск столкновений
    for j in range(len(lines)):
        x2 = lines[j][1][0]
        y2 = lines[j][1][1]
        
        for i in range(len(local_colliders)):
            if local_colliders[i].clipline((x1, y1), (x2, y2)):
                for k in range(50):
                    dr = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5 * (k / 50)
                    angle = m.atan2((y2 - y1), (x2 - x1))
                    local_x2 = x1 + m.cos(angle) * dr
                    local_y2 = y1 + m.sin(angle) * dr
                    if local_colliders[i].clipline((x1, y1), (local_x2, local_y2)):
                        if j not in collisions:
                            collisions[j] = []
                        
                        collisions[j].append((local_x2, local_y2))
                        break
        
        if j in collisions:
            smallest_idx = 0
            smallest_dr = float('inf')
            for i in range(len(collisions[j])):
                dr = ((start_x1 - collisions[j][i][0]) ** 2 + (start_y1 - collisions[j][i][1]) ** 2) ** 0.5
                if dr < smallest_dr:
                    smallest_dr = dr
                    smallest_idx = i
            col_points.append(collisions[j][smallest_idx])
        else:
            col_points.append((float('inf'),float('inf')))
    return col_points


def check_collisions_between_spritelist_and_lines(spritelist, lines, name):
    collisions = {}
    col_points = []
    
    for j in range(len(lines)):
        start_x1 = None
        start_y1 = None
        x1 = lines[j][0][0]
        y1 = lines[j][0][1]
        
        x2 = lines[j][1][0]
        y2 = lines[j][1][1]
        
        line_length = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
        for i in range(len(spritelist)):
            if spritelist[i].name != name:
                start_x1 = x1
                start_y1 = y1
                
                dr_local = ((x1 - spritelist[i].rect.centerx) ** 2 + (y1 - spritelist[i].rect.centery) ** 2) ** 0.5
                
                if dr_local > line_length:
                    continue
                
                if spritelist[i].rect.clipline((x1, y1), (x2, y2)):
                    for k in range(50):
                        dr = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5 * (k / 50)
                        angle = m.atan2((y2 - y1), (x2 - x1))
                        local_x2 = x1 + m.cos(angle) * dr
                        local_y2 = y1 + m.sin(angle) * dr
                        if spritelist[i].rect.clipline((x1, y1), (local_x2, local_y2)):
                            if j not in collisions:
                                collisions[j] = []
                            
                            collisions[j].append((local_x2, local_y2))
                            break
        
        if j in collisions:
            smallest_idx = 0
            smallest_dr = float('inf')
            for i in range(len(collisions[j])):
                dr = ((start_x1 - collisions[j][i][0]) ** 2 + (start_y1 - collisions[j][i][1]) ** 2) ** 0.5
                if dr < smallest_dr:
                    smallest_dr = dr
                    smallest_idx = i
            col_points.append(collisions[j][smallest_idx])
        else:
            col_points.append((float('inf'),float('inf')))
    return col_points


def find_lidar_collisions(tilemap, spritelist, lines, name, line_length_pxls):
    tilemap_cols = check_collisions_between_tilemap_and_lines(tilemap=tilemap, lines=lines, line_length=line_length_pxls)
    spritelist_cols = check_collisions_between_spritelist_and_lines(spritelist=spritelist, lines=lines, name=name)
    
    x0 = lines[0][0][0]
    y0 = lines[0][0][1]
    
    result_cols  = []
    
    for i in range(len(tilemap_cols)):
        dr_t = ((x0 - tilemap_cols[i][0]) ** 2 + (y0 - tilemap_cols[i][1]) ** 2) ** 0.5
        dr_s = ((x0 - spritelist_cols[i][0]) ** 2 + (y0 - spritelist_cols[i][1]) ** 2) ** 0.5
        
        if dr_t < dr_s:
            result_cols.append(tilemap_cols[i])
        else:
            result_cols.append(spritelist_cols[i])
    
    return result_cols