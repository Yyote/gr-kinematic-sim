from gr_kinematic_sim.custom_utils.mathtools import make_non_zero, limit
import math as m


def check_dynamic_collisions_between_tilemap_and_spritelist(tilemap, spritelist):
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


def check_kinematic_collision_between_tilemap_and_rect(tilemap, rect):
    for i in range(len(tilemap.collider_list)):
        if rect.colliderect(tilemap.collider_list[i]):
            return True
    return False

def check_kinematic_collision_between_spritelis_and_rect(spritelist, sprite):
    for i in range(len(spritelist)):
        if sprite.rect.colliderect(spritelist[i].rect) and spritelist[i].name != sprite.name:
            return True
    return False


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
                    

def check_collisions_between_tilemap_and_lines(tilemap, lines):
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
        for i in range(len(tilemap.collider_list)):
            
            start_x1 = x1
            start_y1 = y1
            
            dr_local = ((x1 - tilemap.collider_list[i].centerx) ** 2 + (y1 - tilemap.collider_list[i].centery) ** 2) ** 0.5
            
            if dr_local > line_length:
                continue
            
            if tilemap.collider_list[i].clipline((x1, y1), (x2, y2)):
                for k in range(50):
                    dr = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5 * (k / 50)
                    angle = m.atan2((y2 - y1), (x2 - x1))
                    local_x2 = x1 + m.cos(angle) * dr
                    local_y2 = y1 + m.sin(angle) * dr
                    if tilemap.collider_list[i].clipline((x1, y1), (local_x2, local_y2)):
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


def find_lidar_collisions(tilemap, spritelist, lines, name):
    tilemap_cols = check_collisions_between_tilemap_and_lines(tilemap=tilemap, lines=lines)
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