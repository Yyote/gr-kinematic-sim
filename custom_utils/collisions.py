from custom_utils.mathtools import make_non_zero, limit
import math as m


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
                    

def check_collisions_between_tilemap_and_lines(screen, tilemap, lines):
    collisions = {}
    col_points = []
    
    for j in range(len(lines)):
        start_x1 = None
        start_y1 = None
        for i in range(len(tilemap.collider_list)):
            x1 = lines[j][0][0]
            y1 = lines[j][0][1]
            
            x2 = lines[j][1][0]
            y2 = lines[j][1][1]
            
            start_x1 = x1
            start_y1 = y1
            
            # print(tilemap.collider_list[i].clipline((x1, y1), (x2, y2)))
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
        
    return col_points