from custom_utils.mathtools import make_non_zero, limit


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
                    
