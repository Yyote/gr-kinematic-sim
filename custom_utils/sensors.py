import pygame as pg
import pygame.draw as d
import math as m
# from shapely.geometry import LineString, Point, Polygon
# from shapely.ops import nearest_points

# def find_closest_intersection(rectangle, line):
#     intersection_points = rectangle.intersection(line)
#     closest_point = None
#     try:
#         closest_point = nearest_points(line, intersection_points)
#     except Exception as e:
#         print(e)
#     return closest_point






def _draw_line(screen, x1, y1, x2, y2, offset_x, offset_y):
    d.aaline(screen, (0, 0, 255 ), (x1 + offset_x, y1 + offset_y), (x2 + offset_x, y2 + offset_y), 1)
    return x1, y1, x2, y2



def get_lidar_lines_around_point(screen, x0, y0, num_points, length_pixels, current_orientation, offset_x=0, offset_y=0, draw_lines=False):
    lines = []
    current_orientation = current_orientation * m.pi / 180
    for i in range(num_points):
        angle = i * m.pi / 180.0 * (360 / num_points)
        x2 = m.cos(angle - current_orientation) * length_pixels + x0
        y2 = m.sin(angle - current_orientation) * length_pixels + y0
        if draw_lines:
            _draw_line(screen, x0, y0, x2, y2, offset_x, offset_y)
            if i == 0:
                pg.draw.line(screen, (255, 255, 0), (x0 + offset_x, y0 + offset_y), (x2 + offset_x, y2 + offset_y))
        lines.append([(x0, y0), (x2, y2)])
    
    return lines


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
