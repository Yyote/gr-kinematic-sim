import pygame as pg
import pygame.draw as d
import math as m
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import nearest_points

def find_closest_intersection(rectangle, line):
    intersection_points = rectangle.intersection(line)
    closest_point = None
    try:
        closest_point = nearest_points(line, intersection_points)
    except Exception as e:
        print(e)
    return closest_point






def _draw_line(screen, x1, y1, x2, y2, offset_x, offset_y):
    d.aaline(screen, (0, 0, 255 ), (x1 + offset_x, y1 + offset_y), (x2 + offset_x, y2 + offset_y), 1)
    return x1, y1, x2, y2

def draw_lidar_around_point(screen, x0, y0, num_points, length_pixels):
    lines = []
    for i in range(num_points):
        angle = i * m.pi / 180.0 * (360 / num_points)
        x2 = m.cos(angle) * length_pixels + x0
        y2 = m.sin(angle) * length_pixels + y0
        _draw_line(screen, x0, y0, x2, y2, 0, 0)
        lines.append([(x0, y0), (x2, y2)])
    
    return lines

def check_collisions_between_tilemap_and_lines(tilemap, lines):
    collisions = {}
    col_points = []
    for j in range(len(lines)):
        for i in range(len(tilemap.collider_list)):
            x1 = lines[j][0][0]
            y1 = lines[j][0][1]
            
            x2 = lines[j][1][0]
            y2 = lines[j][1][1]
            
            # print(tilemap.collider_list[i].clipline((x1, y1), (x2, y2)))
            if tilemap.collider_list[i].clipline((x1, y1), (x2, y2)):
                line_points = [(x1, y1), (x2, y2)]
                line = LineString(line_points)

                rect = tilemap.collider_list[i]
                # rect = pg.Rect()

                rect_tl = (rect.x, rect.y)
                rect_tr = (rect.x, rect.y + rect.width)
                rect_bl = (rect.x + rect.height, rect.y)
                rect_br = (rect.x + rect.height, rect.y + rect.width)
                
                rectangle = Polygon([rect_tl, rect_tr, rect_br, rect_bl])
            
                if j not in collisions:
                    collisions[j] = []
                
                closest_point = find_closest_intersection(rectangle, line)
                if closest_point is not None:
                    for ki in range(len(closest_point)):
                        collisions[j].append((int(closest_point[ki].x), int(closest_point[ki].y)))
        
        for key in collisions:
            smallest_distance = 9999999999
            closest_point_calc = None
            
            for k in range(len(collisions[key])):
                p0 = line_points[0]
                p1 = (collisions[key][k][0], collisions[key][k][1])
                
                dr = ((p1[0] - p0[0]) ** 2 + (p1[1] - p0[1]) ** 2) ** 0.5
                if dr < smallest_distance:
                    smallest_distance = dr
                    closest_point_calc = p1
            col_points.append(closest_point_calc)
        
        
    return col_points
