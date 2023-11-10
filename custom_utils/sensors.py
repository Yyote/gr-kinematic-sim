import pygame as pg
import pygame.draw as d
import math as m
from custom_utils.object_tools import Sprite, SMALL_IMAGE_SIZE, DEFAULT_IMAGE_SIZE, VERY_SMALL_IMAGE_SIZE
from custom_utils.collisions import check_collisions_between_tilemap_and_lines



class Lidar(Sprite):
    def __init__(self, x, y, image, screen, offset_x, offset_y, num_rays, ray_length_px):
        super().__init__(x, y, image, screen, offset_x, offset_y, VERY_SMALL_IMAGE_SIZE)
        self.curr_offset_x = offset_x
        self.curr_offset_y = offset_y
        self.num_rays = num_rays
        self.ray_length_px = ray_length_px
    
    def _draw_line(self, screen, x1, y1, x2, y2):
        d.aaline(screen, (0, 0, 255 ), (x1 + self.curr_offset_x, y1 + self.curr_offset_y), (x2 + self.curr_offset_x, y2 + self.curr_offset_y), 1)
        return x1, y1, x2, y2
    
    def get_lidar_lines_around_point(self, screen, draw_lines=False):
        lines = []
        x0 = self.rect.x + self.rect.width / 2
        y0 = self.rect.y + self.rect.height / 2
        current_orientation = (self._current_rotation - 90) * m.pi / 180
        for i in range(self.num_rays):
            angle = i * m.pi / 180.0 * (360 / self.num_rays)
            x2 = m.cos(angle - current_orientation) * self.ray_length_px + x0
            y2 = m.sin(angle - current_orientation) * self.ray_length_px + y0
            if draw_lines:
                self._draw_line(screen, x0, y0, x2, y2)
                if i == 0:
                    pg.draw.line(screen, (255, 255, 0), (x0 + self.curr_offset_x, y0 + self.curr_offset_y), (x2 + self.curr_offset_x, y2 + self.curr_offset_y))
            lines.append([(x0, y0), (x2, y2)])
        
        return lines

    def logic(self, tilemap):
        lines = self.get_lidar_lines_around_point(self.screen, True)
        collisions = check_collisions_between_tilemap_and_lines(self.screen, tilemap, lines)
        for i in range(len(collisions)):
            if collisions[i] is not None:
                p = (collisions[i][0] + self.curr_offset_x, collisions[i][1] + self.curr_offset_y)
                pg.draw.circle(self.screen, (255, 0, 0), p, 3, 3)
        return collisions
    
    def set_center_position(self, center_x, center_y, rotation_deg):
        self.rect.centerx = center_x
        self.rect.centery = center_y
        self.set_rotation(rotation_deg)



class LidarB1(Lidar):
    def __init__(self, screen):
        super().__init__(200, 200, pg.image.load('sprites/LidarBig.png'), screen, 0, 0, 60, 180)


