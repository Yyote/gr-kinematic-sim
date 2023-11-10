import pygame
import pytmx
import math
from copy import copy
from gr_kinematic_sim.custom_utils.mathtools import normalise_in_range, sgn_wo_zero, rotation_matrix

DEFAULT_IMAGE_SIZE = (50, 50)
SMALL_IMAGE_SIZE = (25, 25)
VERY_SMALL_IMAGE_SIZE = (12, 12)

class Sprite(pygame.sprite.Sprite):
    def __init__(self, x, y, image, screen, offset_x, offset_y, image_size=DEFAULT_IMAGE_SIZE):
        super().__init__()
        self.image = pygame.transform.scale(image, image_size)
        self._original_image = pygame.transform.scale(image, image_size)
        self.rect = self.image.get_rect()
        self.rect.x = x
        self.rect.y = y
        self.screen = screen
        self._current_rotation = 0
        self.curr_offset_x = offset_x
        self.curr_offset_y = offset_y

    def rotate(self, degrees):
        self._current_rotation += degrees
        normalise_in_range(0, 360, self._current_rotation)
        self.image = pygame.transform.rotozoom(self._original_image, self._current_rotation, 1)
        self.rect = self.image.get_rect(center=self.rect.center)
    
    def set_rotation(self, angle_deg):
        self._current_rotation = angle_deg
        normalise_in_range(0, 360, self._current_rotation)
        self.image = pygame.transform.rotozoom(self._original_image, self._current_rotation, 1)
        self.rect = self.image.get_rect(center=self.rect.center)
    
    def move(self, x, y, degrees=0):
        self.rect.x += x
        self.rect.y += y
        self.rotate(degrees)
    
    def draw(self):
        rect_to_draw = copy(self.rect)
        rect_to_draw.x += self.curr_offset_x
        rect_to_draw.y += self.curr_offset_y
        self.screen.blit(self.image, rect_to_draw)
    
    def update_offset(self, x, y):
        self.curr_offset_x = x
        self.curr_offset_y = y
    

class PhysicalObject(Sprite):
    def __init__(self, x, y, image, screen, offset_x, offset_y, mass, friction_multiplier=0.95, image_size=DEFAULT_IMAGE_SIZE):
        super().__init__(x, y, image, screen, offset_x, offset_y, image_size)
        self.mass = mass
        self.friction_multiplier = friction_multiplier
        
        self.lin_vel_x = 0
        self.lin_vel_y = 0
        
        self.lin_accel_x = 0
        self.lin_accel_y = 0
            
        self.ang_vel = 0
        
        self.ang_accel = 0
    
    def apply_force_now(self, lin_force_x, lin_force_y, ang_force=0):
        self.lin_accel_x += lin_force_x / self.mass
        self.lin_accel_y += lin_force_y / self.mass
        
        self.lin_vel_x += self.lin_accel_x
        self.lin_vel_y += self.lin_accel_y
        
        self.ang_accel += ang_force / self.mass
        
        self.ang_vel += self.ang_accel

    def apply_force_now_local(self, lin_force_x, lin_force_y, ang_force=0):
        local_force_x, local_force_y = rotation_matrix(lin_force_x, lin_force_y, -self._current_rotation * math.pi / 180)
        self.apply_force_now(local_force_x, local_force_y, ang_force)
    
    def _friction(self):
            self.lin_vel_x *= self.friction_multiplier
            self.lin_vel_y *= self.friction_multiplier
            
            self.lin_accel_x *= self.friction_multiplier
            self.lin_accel_y *= self.friction_multiplier

            self.ang_vel *= self.friction_multiplier
            
            self.ang_accel *= self.friction_multiplier
            

    def _move(self):
        self._friction()
        self.rect.x += self.lin_vel_x
        self.rect.y += self.lin_vel_y
        
        self.rotate(self.ang_vel)


    def draw(self, offset_x, offset_y):
        self.update_offset(offset_x, offset_y)
        super().draw()
        self._move()


class Robot(PhysicalObject):
    def __init__(self, x, y, image, screen, offset_x, offset_y, mass=1, friction_multiplier=0.95, image_size=DEFAULT_IMAGE_SIZE):
        super().__init__(x, y, image, screen, offset_x, offset_y, mass, friction_multiplier, image_size)
        self.sensors = None
    
    def call_sensors(self, tilemap):
        for sensor in self.sensors:
            sensor.update_offset(self.curr_offset_x, self.curr_offset_y)
            sensor.set_center_position(self.rect.x + self.rect.width / 2, self.rect.y + self.rect.height / 2, self._current_rotation)
            sensor.draw()
            sensor.logic(tilemap)
    
    def draw(self, offset_x, offset_y, tilemap):
        super().draw(offset_x, offset_y)
        self.call_sensors(tilemap)
    
    def set_sensors(self, sensors=[]):
        if type(sensors) != type([]):
            print('TypeError: `sensors` must be an array!')
            raise TypeError('`sensors` must be an array!')
            quit()
        self.sensors = sensors





class TiledMap():
    def __init__(self, map_path):
        self.gameMap = pytmx.load_pygame(map_path, pixelalpha=True)
        self.mapwidth = self.gameMap.tilewidth * self.gameMap.width
        self.mapheight = self.gameMap.tileheight * self.gameMap.height
        self.collider_list = []


    def _render(self, surface):
        for layer in self.gameMap.visible_layers:
            if isinstance(layer, pytmx.TiledTileLayer):
                for x, y, gid in layer:
                    tile = self.gameMap.get_tile_image_by_gid(gid)
                    if gid == 1:
                        self.collider_list.append(pygame.Rect(x * self.gameMap.tilewidth, y * self.gameMap.tileheight, self.gameMap.tilewidth, self.gameMap.tileheight))
                    if tile:
                        surface.blit(tile, (x * self.gameMap.tilewidth, y * self.gameMap.tileheight))


    def make_map(self):
        """
        Brief:
            Генерирует карту, которую потом можно отобразить

        Returns:
            pygame.Surface : mapSurface
        """
        self.mapSurface = pygame.Surface((self.mapwidth, self.mapheight))
        self._render(self.mapSurface)
        return self.mapSurface