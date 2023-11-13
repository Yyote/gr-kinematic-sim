import pygame
import pytmx
import math
from copy import copy
from gr_kinematic_sim.custom_utils.mathtools import normalise_in_range, sgn_wo_zero, rotation_matrix
from gr_kinematic_sim.custom_utils.collisions import check_kinematic_collision_between_tilemap_and_rect

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
        self._current_rotation = normalise_in_range(0, 360, self._current_rotation)
        self.image = pygame.transform.rotozoom(self._original_image, self._current_rotation, 1)
        self.rect = self.image.get_rect(center=self.rect.center)
    
    def set_rotation(self, angle_deg):
        self._current_rotation = angle_deg
        self._current_rotation = normalise_in_range(0, 360, self._current_rotation)
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
    def __init__(self, name, tilemap, x, y, image, screen, offset_x, offset_y, mass, friction_multiplier=0.95, image_size=DEFAULT_IMAGE_SIZE, dynamic_model=True):
        super().__init__(x, y, image, screen, offset_x, offset_y, image_size)
        self.mass = mass
        self.friction_multiplier = friction_multiplier
        
        self.lin_vel_x = 0
        self.lin_vel_y = 0
        
        self.lin_accel_x = 0
        self.lin_accel_y = 0
            
        self.ang_vel = 0
        self.ang_accel = 0
        
        self.dynamic_model = dynamic_model
        self.tilemap = tilemap
        self.name = name
    
    def kinematic_collision_check(self, rect):
        return check_kinematic_collision_between_tilemap_and_rect(self.tilemap, rect)
    
    def apply_force_now(self, lin_force_x, lin_force_y, ang_force=0):
        if self.dynamic_model == False:
            raise Exception('Dynamic modeling is turned off for this model, so this function should not be called. Please, check your code')
            quit()
        self.lin_accel_x += lin_force_x / self.mass
        self.lin_accel_y += lin_force_y / self.mass
        
        self.lin_vel_x += self.lin_accel_x
        self.lin_vel_y += self.lin_accel_y
        
        self.ang_accel += ang_force / self.mass
        
        self.ang_vel += self.ang_accel

    def apply_force_now_local(self, lin_force_x, lin_force_y, ang_force=0):
        if self.dynamic_model == False:
            raise Exception('Dynamic modeling is turned off for this model, so this function should not be called. Please, check your code')
            quit()
        local_force_x, local_force_y = rotation_matrix(lin_force_x, lin_force_y, -self._current_rotation * math.pi / 180)
        self.apply_force_now(local_force_x, local_force_y, ang_force)
    
    def set_velocity(self, vel_x, vel_y, ang_vel):
        self.lin_accel_x = 0
        self.lin_accel_y = 0
        self.ang_accel = 0
        
        self.lin_vel_x = vel_x
        self.lin_vel_y = vel_y
        self.ang_vel = ang_vel
    
    def set_local_velocity(self, vel_x, vel_y, ang_vel):
        self.lin_accel_x = 0
        self.lin_accel_y = 0
        self.ang_accel = 0
        
        local_vel_x, local_vel_y = rotation_matrix(vel_x, vel_y, -self._current_rotation * math.pi / 180)
        
        self.lin_vel_x = local_vel_x
        self.lin_vel_y = local_vel_y
        self.ang_vel = ang_vel

    def _friction(self):
        self.lin_vel_x *= self.friction_multiplier
        self.lin_vel_y *= self.friction_multiplier
        
        self.lin_accel_x *= self.friction_multiplier
        self.lin_accel_y *= self.friction_multiplier

        self.ang_vel *= self.friction_multiplier
        
        self.ang_accel *= self.friction_multiplier

    def _move(self):
        if self.dynamic_model:
            self._friction()
        copied_rect = copy(self.rect)
        copied_rect.x += self.lin_vel_x
        copied_rect.y += self.lin_vel_y
        if not self.dynamic_model and self.kinematic_collision_check(copied_rect):
            self.lin_vel_x = 0
            self.lin_vel_y = 0
            self.ang_vel = 0
        self.rect.x += self.lin_vel_x
        self.rect.y += self.lin_vel_y
        self.rotate(self.ang_vel)

    def draw(self, offset_x, offset_y):
        self.update_offset(offset_x, offset_y)
        self._move()
        super().draw()



class Robot(PhysicalObject):
    def __init__(self, name, tilemap, x, y, image, screen, offset_x, offset_y, mass=1, friction_multiplier=0.95, image_size=DEFAULT_IMAGE_SIZE, dynamic_model=True):
        super().__init__(name, tilemap, x, y, image, screen, offset_x, offset_y, mass, friction_multiplier, image_size, dynamic_model)
        self.sensors = None
    
    def call_sensors(self):
        for sensor in self.sensors:
            sensor.update_offset(self.curr_offset_x, self.curr_offset_y)
            sensor.set_center_position(self.rect.x + self.rect.width / 2, self.rect.y + self.rect.height / 2, self._current_rotation)
            sensor.draw()
            sensor.logic(self.tilemap)
    
    def draw(self, offset_x, offset_y):
        super().draw(offset_x, offset_y)
        self.call_sensors()
    
    def set_sensors(self, sensors=[]):
        if type(sensors) != type([]):
            print('TypeError: `sensors` must be an array!')
            raise TypeError('`sensors` must be an array!')
            quit()
        self.sensors = sensors


class AckermanRobot(Robot):
    def __init__(self, name, tilemap, x, y, image, screen, offset_x, offset_y, mass=1, friction_multiplier=0.95, image_size=DEFAULT_IMAGE_SIZE):
        super().__init__(name, tilemap, x, y, image, screen, offset_x, offset_y, mass, friction_multiplier, image_size, False)
        
    def set_local_velocity(self, vel, ang_vel):
        self.lin_vel_x = vel * math.cos(-self._current_rotation * math.pi / 180 - math.pi / 2)
        self.lin_vel_y = vel * math.sin(-self._current_rotation * math.pi / 180 - math.pi / 2)
        self.ang_vel = ang_vel
        if abs(vel) -0.2 < 0:
            self.ang_vel = 0
    
    def apply_force_now(self, lin_force_x, lin_force_y, ang_force=0):
        if self.dynamic_model == False:
            raise Exception('Dynamic modeling is turned off for this model, so this function should not be called. Please, check your code')
            quit()
        
    def apply_force_now_local(self, lin_force_x, lin_force_y, ang_force=0):
        if self.dynamic_model == False:
            raise Exception('Dynamic modeling is turned off for this model, so this function should not be called. Please, check your code')
            quit()


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