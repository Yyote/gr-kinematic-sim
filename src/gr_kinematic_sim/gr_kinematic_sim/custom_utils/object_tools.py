from email.mime import image
from errno import EALREADY
import pygame
import pytmx
import math

import rclpy
import cv2
import numpy as np
from copy import copy

from rclpy.duration import Duration
from rclpy.time import Time

from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseStamped, TwistStamped, Twist, PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap

from gr_kinematic_sim.custom_utils.mathtools import normalise_in_range, sgn_wo_zero, rotation_matrix, EulerAngles, limit, make_non_zero
from gr_kinematic_sim.custom_utils.collisions import check_kinematic_collision_between_tilemap_and_rect, check_kinematic_collision_between_spritelis_and_rect, check_mask_collision_between_spritelis_and_rect, check_mask_collision_between_tilemap_and_sprite
from gr_kinematic_sim.custom_utils.gametools import tick_rate, pygame_surface_to_opencv_image


DEFAULT_IMAGE_SIZE = (50, 50)
SMALL_IMAGE_SIZE = (25, 25)
VERY_SMALL_IMAGE_SIZE = (12, 12)

WORLD_SCALE = 64

pkg_dir = f"{get_package_share_directory('gr_kinematic_sim')}/../../../../src/gr_kinematic_sim/"


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
        # if self.rect.x is None or self.rect.y is None:
        #     print(f"ERROR: self.rect.x = {self.rect.x}, self.rect.y = {self.rect.y}")
        self.rotate(degrees)
    
    def draw(self):
        
        # print(f"self.curr_offset_x = {self.curr_offset_x},\nself.curr_offset_y = {self.curr_offset_y}")
        # print(f"self.rect.x = {self.rect.x},\nself.rect.y = {self.rect.y}")
        
        rect_to_draw = copy(self.rect)
        rect_to_draw.x += self.curr_offset_x
        rect_to_draw.y += self.curr_offset_y
        
        # print(f"self.curr_offset_x = {self.curr_offset_x},\nself.curr_offset_y = {self.curr_offset_y}")
        # print(f"rect_to_draw.x = {rect_to_draw.x},\nrect_to_draw.y = {rect_to_draw.y}")
        
        rotated_image = pygame.transform.rotozoom(self._original_image, self._current_rotation, 1)
        
        rotated_sprite_image = pygame_surface_to_opencv_image(rotated_image)
        # rotated_sprite_image = np.ndarray()
        
        
        
        # rotated_sprite_image.
        # cv2.imwrite(f"~/rotated_sprite_image", rotated_sprite_image)
        
        self.screen.blit(rotated_image, rect_to_draw)
    
    def update_offset(self, x, y):
        self.curr_offset_x = x
        self.curr_offset_y = y
    
    def draw_mask(self):
        rect_to_draw = copy(self.rect)
        print(self.curr_offset_x)
        print(self.curr_offset_y)
        rect_to_draw.x += self.curr_offset_x
        rect_to_draw.y += self.curr_offset_y
        mask = pygame.mask.from_surface(self.image)
        self.screen.blit(mask.to_surface(), rect_to_draw)
    
    def get_mask(self):
        mask = pygame.mask.from_surface(self.image)
        return mask

class PhysicalObject(Sprite):
    def __init__(self, name, tilemap, robot_factory, x, y, image, screen, offset_x, offset_y, mass, friction_multiplier=0.95, image_size=DEFAULT_IMAGE_SIZE, dynamic_model=True):
        super().__init__(x, y, image, screen, offset_x, offset_y, image_size=image_size)
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
        self.robot_factory = robot_factory
    
    def kinematic_rect_collision_check(self, sprite):
        # spritelist_collided, sprite_collider = check_kinematic_collision_between_spritelis_and_rect(self.robot_factory.spritelist, sprite)
        # tilemap_collided, tile_collider = check_kinematic_collision_between_tilemap_and_rect(self.tilemap, sprite.rect)
        tilemap_collided, tile_collider = check_mask_collision_between_tilemap_and_sprite(self.tilemap, sprite)
        spritelist_collided, sprite_collider = check_mask_collision_between_spritelis_and_rect(self.robot_factory.spritelist, sprite)
        
        
        if tilemap_collided:
            return (True, tile_collider)
        if spritelist_collided:
            return (True, sprite_collider)
        return (False, None)
        
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
        
        local_vel_x, local_vel_y = rotation_matrix(vel_x, vel_y, -self._current_rotation * math.pi / 180.0)
        
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
        print(f"ROBOT NAME = {self.name}")
        if math.isnan(self.lin_vel_x) or math.isnan(self.lin_vel_y):
            self.lin_vel_x = 0
            self.lin_vel_y = 0
            self.ang_vel = 0
        if self.dynamic_model:
            self._friction()
        print(f"self.rect = {self.rect}")
        print(f"self.lin_vel_x = {self.lin_vel_x}")
        print(f"self.lin_vel_y = {self.lin_vel_y}")
        print(f"type(self.lin_vel_y) = {type(self.lin_vel_y)}")
        copied_rect = copy(self.rect)
        copied_rect.x += self.lin_vel_x
        copied_rect.y += self.lin_vel_y

        copied_sprite = copy(self)
        copied_sprite.rect = copied_rect
        copied_sprite.rotate(self.ang_vel)
        kinematic_collision, collide_rect = self.kinematic_rect_collision_check(copied_sprite)
        if not self.dynamic_model and kinematic_collision:
            dx = -self.rect.centerx + collide_rect.centerx
            dy = -self.rect.centery + collide_rect.centery
            dx = -self.rect.centerx + collide_rect.centerx
            dy = -self.rect.centery + collide_rect.centery
            # self.lin_vel_x = - limit(10 / (make_non_zero(dx)), 3)
            # self.lin_vel_y = - limit(10 / (make_non_zero(dy)), 3)
            self.lin_vel_x = 0
            self.lin_vel_y = 0
            self.ang_vel = 0
        self.rect.x += self.lin_vel_x
        self.rect.y += self.lin_vel_y
        self.rotate(self.ang_vel)


    def draw(self, offset_x, offset_y):
        self.update_offset(offset_x, offset_y)
        # self._move()
        super().draw()





class TileObj:
    def __init__(self, x, y, gid, real_gid, collider_rect, image):
        self.x = x
        self.y = y
        self.gid = gid
        self.real_gid = real_gid
        self.rect = collider_rect
        self.mask = pygame.mask.from_surface(image)
    
    def reveal(self):
        self.gid = self.real_gid
    
    def get_mask(self):
        return self.mask


class TiledMap():
    def __init__(self, map_path, node):
        self.gameMap = pytmx.load_pygame(map_path, pixelalpha=True)
        self.mapwidth = self.gameMap.tilewidth * self.gameMap.width
        self.mapheight = self.gameMap.tileheight * self.gameMap.height
        self.collider_list = []
        self.map_dict = {}
        self.node = node


    def _render(self, screen):
        for layer in self.gameMap.visible_layers:
            if isinstance(layer, pytmx.TiledTileLayer):
                for x, y, gid in layer:
                    tile = self.gameMap.get_tile_image_by_gid(gid)
                    if gid == 1:
                        self.collider_list.append(pygame.Rect(x * self.gameMap.tilewidth, y * self.gameMap.tileheight, self.gameMap.tilewidth, self.gameMap.tileheight))
                    if tile:
                        screen.blit(tile, (x * self.gameMap.tilewidth, y * self.gameMap.tileheight))
                        crect = pygame.Rect(x * self.gameMap.tilewidth, y * self.gameMap.tileheight, self.gameMap.tilewidth, self.gameMap.tileheight)
                        if x not in self.map_dict:
                            self.map_dict[x] = {}
                        if y not in self.map_dict[x]:
                            self.map_dict[x][y] = TileObj(x=x, y=y, gid=gid, real_gid=gid, collider_rect=crect, image=tile)


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


class FoggedMap(TiledMap):
    def __init__(self, map_path, node):
        super().__init__(map_path=map_path, node=node)
        self.counter = 0
        self.publisher = self.node.create_publisher(OccupancyGrid, 'OccupancyGrid_map', 100)
        
    def update_counter(self, c):
        self.counter = c
        
    def render(self, screen, offset_x, offset_y):
        for x in range(self.gameMap.width):
            for y in range(self.gameMap.height):
                tile_obj = self.map_dict[x][y]
                tile = self.gameMap.get_tile_image_by_gid(tile_obj.gid)
                if tile:
                    screen.blit(tile, (x * self.gameMap.tilewidth + offset_x, y * self.gameMap.tileheight + offset_y))
    
    def _render(self, screen):
        for layer in self.gameMap.visible_layers:
            if isinstance(layer, pytmx.TiledTileLayer):
                for x, y, gid in layer:
                    obstacle_gid = 6
                    tile = self.gameMap.get_tile_image_by_gid(obstacle_gid)
                    if gid == 1:
                        self.collider_list.append(pygame.Rect(x * self.gameMap.tilewidth, y * self.gameMap.tileheight, self.gameMap.tilewidth, self.gameMap.tileheight))
                    if tile:
                        screen.blit(tile, (x * self.gameMap.tilewidth, y * self.gameMap.tileheight))
                        crect = pygame.Rect(x * self.gameMap.tilewidth, y * self.gameMap.tileheight, self.gameMap.tilewidth, self.gameMap.tileheight)
                        if x not in self.map_dict:
                            self.map_dict[x] = {}
                        if y not in self.map_dict[x]:
                            self.map_dict[x][y] = TileObj(x=x, y=y, gid=obstacle_gid, real_gid=gid, collider_rect=crect, image=self.gameMap.get_tile_image_by_gid(gid))

    def unfog_map(self, points, pos, line_length_pxls):
        if self.counter % 10 != 0:
            return
        x0 = pos[0]
        y0 = pos[1]
        
        local_tiles = []
        for x in range(self.gameMap.width):
            for y in range(self.gameMap.height):
                dr = ((x * self.gameMap.tilewidth - x0) ** 2 + (y * self.gameMap.tileheight - y0) ** 2) ** 0.5
                if dr < line_length_pxls:
                    local_tiles.append((x,y))
        
        for i in range(len(points)):
            x1 = points[i][0]
            y1 = points[i][1]
            line = ((x0, y0), (x1, y1))
            for j in range(len(local_tiles)):
                if self.map_dict[local_tiles[j][0]][local_tiles[j][1]].rect.clipline(line):
                    self.map_dict[local_tiles[j][0]][local_tiles[j][1]].reveal()
        
    def gameMap_to_OpenCv(self):
        """Это функция из открытых тайлов делает картинку OpenCV

        Returns:
            img - np.ndarray - карта в формате изображения
        """
        img = np.ones(( self.gameMap.height, self.gameMap.width, 3), np.uint8)* 255
        for j in range(self.gameMap.width):
            for i in range(self.gameMap.height):
                #def switch(index):
                    #if index == 1:
                        #colour = red    
                colour = [0, 0, 0]
                if self.map_dict[j][i].gid == 1:
                    colour = [0,0,0]
                elif self.map_dict[j][i].gid == 2:
                    colour = [255,255,255]
                elif self.map_dict[j][i].gid == 3:
                    colour = [100,100,230] #red
                elif self.map_dict[j][i].gid == 4:
                    colour = [122,237,155] #green
                elif self.map_dict[j][i].gid == 5:
                    colour = [245,143,230] #purple
                else:
                    colour = [95,95,47] #Gray (Vlados colour)
                img[i][j] = colour    
        cv2.imwrite ("Image.png", img)
        return img
    
    def OpenCV_to_OccupancyGRID(self, img):
        """Переводит карту в формате изображения в формат OccupancyGrid

        Args:
            img (np.ndarray): Карта

        Returns:
            OccupancyGrid 
        """
        occupancyG = OccupancyGrid()
        angle = EulerAngles()
        prob_vec = [-1 for i in range(self.gameMap.height * self.gameMap.width)]
        #prob_vec = []
        occupancyG.header.frame_id = "map"
        occupancyG.header.stamp = self.node.get_clock().now().to_msg()
        occupancyG.info.height = self.gameMap.height
        occupancyG.info.width = self.gameMap.width
        occupancyG.info.origin.position.x = 0.0
        occupancyG.info.origin.position.y = 0.0
        occupancyG.info.origin.position.z = 0.0
        occupancyG.info.origin.orientation = angle.setRPY_of_quaternion(0, math.pi, math.pi/2)
        occupancyG.info.resolution = 0.5
        k = 0

        for j in range(self.gameMap.height):
            for i in range(self.gameMap.width):
                if img[j][i][0] == 0: 
                    prob_vec[k] = 100
                elif img[j][i][0] == 95: 
                    prob_vec[k] = -1
                else:
                    prob_vec[k] = 0
                k += 1
                
                # if img[j][i][0] == color_template: 
                #     prob_vec.append(0)
                # else:
                #     prob_vec.append(100)

        occupancyG.data = prob_vec
        self.publisher.publish(occupancyG)
        return occupancyG