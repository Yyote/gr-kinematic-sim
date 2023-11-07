import pygame
import math
from copy import copy
from custom_utils.mathtools import normalise_in_range, rotation_matrix

DEFAULT_IMAGE_SIZE = (50, 50)

class Sprite(pygame.sprite.Sprite):
    def __init__(self, x, y, image, screen):
        super().__init__()
        self.image = pygame.transform.scale(image, DEFAULT_IMAGE_SIZE)
        self._original_image = pygame.transform.scale(image, DEFAULT_IMAGE_SIZE)
        self.rect = self.image.get_rect()
        self.rect.x = x
        self.rect.y = y
        self.screen = screen
        self._current_rotation = 0

    def rotate(self, degrees):
        # orig_x = copy(self.rect.x)
        
        # w, h = self.image.get_size()
        # box = [pygame.math.Vector2(p) for p in [(0, 0), (w, 0), (w, -h), (0, -h)]]

        # box_rotate = [p.rotate(degrees) for p in box]

        # min_box = (min(box_rotate, key=lambda p: p[0])[0], min(box_rotate, key=lambda p: p[1])[1])
        # max_box = (max(box_rotate, key=lambda p: p[0])[0], max(box_rotate, key=lambda p: p[1])[1])
        
        # pos = [self.rect.x + w / 2, self.rect.y + h / 2]
        # origin = (pos[0] + min_box[0], pos[1] - max_box[1])
        
        self._current_rotation += degrees
        normalise_in_range(0, 360, self._current_rotation)
        self.image = pygame.transform.rotozoom(self._original_image, self._current_rotation, 1)
        self.rect = self.image.get_rect(center=self.rect.center)
    
    
    def move(self, x, y, degrees=0):
        self.rect.x += x
        self.rect.y += y
        self.rotate(degrees)
    
    def draw(self):
        self.screen.blit(self.image, self.rect)