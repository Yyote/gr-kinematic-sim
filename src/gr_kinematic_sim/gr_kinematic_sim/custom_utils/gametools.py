import pygame as pg
import math
import rclpy
from geometry_msgs.msg import TwistStamped

tick_rate = 30


def handle_keypresses(sprite):
    """
    Brief: 
        Работает только с классом custom_utils.object_tools.Sprite
    """
    keys = pg.key.get_pressed()
    if keys[pg.K_LEFT]:
        sprite.move(-1, 0)
    if keys[pg.K_RIGHT]:
        sprite.move(1, 0)
    if keys[pg.K_UP]:
        sprite.move(0, -1)
    if keys[pg.K_DOWN]:
        sprite.move(0, 1)
    if keys[pg.K_z]:
        sprite.rotate(1)
    if keys[pg.K_x]:
        sprite.rotate(-1)
    if keys[pg.K_ESCAPE]:
        pg.quit()


def handle_keypresses_through_force(sprite):
    """
    Brief: 
        Работает только с классом custom_utils.object_tools.Sprite
    """
    keys = pg.key.get_pressed()
    if keys[pg.K_LEFT]:
        sprite.apply_force_now_local(-0.05, 0)
    if keys[pg.K_RIGHT]:
        sprite.apply_force_now_local(0.05, 0)
    if keys[pg.K_UP]:
        sprite.apply_force_now_local(0, -0.05)
    if keys[pg.K_DOWN]:
        sprite.apply_force_now_local(0, 0.05)
    if keys[pg.K_z]:
        sprite.apply_force_now_local(0, 0, 0.1)
    if keys[pg.K_x]:
        sprite.apply_force_now_local(0, 0, -0.1)


def handle_keypresses_through_velocity(sprite, node):
    """
    Brief: 
        Работает только с классом custom_utils.object_tools.Sprite
    """
    msg = TwistStamped()
    keys = pg.key.get_pressed()
    vel = 0
    ang_vel = 0
    
    if keys[pg.K_UP]:
        vel += 2
    if keys[pg.K_LEFT]:
        ang_vel -= 159
    if keys[pg.K_RIGHT]:
        ang_vel += 159
    if keys[pg.K_DOWN]:
        vel -= 2

    
    # sprite.set_local_velocity(vel, ang_vel)
    msg.header.frame_id = 'robot1/base_link'
    msg.twist.linear.x = float(vel)
    msg.twist.angular.z = float(ang_vel * math.pi / 180)
    if abs(vel) > 1 or abs(ang_vel) > 1:
        node.cmd_vel_pub.publish(msg)
        # pass



def handle_keypresses_through_velocity_omni(sprite, node):
    """
    Brief: 
        Работает только с классом custom_utils.object_tools.Sprite
    """
    msg = TwistStamped()
    keys = pg.key.get_pressed()
    vel = 0
    ang_vel = 0
    
    if keys[pg.K_UP]:
        vel += 2
    if keys[pg.K_DOWN]:
        vel -= 2
    if keys[pg.K_RIGHT]:
        ang_vel += -2
    if keys[pg.K_LEFT]:
        ang_vel += 2
    
    # sprite.set_local_velocity(vel, ang_vel)
    msg.header.frame_id = 'robot1/base_link'
    msg.twist.linear.x = float(vel)
    msg.twist.linear.y = float(ang_vel)
    if abs(vel) > 1 or abs(ang_vel) > 1:
        # node.cmd_vel_pub.publish(msg)
        pass




def handle_key_events():
    keys = pg.key.get_pressed()
    if keys[pg.K_ESCAPE]:
        pg.quit()


def handle_offset_change(offset_x, offset_y):
    keys = pg.key.get_pressed()
    if keys[pg.K_w]:
        offset_y += 5
    if keys[pg.K_s]:
        offset_y -= 5
    if keys[pg.K_a]:
        offset_x += 5
    if keys[pg.K_d]:
        offset_x -= 5
    if keys[pg.K_BACKSPACE]:
        offset_x = 0
        offset_y = 0
    return offset_x, offset_y


def draw_every_sprite_in_list(list_, offset_x, offset_y):
    for i in range(len(list_)):
        list_[i].update_offset(offset_x, offset_y)
        list_[i].draw(offset_x, offset_y)
        

def scroll_screen_with_mouse(screen_width, screen_height, offset_x, offset_y):
    mx, my = pg.mouse.get_pos()
    
    if mx < screen_width / 10 and mx > 0:
        offset_x += 5
    if mx > screen_width * 9 / 10 and mx < screen_width:
        offset_x -= 5
    if my < screen_height / 10 and my > 0:
        offset_y += 5
    if my > screen_height * 9 / 10 and my < screen_height:
        offset_y -= 5
        
    return offset_x, offset_y