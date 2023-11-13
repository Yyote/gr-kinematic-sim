import math
from geometry_msgs.msg import PoseStamped, Quaternion
from tf_transformations import quaternion_from_euler, euler_from_quaternion

def rotation_matrix(x_in, y_in, angle_rad):
    """Матрица поворота

    Args:
        x_in (float): координата вектора в метрах
        y_in (float): координата вектора в метрах
        angle_rad (float): угол поворота в радианах

    Returns:
        x, y - координаты повернутого вектора
    """
    x = x_in * math.cos(angle_rad) - y_in * math.sin(angle_rad)
    y = x_in * math.sin(angle_rad) + y_in * math.cos(angle_rad)
    
    return x, y


def normalise_in_range(lower_bound, upper_bound, var):
    range_ = upper_bound - lower_bound
    if var < lower_bound:
        var += range_
    elif var > upper_bound:
        var -= range_
    return var


def sgn(num):
    if num < 0:
        return -1
    else:
        return 1


def sgn_wo_zero(num):
    if num < 0:
        return -1
    elif num > 0:
        return 1
    else:
        return 0
    

def make_non_zero(num):
    if num == 0:
        num = 0.1
    return num


def limit(num, max_limit):
    absnum = abs(num)
    if absnum > max_limit:
        absnum = max_limit
    
    return absnum * sgn(num)

class EulerAngles:
    def __init__(self):
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
    
    
    def getRPY_from_quaternion(self, quat):
        """Устанавливает внутренние значения RPY равным вращению кватерниона и 
        возвращает эти RPY значения 

        Args:
            quat (tf.Quaternion): Кватернион вращения

        Returns:
            _roll, _pitch, _yaw (tuple): Эйлеровы углы кватерниона
        """
        quaternion = (
            quat.x,
            quat.y,
            quat.z,
            quat.w
        )
        euler = euler_from_quaternion(quaternion)
        self._roll = euler[0]
        self._pitch = euler[1]
        self._yaw = euler[2]
        
        return self._roll, self._pitch, self._yaw
    
    
    def setRPY_of_quaternion(self, roll, pitch, yaw):
        quat = Quaternion()
        
        q = quaternion_from_euler(roll, pitch, yaw)
        quat.x = q[0]
        quat.y = q[1]
        quat.z = q[2]
        quat.w = q[3]
        return quat
    
    
    def rotate_2d_vector_by_angle(self, x_in, y_in, angle):
        """Матрица поворота

        Args:
            x_in (float): координата вектора в метрах
            y_in (float): координата вектора в метрах
            angle (float): угол поворота в радианах

        Returns:
            x, y - координаты повернутого вектора
        """
        x = x_in * math.cos(angle) - y_in * math.sin(angle)
        y = x_in * math.sin(angle) + y_in * math.cos(angle)
        
        return x, y
