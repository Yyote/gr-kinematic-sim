import math

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