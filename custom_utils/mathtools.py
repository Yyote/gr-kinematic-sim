import math

def rotation_matrix(x_in, y_in, angle):
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


def normalise_in_range(lower_bound, upper_bound, var):
    range_ = upper_bound - lower_bound
    if var < lower_bound:
        var += range_
    elif var > upper_bound:
        var -= range_