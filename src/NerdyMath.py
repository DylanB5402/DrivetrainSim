import math

def distance_formula(x1, y1, x2, y2):
    return math.sqrt((x1-x2)**2 + (y1-y2)**2)

def get_greater_value(a, b):
    if a > b:
        return a
    elif a < b:
        return  b
    elif a == b:
        return a
