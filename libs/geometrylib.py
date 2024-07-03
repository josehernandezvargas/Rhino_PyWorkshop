import math
import rhinoscriptsyntax as rs


def lerp(a, b, t):
    """"
    Creates a linear interpolation between two values
    Returns the value for a parameter t in the range a - b"""
    return((1 - t) * a + b * t)


def invlerp(a, b, v):
    """
    Returns the parameter t for a given value v in the range a - b"""
    return((v - a) / (b - a))


def remap(imin, imax, omin, omax, v):
    """ remaps a parameter v from input domain imin to imax 
    to output domain omin to omax
    """
    t = invlerp(imin, imax, v)
    return(lerp(omin, omax, t))


def minmaxcaplist(lo, hi, t):
    """Clamps a value between two limits"""
    results = []
    for item in t:
        newitem = min(hi, max(lo, t))
        results.append(newitem)
    return(results)


def minmaxcap(lo, hi, t):
    """Clamps a list between to limits"""
    newitem = min(hi, max(lo, t))
    return(newitem)


def flattenlist(list):
    """flattens a nested list into a simple list"""
    flatlist = []
    for i in list:
        for j in i:
            point = rs.CreatePoint(j)
            flatlist.append(point)
    return(flatlist)


def shortestangle(a1, a2):
    """ 
    Returns the lowest angle between two points
    the result is negative for angles > 180
    Arguments
        a1: first angle
        a2: second angle
    Returns
        angle in degrees
    """
    angle = ((a2 % 360) - (a1 % 360) + 360) % 360
    if angle > 180:
        angle = 360 - angle
        return -angle
    return (angle)


def absoluteangle(pt1, pt2):
    """
    calculates the absolute angle for two points to the X axis
    Arguments
        pt1: first point
        pt2: second point
    Returns
        the absolute angle in degrees
    """
    dx = pt1[0] - pt2[0]
    dy = pt1[1] - pt2[1]
    a = math.atan2(dy, dx) * (180.0 / math.pi)
    a = -a + 90
    a = a % 360
    if a == 360:
        a = 0
    angle = a
    # angle = rs.Angle(pt,prevpt)
    if angle:
        return angle
    else:
        return 0

def timestamp(format=0):
    """
    Returns a timestamp without having to import time in the header
    """
    import time 
    timestamp = time.strftime("%Y%m%d")  # adds a timestamp with the date
    hourstamp = " at " + time.strftime("%X")  # a timestamp with the hour
    if format:
        if format == 1:
            return "_" + time.strftime("%Y%m%d")
        elif format == 2:
            return timestamp + hourstamp
        elif format == 3:
            return time.strftime("%X")
    else:
        return timestamp

def plane_to_abc(origin, x_vector, y_vector):
    # Calculate the Z vector as the cross product of X and Y vectors
    z_vector = (x_vector[1] * y_vector[2] - x_vector[2] * y_vector[1],
                x_vector[2] * y_vector[0] - x_vector[0] * y_vector[2],
                x_vector[0] * y_vector[1] - x_vector[1] * y_vector[0])

    # Normalize the Z vector
    z_length = math.sqrt(z_vector[0] ** 2 + z_vector[1] ** 2 + z_vector[2] ** 2)
    z_vector = (z_vector[0] / z_length, z_vector[1] / z_length, z_vector[2] / z_length)

    # Calculate the rotation matrix
    rotation_matrix = [[x_vector[0], y_vector[0], z_vector[0]],
                       [x_vector[1], y_vector[1], z_vector[1]],
                       [x_vector[2], y_vector[2], z_vector[2]]]

    # Extract the Euler angles
    roll = math.atan2(rotation_matrix[2][1], rotation_matrix[2][2])
    pitch = math.atan2(-rotation_matrix[2][0], math.sqrt(rotation_matrix[2][1] ** 2 + rotation_matrix[2][2] ** 2))
    yaw = math.atan2(rotation_matrix[1][0], rotation_matrix[0][0])

    # Convert angles to degrees
    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)

    # Adjust angle ranges
    roll = roll % 360
    pitch = pitch % 360
    yaw = yaw % 360

    if roll > 185:
        roll -= 360
    if pitch > 135:
        pitch -= 360
    if yaw > 350:
        yaw -= 360

    return yaw, pitch, roll