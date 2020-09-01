import math
import numpy as np



def NormalizeAngle(angle): #input angle [-pi, pi] mapped to [0, 2* pi]
    if angle < 0.0:
        return (angle + 2 * math.pi)
    else:
        return angle

def GetAngle(p1,p2):
    angleFromXAxis = math.atan2(p2[1] - p1[1], p2[0] - p1[0]);   # where y = m * x + K # [-pi, pi]
    return angleFromXAxis                                        #Atan2 result is always between -pi and pi                           

def GetDirection(a, b, c):
    theta1 = GetAngle(a, b) 
    theta2 = GetAngle(b, c)
    delta = NormalizeAngle(theta2 - theta1)

    if (delta == 0.0):
        return 'straight'
    elif (delta == math.pi):
        return 'Backwards'
    elif (delta < math.pi):
        return 'left'
    else:
        return 'right'

def find_cte(wp1, wp2, curr_point, curr_yaw):

    desired_heading = NormalizeAngle(GetAngle(wp1,wp2))
    heading_offset  = desired_heading - curr_yaw

    


    return cte, heading_offset

def find_segment_orientation(x,y):
    segment_orientation = []    
    for i in range(0,len(x) - 1):
        theta = GetAngle([x[i], y[i]], [x[i+1], y[i+1]])
        segment_orientation +=[ NormalizeAngle(theta)]
    return segment_orientation


def find_segment_direction(x, y, theta):
    direction = []      
    for i in range(0, len(theta)):
        if (theta[i] == 0.0 or (theta[i] % math.pi) ==0 or (theta[i] % (math.pi/ 2)) == 0 ):
            direction +=['straight']    #check if straight line parallel or perpendicular    
        else:
            temp_direction = GetDirection([x[i-1], y[i-1]], [x[i], y[i]], [x[i+1], y[i+1]])
            direction +=[temp_direction]    #if a curve, estimate whether left or right        
    return direction


def find_segment_radius(direction):
    radius = []    
    for i in range(0,len(direction)):        
        if direction[i] =='straight':
            radius +=[0.0]
        if direction[i] =='left':
            radius +=[-tcr]
        if direction[i] =='right':
            radius +=[tcr]    
    return radius

def segment_path_data(x_pts, y_pts):
    segment_orientation = find_segment_orientation(x_pts, y_pts)
    segment_direction   = find_segment_direction(x_pts, y_pts, segment_orientation)
    segment_radius      = find_segment_radius(segment_direction)
    return segment_orientation, segment_direction, segment_radius


        
