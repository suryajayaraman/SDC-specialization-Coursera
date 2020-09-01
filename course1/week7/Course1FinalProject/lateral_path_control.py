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

def lateral_controller(wp1, wp2, curr_pt, curr_head):
    #find line angle bw next two waypoints
    #steer = +ve RHS, orientation increases CW
    #assume des_head = 0; curr_head = -ve( turned left); err = -ve; correction = +ve
##    des_head = NormalizeAngle(GetAngle(wp1,wp2))
##    head_offset = NormalizeAngle(curr_head) - des_head

    des_head    = GetAngle(wp1,wp2)
    head_offset = curr_head - des_head
    
##    print(math.degrees(head_offset))
    
    a = wp1[1]-wp2[1]
    b = wp1[0]-wp2[0]
    c = (wp1[0]-wp2[0])*wp1[1] + (wp2[1]-wp1[1])*wp1[0]
##    a = y1-y2,
##    b = x2-x1,
##    c = (x1-x2)*y1 + (y2-y1)*x1
    #perpendicular distance from the line connecting next two points
    cte_mag = abs(a*curr_pt[0] + b*curr_pt[1] + c) / math.sqrt(a**2 + b**2)
    cte_mag = math.sqrt( (curr_pt[0] - wp1[0])**2 + (curr_pt[1] - wp1[1])**2 )
    
    direction = (wp2[0]-wp1[0])*(curr_pt[1]-wp1[1]) - (curr_pt[0]-wp1[0]) *(wp2[1] - wp1[1])
    if direction > 0:
        cte_mag *= -1.0
    elif direction < 0:
        cte_mag *= 1.0
    elif direction ==0:
        cte_mag = 0.0
        
    return cte_mag, head_offset


def compute_front_axle(centre, yaw, lf):
    frontx = centre[0] + lf * np.cos(yaw)
    fronty = centre[1] + lf * np.sin(yaw)
    return frontx, fronty
