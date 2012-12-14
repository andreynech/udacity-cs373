from math import *
import random

eps = 1.0e-10

def line_equation_ap(angle, (x1, y1)):
    """Calculates A, B and C coefficients of the line \
    equation in the form Ax+By+C=0"""
    
    # get second point on the line
    x2 = float(x1) + cos(angle)
    y2 = float(y1) + sin(angle)
    
    # return A, B and C coefficients
    return (y1 - y2, x2 - x1, x1*y2 - x2*y1)


def line_equation_pp((x1, y1), (x2, y2)):
    if x1 == x2:
        angle = pi/2
    else:
        angle = atan((y2-y1)/(x2-x1))
    return line_equation_ap(angle, (x1, y1))


def is_parallel((A1,B1,C1), (A2,B2,C2)):
    return (A1*B2 == A2*B1)


def intersection((A1,B1,C1), (A2,B2,C2)):
    return ((B1*C2 - B2*C1) / (A1*B2 - A2*B1), 
            (C1*A2 - C2*A1) / (A1*B2 - A2*B1))


def sign(x):
    return 1 if x >= 0 else -1


def point_inside_polygon(plan, (x,y,theta)):
    """Implementation of the ray casting algorithm to check if the point is inside the polygon"""
    count = 0
    ray = line_equation_ap(theta, (x,y))
    for (a, b) in plan:
        (ax, ay) = a
        (bx, by) = b
        wall = line_equation_pp(a, b)
        if not is_parallel(wall, ray):
            (xi, yi) = intersection(wall, ray)
            #print "Intersection candidate:", xi, yi
            if abs(xi) < eps:
                xi = 0.0
            if abs(yi) < eps:
                yi = 0.0
            if xi > min(ax,bx)-eps and xi < max(ax,bx)+eps and yi > min(ay,by)-eps and yi < max(ay,by)+eps:
                if sign(xi-x) == sign(cos(theta)) and sign(yi-y) == sign(sin(theta)):
                    count += 1
    # IN if count is odd OUT if count is even
    return count % 2 is not 0


def measurements(plan, (x, y, theta)):
    mes = [0]*4
    coeff_fwd = line_equation_ap(theta, (x,y))
    coeff_side = line_equation_ap(theta+pi/2, (x,y))
    #(A,B,C) = coeff_side
    #print "k =", -(A/B), "b =", -(C/B)
    
    for (a, b) in plan:
        (ax, ay) = a
        (bx, by) = b
        wall = line_equation_pp(a, b)
        #print "Checking wall:", (a,b)
        #(A,B,C) = wall
        #k = -(A/B)
        #bb = -(C/B)
        #print "Wall:", wall, k, bb

        # Checking forward/backward direction
        if not is_parallel(wall, coeff_fwd):
            (xi, yi) = intersection(wall, coeff_fwd)
            #print "Intersection candidate:", xi, yi
            if abs(xi) < eps:
                xi = 0.0
            if abs(yi) < eps:
                yi = 0.0
            if xi > min(ax,bx)-eps and xi < max(ax,bx)+eps and yi > min(ay,by)-eps and yi < max(ay,by)+eps:
                #print "Found intersection:", xi, yi
                dist = sqrt((xi-x)*(xi-x) + (yi-y)*(yi-y))
                #print "Distance:", dist, dist*cos(theta), dist*sin(theta)
                #print "Signs:", sign(xi-x), sign(cos(theta)), sign(yi-y), sign(sin(theta))
                if sign(xi-x) == sign(cos(theta)) and sign(yi-y) == sign(sin(theta)):
                    if mes[0] == 0 or mes[0] > dist:
                        #print "assigning to front sonar"
                        mes[0] = dist
                elif mes[2] == 0 or mes[2] > dist:
                    #print "assigning to rear sonar"
                    mes[2] = dist

        # Checking left/right direction
        if not is_parallel(wall, coeff_side):
            (xi, yi) = intersection(wall, coeff_side)
            #print "Intersection candidate:", xi, yi
            if abs(xi) < eps:
                xi = 0.0
            if abs(yi) < eps:
                yi = 0.0
            if xi > min(ax,bx)-eps and xi < max(ax,bx)+eps and yi > min(ay,by)-eps and yi < max(ay,by)+eps:
                #print "Found intersection:", xi, yi
                dist = sqrt((xi-x)*(xi-x) + (yi-y)*(yi-y))
                #print "Distance:", dist, (dist*cos(theta), dist*sin(theta))
                #print "Signs:", sign(xi-x), sign(cos(theta)), sign(yi-y), sign(sin(theta))
                if sign(xi-x) == sign(cos(theta+pi/2)) and sign(yi-y) == sign(sin(theta+pi/2)):
                    if mes[1] == 0 or mes[1] > dist:
                        #print "assigning to right sonar"
                        mes[1] = dist
                elif mes[3] == 0 or mes[3] > dist:
                    #print "assigning to left sonar"
                    mes[3] = dist

    return mes


if __name__ == '__main__':
    import csv

    with open('plan.dat', 'r') as planfile:
        planreader = csv.reader(planfile, delimiter='\t')
        b = None
        plan_list = []
        for (x, y) in planreader:
            if b is not None:
                a = b
                b = (float(x), float(y))
                plan_list.append((a,b))
            else:
                b = (float(x), float(y))

                robot_pos = (-0.06, 1.5, 0.0*pi/8)
                # robot_pos = (0.5, 0.5, pi/3)
                # robot_pos = (2.0, 1.0, pi/4)
                mes = measurements(plan_list, robot_pos)

                print robot_pos[0], robot_pos[1]
                print robot_pos[0] + mes[0] * cos(robot_pos[2]), robot_pos[1] + mes[0] * sin(robot_pos[2])
                print
                print robot_pos[0], robot_pos[1]
                print robot_pos[0] - mes[2] * cos(robot_pos[2]), robot_pos[1] - mes[2] * sin(robot_pos[2])
                print
                print robot_pos[0], robot_pos[1]
                print robot_pos[0] + mes[1] * cos(robot_pos[2]+pi/2), robot_pos[1] + mes[1] * sin(robot_pos[2]+pi/2)
                print
                print robot_pos[0], robot_pos[1]
                print robot_pos[0] - mes[3] * cos(robot_pos[2]+pi/2), robot_pos[1] - mes[3] * sin(robot_pos[2]+pi/2)
                print
