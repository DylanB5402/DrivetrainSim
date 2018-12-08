import math
import NerdyMath
import matplotlib.pyplot as plt

class Pose2D:

    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = math.radians(theta)

    
class Arc2D:

    def __init__(self, p0 : Pose2D, p1: Pose2D, p2: Pose2D):
        self.p0 = p0
        self.p1 = p1
        self.p2 = p2
        self.dx = p2.x - p0.x
        self.dy = p2.y - p0.y
        self.dtheta = p2.theta - p0.theta
        self.calculate()

    def calculate(self):
        if (self.p1.x == self.p2.x or self.p2.x == self.p0.x or self.p1.y == self.p2.y or self.p2.y == self.p0.y):
            self.length = NerdyMath.distance_formula(self.p0.x, self.p0.y, self.p2.x, self.p2.y)
            self.curvature = 0
        else:
            m1 = (self.p1.y - self.p2.y)/(self.p1.x - self.p2.x)
            m2 = (self.p2.y - self.p0.y)/(self.p2.x - self.p0.x)
            # print(m2, m1)
            if (-m1**-1)+(m2**-1) == 0:
                self.length = NerdyMath.distance_formula(self.p0.x, self.p0.y, self.p2.x, self.p2.y)
                self.curvature = 0 
            else:
                h = ((-m1**-1) * self.avg(self.p1.x, self.p2.x) - self.avg(self.p1.y, self.p2.y) + (m2**-1) * self.avg(self.p2.x, self.p0.x) + self.avg(self.p2.y, self.p0.y))/((-m1**-1)+(m2**-1))
                k = (-m2**-1) * (h - self.avg(self.p2.x, self.p0.x)) + self.avg(self.p2.y, self.p0.y)
                radius = math.sqrt((self.p0.x -h)**2 + (self.p0.y - k)**2)
                self.length = abs(self.dtheta) * radius
                self.curvature = radius**-1
       


    def avg(self, a, b):
        return (a + b)/2

    # high key stolen from poofs
class QuinticHermiteSpline:


    def __init__(self, p0: Pose2D, p1: Pose2D):
        self.p0 = p0
        self.p1 = p1

        scale = NerdyMath.distance_formula(p0.x, p0.y, p1.x, p1.y) * 1.2
        self.x0 = p0.x
        self.x1 = p1.x
        self.dx0 = math.cos(p0.theta) * scale
        self.dx1 = math.cos(p1.theta) * scale
        self.ddx0 = 0
        self.ddx1 = 0

        self.y0 = p0.y
        self.y1 = p1.y
        self.dy0 = math.sin(p0.theta) * scale
        self.dy1 = math.sin(p1.theta) * scale
        self.ddy0 = 0
        self.ddy1 = 0

        self.max_dx = 3/12 
        self.max_dy = 3/12
        # radians
        self.max_dtheta = 0.15

        self.max_arc_length = 0.5
        self.max_dcurvature = 0.1
        self.arc_list = []
        self.compute()

    def compute(self):
        self.ax = -6 * self.x0 - 3 * self.dx0 - 0.5 * self.ddx0 + 0.5 * self.ddx1 - 3 * self.dx1 + 6 * self.x1
        self.bx = 15 * self.x0 + 8 * self.dx0 + 1.5 * self.ddx0 - self.ddx1 + 7 * self.dx1 - 15 * self.x1;
        self.cx = -10 * self.x0 - 6 * self.dx0 - 1.5 * self.ddx0 + 0.5 * self.ddx1 - 4 * self.dx1 + 10 * self.x1;
        self.dx = 0.5 * self.ddx0;
        self.ex = self.dx0;
        self.fx = self.x0;

        self.ay = -6 * self.y0 - 3 * self.dy0 - 0.5 * self.ddy0 + 0.5 * self.ddy1 - 3 * self.dy1 + 6 * self.y1;
        self.by = 15 * self.y0 + 8 * self.dy0 + 1.5 * self.ddy0 - self.ddy1 + 7 * self.dy1 - 15 * self.y1;
        self.cy = -10 * self.y0 - 6 * self.dy0 - 1.5 * self.ddy0 + 0.5 * self.ddy1 - 4 * self.dy1 + 10 * self.y1;
        self.dy = 0.5 * self.ddy0;
        self.ey = self.dy0;
        self.fy = self.y0;

        t = 0
        a = 0
        self.x_list = []
        self.y_list = []
        while t != 1:
            t = a/200
            self.x_list.append(self.get_pose(t).x)
            self.y_list.append(self.get_pose(t).y)
            a += 1

    def get_pose(self, t):
        x = self.ax * t**5 + self.bx * t **4 + self.cx * t **3 + self.dx * t ** 2 + self.ex * t + self.fx
        y = self.ay * t**5 + self.by * t **4 + self.cy * t **3 + self.dy * t ** 2 + self.ey * t + self.fy
        dx = 5 * self.ax * t * t * t * t + 4 * self.bx * t * t * t + 3 * self.cx * t * t + 2 * self.dx * t + self.ex
        dy = 5 * self.ay * t * t * t * t + 4 * self.by * t * t * t + 3 * self.cy * t * t + 2 * self.dy * t + self.ey
        return Pose2D(x, y, math.atan2(dx, dy))
    
    def graph(self):
        plt.plot(self.x_list, self.y_list)
        plt.xlabel('x')
        plt.ylabel('y')
        plt.show()

    # def arc_parameterize(self, t0, t1):
    #     p0 = self.get_pose(t0)
    #     p1 = self.get_pose((t0+t1)/2)
    #     p2 = self.get_pose(t1)
    #     arc = Arc2D(p0, p1, p2)
    #     if arc.dx > self.max_dx or arc.dy > self.max_dy or arc.dtheta > self.max_dtheta:
    #         self.arc_parameterize(t0, ((t0 + t1)/2))
    #     else if :
    #         self.arc_list.append(arc)
    #         self.arc_parameterize(t1, 1)

    def arc_parameterize(self):
        t0 = 0
        t1 = 0.5
        i = 0
        arc = Arc2D(self.get_pose(t0), self.get_pose((t0 + t1)/2), self.get_pose(t1))
        last_curvature = 0
        delta_curvature = 0
        self.spline_length = 0
        # while arc.p2 != self.get_pose(1) or arc.dx > self.max_dx or arc.dy > self.max_dy or arc.dtheta > self.max_dtheta:
        while i < 100:
        # print(arc.p2.x !=self.get_pose(1).x)
        # print(arc.p2.y != self.get_pose(1).y)
        # print(arc.length > self.max_arc_length)
        # print(abs(delta_curvature) > self.max_dcurvature)
        # while arc.p2.x != self.get_pose(1).x or arc.p2.y != self.get_pose(1).y or arc.length > self.max_arc_length or abs(delta_curvature) > self.max_dcurvature:
            if t1 > 1:
                t1 = 1
            p0 = self.get_pose(t0)
            p1 = self.get_pose((t1 + t0)/2)
            p2 = self.get_pose(t1)
            arc = Arc2D(p0, p1, p2)
            delta_curvature = arc.curvature - last_curvature
            # print(arc.length)
            if arc.length < self.max_arc_length and abs(delta_curvature) < self.max_dcurvature:
                self.arc_list.append(arc)
                t0 = t1
                t1 = 1
                # t1 = 1
                last_curvature = arc.curvature
                print(254)
                self.spline_length += arc.length
            else:
                t1 = t1/2
                print(1678)
            
            # print(len(self.arc_list))
            # print(t0, t1)
            # print(i, t1)
            # print(arc.dx, arc.dy, arc.dtheta)
            # print(len(self.arc_list), arc.p2.y)
            i += 1
            print(i)


        
        
spline = QuinticHermiteSpline(Pose2D(0, 0, 45), Pose2D(10, 10, 45))
# spline.graph()
spline.arc_parameterize()
print('----------------')
print(len(spline.arc_list))
for arc in spline.arc_list:
    print(arc.length)
    a += arc.length
print(spline.spline_length)

# t = 0
# i = 0
# while t != 1:
#     t = i/100
#     print(spline.get_pose(t).theta)
#     i+= 1