
'''Bezier Curve without stuff for craig reynolds path following'''
class BezierCurve:

    def __init__(self, x0, y0, x1, y1, x2, y2, x3, y3, step):
        self.x0 = x0
        self.y0 = y0
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.x3 = x3
        self.y3 = y3
        self.step = step
        self.generate_curve()
        # self.graph()


    def generate_curve(self):
        x0 = self.x0
        y0 = self.y0
        x1 = self.x1
        y1 = self.y1
        x2 = self.x2
        y2 = self.y2
        x3 = self.x3
        y3 = self.y3
        step = self.step
        self.distance = 0
        # step is the number of points on the curve + 1 for t = 0,
        t = 0
        a = 0
        t_list = []
        x_list = []
        y_list = []
        angle = 0
        angle_list = []
        dist_list = []
        prev_x = self.x0
        prev_y = self.y0
        while t != 1:
            t = a/step
            x = (x0*(1-t)**3) + (3*x1*t*(1-t)**2) + (3*x2*(1-t)*t**2) + (x3*t**3)
            y = (y0*(1-t)**3) + (3*y1*t*(1-t)**2) + (3*y2*(1-t)*t**2) + (y3*t**3)
            delta_x = x-prev_x
            delta_y = y - prev_y
            angle = math.atan2(delta_x, delta_y)
            angle = math.degrees(angle)
            hypotenuse = NerdyMath.distance_formula(x, y, prev_x, prev_y)
            self.distance += hypotenuse
            dist_list.append(self.distance)
            t_list.append(t)
            x_list.append(x)
            y_list.append(y)
            angle_list.append(angle)
            a += 1
            prev_x = x
            prev_y = y
            # print(t)
            # print(self.distance)
            # print(delta_x, delta_y)
            # print(x, y, angle)
        self.t_list = t_list
        self.x_list = x_list
        self.y_list = y_list
        self.angle_list = angle_list
        self.dist_list = dist_list
        # print('done')
        # print(t_list)
        # print(x_list)
        # print(y_list)
        # print(angle_list)

    def get_last_x(self):
        return self.x_list[-1]

    def get_last_y(self):
        return self.y_list[-1]

    def make_csv(self, file_name):
        x0 = self.x0
        y0 = self.y0
        x1 = self.x1
        y1 = self.y1
        x2 = self.x2
        y2 = self.y2
        x3 = self.x3
        y3 = self.y3
        step = self.step
        file = open("BezierCurveTests/" + file_name + ".csv", "a")
        file.truncate(0)
        file.write('x0,y0,x1,y1,x2,y2,x3,y3\n')
        file.write(str(x0) + ',' + str(y0) + ',' + str(x1) + ',' + str(y1) + ',' + str(x2) + ',' + str(y2) + ',' + str(x3) + ',' + str(y3) + '\n')
        file.write('\n')
        file.write('t,x,y,angle\n')
        a = 0
        for t in self.t_list:
            x = self.x_list[a]
            y = self.y_list[a]
            angle = self.angle_list[a]
            file.write(str(t) + ',' + str(x) + ',' + str(y) + ',' + str(angle) + '\n')
            a += 1
        file.close()

    def graph(self):
        plt.plot(self.x_list, self.y_list)
        plt.xlabel('x')
        plt.ylabel('y')
        x_scale = max(self.x_list) * 1.25
        x_min = min(self.x_list) * 1.25
        y_scale = max(self.y_list) * 1.25
        y_min = min(self.y_list) * 1.25
        scale = NerdyMath.get_greater_value(x_scale, y_scale)
        min_scale = NerdyMath.get_greater_value(x_min, y_min)
        plt.axis([min_scale, scale, min_scale, scale])
        plt.show()

    def get_closest_point(self, x, y):
        a = 1
        index = 0
        min = NerdyMath.distance_formula(x, y, self.x_list[0], self.y_list[0])
        while a != len(self.x_list):
            dist = NerdyMath.distance_formula(x, y, self.x_list[a], self.y_list[a])
            if dist < min:
                min = dist
                index = a
            a += 1
        return index