#reference, part of the code is in https://www.cnblogs.com/clemente/p/9543106.html
import matplotlib.pyplot as plt
import random
import math
import copy
car_radius = 7
start_x = 0
start_y = 0

show_animation = True

#RRT node
class Node(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.theta = 0

#The conner of the rectangle obstagle
class Conner_rectangle(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y


#Class for RRT Planning
class RRT(object):
    def __init__(self, start, goal, obstacle_list, rand_area):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:random sampling Area [min,max]

        """
        self.start = Node(start[0], start[1])
        self.end = Node(goal[0], goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expandDis = 3
        self.goalSampleRate = 0.05  # 0.05
        self.maxIter = 500
        self.obstacleList = obstacle_list
        self.nodeList = [self.start]

    #return random node
    def random_node(self):
        node_x = random.uniform(self.min_rand, self.max_rand)
        node_y = random.uniform(self.min_rand, self.max_rand)
        node = [node_x, node_y]

        return node

    @staticmethod
    def get_nearest_list_index(node_list, rnd):
        """
        :param node_list:
        :param rnd:
        :return:
        """       
        d_list = [math.sqrt((node.x - rnd[0]) ** 2 + (node.y - rnd[1]) ** 2)+math.fabs(math.atan2(rnd[1] - node.y, rnd[0] - node.x)) for node in node_list]
        min_index = d_list.index(min(d_list))
        return min_index


    
    #check collion between car and obstacle
    @staticmethod
    def collision_check(nearest_node, new_node, obstacle_list):
        #no_collion = 1
        for (ox, oy, size_x, size_y) in obstacle_list:
             no_collion = 1
             top_left_conner = Conner_rectangle(ox,oy+size_y)
             top_right_conner = Conner_rectangle(ox+size_x,oy+size_y)
             lower_left_conner = Conner_rectangle(ox,oy)
             lower_right_conner = Conner_rectangle(ox+size_x,oy)
             #determine if the origin of the circle of the car is in the rectangle
             if new_node.x>=ox and new_node.x<=ox+size_x and new_node.y >=oy and new_node.y<=oy+size_y:
                 no_collion=0
                 return no_collion #collion 0
             #the else if statements check if the car circle intersects the rectangle obstacles
             #we use the position of the origin of the circle for different situation
             #car's origin on the north-west side of the obstacle
             elif new_node.x<top_left_conner.x and new_node.y>top_left_conner.y:
                 d=math.sqrt((new_node.x-top_left_conner.x)*(new_node.x-top_left_conner.x)+(new_node.y-top_left_conner.y)*(new_node.y-top_left_conner.y))
                 if(d<car_radius):
                     no_collion=0
                     return no_collion #collion 0
             #car's origin on the north_east side of the obstacle
             elif new_node.x>top_right_conner.x and new_node.y>top_right_conner.y:
                 d=math.sqrt((new_node.x-top_right_conner.x)*(new_node.x-top_right_conner.x)+(new_node.y-top_right_conner.y)*(new_node.y-top_right_conner.y))
                 if(d<car_radius):
                     no_collion=0
                     return no_collion #collion 0
             #car's origin on the north of the obstacle
             elif new_node.x>top_left_conner.x and new_node.x<top_right_conner.x and new_node.y>top_left_conner.y:
                 d=new_node.y-top_left_conner.y
                 if(d<car_radius):
                     no_collion=0
                     return no_collion #collion 0
             #car's origin on the south of the obstacle
             elif new_node.x<lower_right_conner.x and new_node.x>lower_left_conner.x and new_node.y<lower_right_conner.y:
                 d=lower_right_conner.y-new_node.y
                 if(d<car_radius):
                     no_collion=0
                     return no_collion #collion 0
             #car's origin on the south-west of the obstacle
             elif new_node.x<lower_left_conner.x and new_node.y<lower_left_conner.y:
                 d=math.sqrt((new_node.x-lower_left_conner.x)*(new_node.x-lower_left_conner.x)+(new_node.y-lower_left_conner.y)*(new_node.y-lower_left_conner.y))
                 if(d<car_radius):
                     no_collion=0
                     return no_collion #collion 0
             #car's origin on the south-east of the obstacle
             elif new_node.x>lower_right_conner.x and new_node.y<lower_right_conner.y:
                 d=math.sqrt((new_node.x-lower_right_conner.x)*(new_node.x-lower_right_conner.x)+(new_node.y-lower_right_conner.y)*(new_node.y-lower_right_conner.y))
                 if(d<car_radius):
                     no_collion=0
                     return no_collion #collion 0
             #car's origin on the east of the obstacle
             elif new_node.x>lower_right_conner.x and new_node.y>lower_right_conner.y and new_node.y<top_left_conner.y:
                 d=new_node.x-lower_right_conner.x
                 if(d<car_radius):
                     no_collion=0
                     return no_collion #collion 0
             #car's origin on the west of the obstacle
             elif new_node.x<lower_left_conner.x and new_node.y>lower_left_conner.y and new_node.y<top_left_conner.y:
                 d=lower_left_conner.x-new_node.x
                 if(d<car_radius):
                     no_collion=0
                     return no_collion #collion 0
                 #determine if nearest_node and new_node are on the same side
                 #then check if the car's road will intersect with the obstacles
             else:
                 #slope of the line from nearest_node to new_node
                 k=(nearest_node.y-new_node.y)/(nearest_node.x-new_node.x)
                 #if nearest_node and new_node are on the same side
                 if ((nearest_node.x <ox and new_node.x < ox )or (nearest_node.x > ox+size_x and new_node.x > ox+size_x )or (nearest_node.y<oy and new_node.y <oy )or (nearest_node.y > oy+size_y and new_node.y > oy+size_y)):                     
                     #nearest_node and new_node are on the left side                    
                     if new_node.x >nearest_node.x and nearest_node.x <ox:
                         if k>0:
                             d = calculate_d(top_left_conner.x,top_left_conner.y,new_node.x,new_node.y,nearest_node.x,nearest_node.y)
                             if d < car_radius:
                                 no_collion = 0
                                 return no_collion                     
                         elif k<0:
                             d = calculate_d(lower_left_conner.x,lower_left_conner.y,new_node.x,new_node.y,nearest_node.x,nearest_node.y)
                             if d < car_radius:
                                 no_collion = 0
                                 return no_collion                             
                     #nearest_node and new_node are on the right side
                     elif new_node.x <nearest_node.x and new_node.x > ox+size_x:
                        if k<0:
                             d = calculate_d(top_right_conner.x,top_right_conner.y,new_node.x,new_node.y,nearest_node.x,nearest_node.y)
                             if d < car_radius:
                                 no_collion = 0
                                 return no_collion                     
                        elif k>0:
                             d = calculate_d(lower_right_conner.x,lower_right_conner.y,new_node.x,new_node.y,nearest_node.x,nearest_node.y)
                             if d < car_radius:
                                 no_collion = 0
                                 return no_collion
                     #nearest_node and new_node are on the top side
                     elif new_node.y<nearest_node.y and nearest_node.y > oy+size_y:
                         if k>0:
                             d = calculate_d(top_left_conner.x,top_left_conner.y,new_node.x,new_node.y,nearest_node.x,nearest_node.y)
                             if d < car_radius:
                                 no_collion = 0
                                 return no_collion                     
                         elif k<0:
                             d = calculate_d(top_right_conner.x,top_right_conner.y,new_node.x,new_node.y,nearest_node.x,nearest_node.y)
                             if d < car_radius:
                                 no_collion = 0
                                 return no_collion
                     #nearest_node and new_node are on the bottom side
                     elif new_node.y >  nearest_node.y and nearest_node.y< oy:
                         if k>0:
                             d = calculate_d(lower_right_conner.x,lower_right_conner.y,new_node.x,new_node.y,nearest_node.x,nearest_node.y)
                             if d < car_radius:
                                 no_collion = 0
                                 return no_collion                     
                         elif k<0:
                             d = calculate_d(lower_left_conner.x,lower_left_conner.y,new_node.x,new_node.y,nearest_node.x,nearest_node.y)
                             if d < car_radius:
                                 no_collion = 0
                                 return no_collion
                 #nearest_node and new_node are not on the same side
                 #then check if the car's road will intersect with the obstacles
                 else:
                     #nearest_node on top of the rectangle
                     if nearest_node.y >  oy+size_y:
                         #slopes of the lines from nearest_node to the reactangle's two nearest conners
                         k1 = (nearest_node.y-top_left_conner.y)/(nearest_node.x-top_left_conner.x)
                         k2 = (nearest_node.y-top_right_conner.y)/(nearest_node.x-top_right_conner.x)
                         if not (k>k2 and k<k1):
                             collion = 0
                             return no_collion #no collion 0
                         else:
                             if k>0:
                                 d = calculate_d(top_left_conner.x,top_left_conner.y,new_node.x,new_node.y,nearest_node.x,nearest_node.y)
                                 if d < car_radius:
                                     no_collion = 0
                                     return no_collion                     
                             elif k<0:
                                 d = calculate_d(top_right_conner.x,top_right_conner.y,new_node.x,new_node.y,nearest_node.x,nearest_node.y)
                                 if d < car_radius:
                                     no_collion = 0
                                     return no_collion                             
                     #nearest_node on bottom of the rectangle
                     elif nearest_node.y < oy:
                         #slopes of the lines from nearest_node to the reactangle's two nearest conners
                         k1 = (nearest_node.y-lower_left_conner.y)/(nearest_node.x-lower_left_conner.x)
                         k2 = (nearest_node.y-lower_right_conner.y)/(nearest_node.x-lower_right_conner.x)
                         if not (k>k2 or k<k1):
                             collion = 0
                             return no_collion #no collion 0 
                         else:
                              if k>0:
                                  d = calculate_d(lower_right_conner.x,lower_right_conner.y,new_node.x,new_node.y,nearest_node.x,nearest_node.y)
                                  if d < car_radius:
                                      no_collion = 0
                                      return no_collion                     
                              elif k<0:
                                  d = calculate_d(lower_left_conner.x,lower_left_conner.y,new_node.x,new_node.y,nearest_node.x,nearest_node.y)
                                  if d < car_radius:
                                      no_collion = 0
                                      return no_collion


                     #nearest_node on left of 
                     elif nearest_node.x <ox:
                         #slopes of the lines from nearest_node to the reactangle's two nearest conners
                         k1 = (nearest_node.y-top_left_conner.y)/(nearest_node.x-top_left_conner.x)
                         k2 = (nearest_node.y-lower_left_conner.y)/(nearest_node.x-lower_left_conner.x)
                         if not (k<k2 or k>k1):
                             collion = 0
                             return no_collion #no collion 0
                         else:
                             if k>0:
                                 d = calculate_d(top_left_conner.x,top_left_conner.y,new_node.x,new_node.y,nearest_node.x,nearest_node.y)
                                 if d < car_radius:
                                     no_collion = 0
                                     return no_collion                     
                             elif k<0:
                                 d = calculate_d(lower_left_conner.x,lower_left_conner.y,new_node.x,new_node.y,nearest_node.x,nearest_node.y)
                                 if d < car_radius:
                                     no_collion = 0
                                     return no_collion
                     #nearest_node on right of the rectangle
                     elif nearest_node.x > ox + size_x:
                         #slopes of the lines from nearest_node to the reactangle's two nearest conners
                         k1 = (nearest_node.y-top_right_conner.y)/(nearest_node.x-top_right_conner.x)
                         k2 = (nearest_node.y-lower_right_conner.y)/(nearest_node.x-lower_right_conner.x)
                         if not (k>k2 or k<k1):
                             collion = 0
                             return no_collion #no collion 0
                         else:
                             if k<0:
                                 d = calculate_d(top_right_conner.x,top_right_conner.y,new_node.x,new_node.y,nearest_node.x,nearest_node.y)
                                 if d < car_radius:
                                     no_collion = 0
                                     return no_collion                     
                             elif k>0:
                                 d = calculate_d(lower_right_conner.x,lower_right_conner.y,new_node.x,new_node.y,nearest_node.x,nearest_node.y)
                                 if d < car_radius:
                                     no_collion = 0
                                     return no_collion
        return no_collion

    #calculate the distance between the conner and line
    def calculate_d(x0,y0,x1,y1,x2,y2):
        A=y1-y2
        B=x2-x1
        C=(y2-y1)*x2
        return math.fabs(A*x0+B*y0+C)/math.sqrt(math.pow(A,2)+math.pow(B,2))

    def planning(self):
        """
        Path planning

        animation: flag for animation on or off
        """

        while True:
            # Random Sampling
            rnd = self.random_node()

            # Find nearest node
            min_index = self.get_nearest_list_index(self.nodeList, rnd)

            # expand tree
            nearest_node = self.nodeList[min_index]

            # return theta
            theta = math.atan2(rnd[1] - nearest_node.y, rnd[0] - nearest_node.x)

            new_node = copy.deepcopy(nearest_node)
            new_node.x += self.expandDis * math.cos(theta)
            new_node.y += self.expandDis * math.sin(theta)
            new_node.parent = min_index
            new_node.theta = theta

            if not self.collision_check(nearest_node, new_node, self.obstacleList):
                continue

            self.nodeList.append(new_node)

            # check goal
            dx = new_node.x - self.end.x
            dy = new_node.y - self.end.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= self.expandDis:
                print("Goal!!")
                break

            if True:
                self.draw_graph(rnd)
        path = [[self.end.x, self.end.y,0]]
        last_index = len(self.nodeList) - 1
        while self.nodeList[last_index].parent is not None:
            node = self.nodeList[last_index]
            path.append([node.x, node.y,node.theta])
            last_index = node.parent
            
        path.append([self.start.x, self.start.y,0])

        return path

    #draw dynamic graph
    def draw_graph(self, rnd=None):
        plt.clf()  # clf graph 
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^g")

        #draw and connect all the nodes that is in the RRT
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.x, self.nodeList[node.parent].x], [
                         node.y, self.nodeList[node.parent].y], "-g")

        #draw the obstacles
        for (ox, oy, size_x,size_y) in self.obstacleList:
            rect = plt.Rectangle((ox,oy),size_x,size_y, fc='k')
            plt.gca().add_patch(rect)
            #plt.plot(ox, oy, "sk", ms=10*size)

        plt.plot(self.start.x, self.start.y, "^r")
        plt.plot(self.end.x, self.end.y, "^b")
        plt.axis([self.min_rand, self.max_rand, self.min_rand, self.max_rand])
        plt.grid(True)
        plt.pause(0.01)

    #draw static graph
    def draw_static(self, path):
        plt.clf()  # clear graph

        #draw circle around the origin of the car
        for data in path:
            circle = plt.Circle((data[0],data[1]),7,fc='y')
            plt.gca().add_patch(circle)

        #draw and connect all the nodes that is in the RRT
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.x, self.nodeList[node.parent].x], [
                    node.y, self.nodeList[node.parent].y], "-g")

        #draw the obstacles
        for (ox, oy, size_x,size_y) in self.obstacleList:
            rect = plt.Rectangle((ox,oy),size_x,size_y, fc='k')
            plt.gca().add_patch(rect)

        plt.plot(self.start.x, self.start.y, "^r")
        plt.plot(self.end.x, self.end.y, "^b")
        plt.axis([self.min_rand, self.max_rand, self.min_rand, self.max_rand])

        #indicate the nearest path for the car
        plt.plot([data[0] for data in path], [data[1] for data in path], '-r')
        plt.grid(True)
        plt.show()


def main():
    print("start RRT path planning")

    obstacle_list = [
        (-2,10,5,40),
        (40,-2,5,40),
        (70, 40, 5,40)
        ]

    # Set Initial parameters
    rrt = RRT(start=[start_x, start_y], goal=[85,48], rand_area=[-2, 100], obstacle_list=obstacle_list)
    path = rrt.planning()
    print(path)

    # Draw final path
    if show_animation:
        plt.close()
        rrt.draw_static(path)


if __name__ == '__main__':
    main()

