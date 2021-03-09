from math import tan,sin,cos,atan 

class Jetbot():
    '''
    Jetbot class for jetbot movement 
    '''
    def __init__(self, position, grabber_p):
        self.position = list(position)
        self.heading =  self.checkHeading(grabber_p,position)
        self.visited = [self.position + [self.heading]]
        self.Horizon = 3  # A tuning parameter 
        self.Angle = 3.1415 / 12  # Turning angle at each step 
        self.StepLen = 10  # Step Length, should be shorter than Horizon

    def right(self):
        self.heading -= self.Angle
        self.visited.append(self.position + [self.heading])

    def left(self): 
        self.heading += self.Angle
        self.visited.append(self.position + [self.heading])

    def forward(self):
        x, y = self.position 
        x += int(self.StepLen * cos(self.heading))
        y += int(self.StepLen * sin(self.heading))
        self.position = [x,y]
        self.visited.append([x,y,self.heading])
        


    def back(self):
        """ It might not be used in real-time movement """
        x, y = self.position 
        x -= 10 
        y -= int(10 * tan(self.heading))
        self.position = [x,y]
        print('backward')

    def jetbot_step(self, cmds, obs_set):
        """ A step function for jetbot simulation 
        First turn to the desired direction 
        Then move forward in that direction 
        :cmd: the list of path solved by Astar 
        :obs_ls: the list of obstacles """
        target_point = cmds[-self.Horizon]
        target_heading = self.checkHeading(target_point,self.position)
        while abs(self.heading - target_heading) > 0.27:
            # turn until it reaches a desired angle,
            # the angle should be slightly larger than self.Angle
            if target_heading > self.heading:
                self.left() 
            else:
                self.right() 
        i = 1 
        while not self.can_step(obs_set):  # check if one step further will collide to choose the right side
            if target_heading > self.heading:  
                if i % 2 ==1: 
                    for _ in range(i): self.left() 
                if i % 2 == 0: 
                    for _ in range(i): self.right()  
            else: 
                if i % 2 ==1: 
                    for _ in range(i): self.right() 
                if i % 2 == 0: 
                    for _ in range(i): self.left()   
            i += 1 
        self.forward() 
        print('Position: ',self.position, 
            'Heading: ', self.heading *360/(2*3.14),
            'Target Point: ', target_point)
        
    def get_trajectory(self):
        return self.visited

    def get_position(self):
        return tuple(self.position)

    def checkHeading(self, pHead, pBody):
        ''' check where the jetbot is heading for
        :return: the theta of heading angle '''
        Pi = 3.1415926
        # pHead = objs['Target'][0] 
        # pBody = objs['Jetbot'][0] 
        x1, y1 = pHead[0], pHead[1] 
        x2, y2 = pBody[0], pBody[1] 
        dx, dy = x1-x2, y1-y2 
        if dx == 0: 
            return Pi if dy > 0 else -Pi 
        if dx > 0:  # first quadrant and forth quadrant 
            return atan((y1-y2)/(x1-x2))
        if dx < 0 and dy > 0:  # second quadrant 
            return atan((y1-y2)/(x1-x2)) + Pi  
        if dx < 0 and dy < 0:  # third quadrant 
            return atan((y1-y2)/(x1-x2)) - Pi 
    
    def can_step(self, obs): 
        """ check if jetbot can go in that direction
        Since one step further can collide
        :obs_set: the Obstacle set (same idea as that in Astar env) """
        x_test, y_test = self.position
        x_test += int(self.StepLen * cos(self.heading))
        y_test += int(self.StepLen * sin(self.heading)) 
        return False if (x_test,y_test) in obs else True 

if __name__ == '__main__':
    PI = 3.1415
    print(PI/12  * 180/ PI)
    print(-29.418124371804982 * PI / 180)
    