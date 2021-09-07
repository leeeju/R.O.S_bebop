#!/usr/local/bin/python
""" potential_path.py
2D Potential Path Planning Algorithm
gradient values are given in (x, y)
potential values are given as scalar U
Currently potential maxes out at the specified max_potential for each obstacle.
- It would be nice if they created a mountain structure such that the visualization of potential looks nicer
Most interactions will take place with the map class. Obstacles and goals should largely not be touched after initialization.
"""
import numpy as np

def pol2cart(rho, theta):
    return (rho*np.cos(theta), rho*np.sin(theta))

class ObstaclePotential:
    """ Obstacle Class """
    """ 2 Kinds of Obstacles: Circles and lines) """
    """ Circles are given in radius and a (x, y) coordinate center """
    """ lines are given in a slope and a (x, y) coordinate intercept """
    """ r is the radius of the absolute wall. """
    """ keep_out ('both', 'out', 'in'):
        'both' -> Keeps the object in when its inside the wall keeping it out when it is outside the wall
        'out' -> forces the gradient away from the center of the obstacle even if the location is inside the wall (radius r), forces the gradient of lines to have a positive y (in the case of m = inf -> positive x)
        'in' -> forces the gradient into the center of the obstacle even if the location is outside the wall (radius r), forces the gradient of lines to have a negative y (in the case of m = inf -> negative x)
    """
    #TODO make the keep_out variable more clear for lines
    def __init__(self, kind='circle', location=(0,0), d_safe=1, r=0, m=1, n=1, keep_out='out', max_potential=100, max_gradient=100):
        if kind in ('circle','line'):
            self.kind = kind #Type of obstacle
        else:
            raise KeyError()
        self.location = location #Location of the obstacle (for circles -> center, for lines -> a point on the line)
        self.d_safe = d_safe #Distance at which the obstacle no longer has an affect
        self.n = n #The "gain" of the obstacle
        self.r = r #The radius. Only used for Circles
        self.m = m #The slope. Only used for Lines
        self.dist = d_safe #The current distance away from the obstacle (for cylinders, its the distance to the wall not the center)
        if keep_out in ('both', 'out', 'in'):
            self.keep_out = keep_out #For circles if the obstacle should push objects towards the center or away. For lines if the obstacle should push objects to the left or the right
        else:
            raise KeyError() #TODO figure out if this is the right error.
            self.keep_out = 'out'
        self.max_potential = max_potential #Maximum potential this obstacle will output
        self.max_gradient = max_gradient #Maximum gradient this obstacle will output
    
    #TODO Implement Velocity
    #@param location and velocity of the object in (x,y)
    #Returns the potential at a point. A scalar value.
    def potential(self, location=(0,0), velocity=(0,0)): 
        self.distance(location)
        #Special Conditions for Lines
        if self.kind == 'line':
            self.get_angle(location)
            # If the location is to the right of a vertical line pushing to the left
            if self.angle == 0 and self.keep_out == 'in':
                return self.max_potential
            # If the location is to the left of a vertical line pushing to the right 
            elif np.absolute(self.angle) == np.pi and self.keep_out == 'out':
                return self.max_potential
            # If the location is in quadrants 3 or 4 relative to the line and the line is pushing up
            elif self.angle < 0 and self.keep_out == 'out':
                return self.max_potential
            # If the location is in quadrants 1 or 2 relatice to the line and the line is pushing down
            elif self.angle > 0 and self.keep_out == 'in':
                return self.max_potential
        #Special conditions for circles
        elif self.kind == 'circle':
            # If the location is inside the circle and the circle is pushing out
            if self.true_dist < self.r and self.keep_out == 'out':
                return self.max_potential 
            # If the location is outside the circle and the circle is pushing in
            elif self.true_dist > self.r and self.keep_out == 'in':
                return self.max_potential
        # If the location is out of the obstacle's range
        if self.dist >= self.d_safe:
            return 0
        else:
            # Returns the potential capped at the max_potential
            return min(0.5*self.n*np.square(1.0/self.dist - 1.0/self.d_safe), self.max_potential, key=lambda value: np.absolute(value))

    #TODO Implement Velocity
    #@Params location and velocity in (x,y) form
    def gradient(self, location=(0,0), velocity=(0,0)):
        self.distance(location)
        # Special Conditions for Lines
        if self.kind == 'line':
            #Note mathematically the gradient here should be negative but because of the get_angle function it needs to be positive
            self.get_angle(location)
            # If the location is to the right of a vertical line pushing to the left
            if self.angle == 0 and self.keep_out == 'in':
                return pol2cart(self.max_gradient, self.get_angle(location, velocity))
            # If the location is to the left of a vertical line pushing to the right
            elif np.absolute(self.angle) == np.pi and self.keep_out == 'out':
                return pol2cart(self.max_gradient, self.get_angle(location,velocity))
            # If the location is in quadrants 3 or 4 relative to the line and the line is pushing up
            elif self.angle < 0 and self.keep_out == 'out':
                return pol2cart(self.max_gradient, self.get_angle(location, velocity))
            # If the location is in quadrants 1 or 2 relative to the line and the line is pushing down
            elif self.angle > 0 and self.keep_out == 'in':
                return pol2cart(self.max_gradient, self.get_angle(location, velocity))
        # Special conditions for circles
        elif self.kind == 'circle':
            # If the location is inside the circle and the circle is pushing out
            if self.true_dist < self.r and self.keep_out == 'out':
                return pol2cart(-1*self.max_gradient, self.get_angle(location, velocity))
            # If the location is outside the circle and the circle is pushing in
            elif self.true_dist > self.r and self.keep_out == 'in':
                return pol2cart(self.max_gradient, self.get_angle(location, velocity))
        # If the location is outside of the obstacle's range
        if self.dist >= self.d_safe:
            return (0,0)
        else:
            # Returns the gradient in (x,y) form, the magnitude of the vector is capped at max_gradient
            return pol2cart(self.coeff(location)*min(self.n*((1.0/self.d_safe) - (1.0/self.dist))/(np.square(self.dist)),-1*self.max_gradient,key=lambda value: np.absolute(value)), self.get_angle(location, velocity))

    #TODO Implement Velocity
    #@Params location and velocity in (x,y) form
    def get_angle(self, location=(0,0), velocity =(0,0)):
        # Calculation for lines
        if self.kind == 'line':
            # If the point is below the line
            if self.m*(location[0]-self.location[0])+self.location[1] > location[1]:
                self.angle = -np.pi/2.0 + np.arctan(self.m)
            # If the point is above the line
            else:
                self.angle = np.pi/2.0 + np.arctan(self.m)
        # Calculation for circles
        elif self.kind == 'circle':
            x = location[0] - self.location[0]
            y = location[1] - self.location[1]
            self.angle = np.arctan2(y,x)
        return self.angle

    #TODO Would it be more efficient if I saved the previous location and if its the same location just skip the calculation?
    # consider the same thing for angle
    #@Param location in (x,y) coordinates
    def distance(self, location=(0,0)):
        # Calculation for lines
        if self.kind == 'line':
            # special case for horizontal lines
            if self.m == 0:
                self.true_dist = location[1] - self.location[1]
            # special case for vertical lines
            elif self.m == np.inf:
                self.true_dist = location[0] - self.location[0]
            else:
                self.true_dist = (self.m*location[0]-location[1]-self.m*self.location[0]+self.location[1])/np.sqrt(np.square(self.m)+1)
            self.dist = np.absolute(self.true_dist)
        # calcualtion for circles
        elif self.kind == 'circle':
            # self.true_dist is the distance from the center of the circle
            self.true_dist = np.sqrt(np.square(location[0]-self.location[0])+np.square(location[1]- self.location[1]))
            # self.dist is the distance from the wall of the circle.
            self.dist = np.absolute(self.true_dist-self.r)
        return self.dist
    
    #@Param location in (x,y) coordinates
    #Gives coefficients to multiply the gradient by in order to orient them in the correct way.
    #Note that this does not apply to special cases in which max_gradient is used. Those are already considered.
    def coeff(self, location=(0,0)):
        # If it is a line, it should always be pushing away
        if self.kind == 'line':
            return 1
        # If it is a circle
        elif self.kind == 'circle':
            # Push it away to keep things out
            if self.keep_out == 'out':
                return 1
            # Push it in to keep things in
            elif self.keep_out == 'in':
                return -1
            # Depending on where the location is, push in or out.
            elif self.keep_out == 'both':
                if (self.true_dist - self.r) > 0:
                    return 1
                else:
                    return -1
            else:
                raise KeyError("Keep_out has an invalid value") #TODO figure out if this is the right error.
                return 0

class GoalPotential:
    """ Goal Class """
    """ z: gain
        location: location of the goal
        kind: algorithm to use (Quadratic changes force with distance and conics apply a constant force. Hybrid uses quadratic when close and conic when far)
        d_threshold: the threshold at which to switch between algorithms if hybrid was chosen
    """
    def __init__(self, location=(0,0), z=1, kind='quadratic', d_threshold=10):
        if kind in ('quadratic', 'conic', 'hybrid'):
            self.kind = kind #The kind of algorithm to use
        else:
            raise KeyError() #TODO figure out if this is the right error.
            self.kind=0
        self.location = location #Location of the goal
        self.z = z #"Gain" of the goal
        self.d_threshold = d_threshold #The distance away from the goal at which point the threshold switches from conic to quadratic

    #TODO Implement Velocity
    #@param location and velocity in (x,y) coordinates
    def potential(self, location=(0,0), velocity=(0,0)):
        potential = 0
        # Get the appropriate algorithm
        if self.kind == 'quadratic':
            potential = self._potential_quadratic(location, velocity)
        elif self.kind == 'conic':
            potential = self._potential_conic( location, velocity)
        elif self.kind == 'hybrid':
            # Check if the threshold for conics is met
            if self.distance(location) > self.d_threshold:
                potential = self._potential_conic(location, velocity)
            else:
                potential = self._potential_quadratic(location, velocity)
        return potential

    #TODO Implement Velocity
    #@param location and velocity in (x,y) coordinates.
    def _potential_quadratic(self, location, velocity):
        return 0.5*self.z*np.square(self.distance(location))

    #TODO Implement Velocity
    #@param location and velocity in (x,y) coordinates.
    def _potential_conic(self, location, velocity):
        return self.d_threshold*self.z*self.distance(location)-0.5*self.z*np.square(self.d_threshold)

    #TODO Implement Velocity
    #@param location and velocity in (x,y) coordinates.
    def gradient(self, location=(0,0), velocity=(0,0)):
        gradient = 0 #Note gradient is the magnitude of the gradient.
        # Get the appropriate algorithm
        if self.kind == 'quadratic':
            gradient = self._gradient_quadratic(location, velocity)
        elif self.kind == 'conic':
            gradient = self._gradient_conic(location, velocity)
        elif self.kind == 'hybrid':
            # check if the threshold for conics is met
            if self.distance(location=location) > self.d_threshold:
                gradient = self._gradient_conic(location, velocity)
            else:
                gradient = self._gradient_quadratic(location, velocity)
        #Convert magnitude and angle to <x,y> form
        return pol2cart(gradient, self.get_angle(location, velocity))

    #TODO Implement Velocity
    #@param location and velocity in (x,y) coordinates
    def _gradient_quadratic(self, location, velocity):
        return self.z*self.distance(location)

    #TODO Implement Velocity
    #@param location and velocity in (x,y) coordinates
    def _gradient_conic(self, location, velocity):
        return self.d_threshold*self.z

    #TODO Implement Velocity
    #@param location and velocity in (x,y) coordinates
    #returns the angle pointing from the object to the location inputed
    def get_angle(self, location=(0,0), velocity =(0,0)):
        x = location[0] - self.location[0]
        y = location[1] - self.location[1]
        self.angle = np.arctan2(y,x)
        return self.angle

    #TODO Would it be more efficient if I saved the previous location and if its the same location just skip the calculation?
    def distance(self, location=(0,0)):
        self.dist = np.sqrt(np.square(location[0]-self.location[0])+np.square(location[1]- self.location[1]))
        return self.dist

    #Sets the location of the goal
    #@param location given in (x,y) coordinates
    def set_location(self, location): #TODO check if location is valid.
        self.location = location

class MapPotential:
    """ Stores all goals and obstacles """
    # Creates a map with the default goal and no obstacles
    # @param any goal or obstacle to override
    def __init__(self, goal=GoalPotential(), obstacles=()):
        self.goal = goal
        self.obstacles = obstacles

    #TODO Implement velocity
    # @param location and velocity in (x,y) form
    # returns the added potentials of the goals and obstacles
    def potential(self, location, velocity=(0,0)):
        potential_obstacles = 0
        for obstacle in self.obstacles:
            potential_obstacles += obstacle.potential(location, velocity)
        return self.goal.potential(location, velocity)+potential_obstacles

    #TODO Implement velocity
    # @param location and velocity in (x,y) form
    # Returns the negative total gradient of goals and obstacles such that it can be directly inputted into speed.
    def gradient(self, location, velocity=(0,0)):
        gradient_obstacles = (0,0)
        for obstacle in self.obstacles:
            gradient_obstacles = np.add(gradient_obstacles, obstacle.gradient(location, velocity))
        return -1*np.add(self.goal.gradient(location, velocity), gradient_obstacles)

    # returns the goal
    def get_goal(self):
        return self.goal

    # sets the goal
    def set_goal(self, goal): #TODO verify goal
        self.goal = goal
