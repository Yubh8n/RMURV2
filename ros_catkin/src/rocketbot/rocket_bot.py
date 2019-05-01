#!/usr/bin/env python
"""ROCKET BOT

A simulated Rocket with a ROS interface.  Just intended to serve as an
alternative to turtlesim for playing with topics and messages.

Publishes to:
  location  (geomtry_msgs/Point)

Subscribes to:
  thrust    (geometry_msgs/Vector3)

(0, 0, 0) is at the lower left edge of the screen.  The x axis points
to the right and the y axis points up.  Thrust along the z axis will
be ignored.  Negative thrusts along the y axis will be ignored.

Author: Nathan Sprague
Updated by: Martin Peter Christiansen

"""
# Some code taken from :
#http://www.gpwiki.org/index.php/Python:Pygame_basics



import pygame
import random
import numpy as np
import rospy

from pygame.locals import *
import time

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point

GRAVITY = 9.8

class Rocket(pygame.sprite.Sprite):
    """ This is a rocket sprite. """

    HEIGHT = 30
    WIDTH = 10
    FLAME_HEIGHT = 20
    CONE_HEIGHT = 10
    def __init__(self, screen, pos, vel):
        pygame.sprite.Sprite.__init__(self)
        self._screen = screen
        self.thrust = np.array(pos, dtype='float64')
        self.pos = np.array(pos, dtype='float64')
        self.vel = np.array(vel, dtype='float64')
        self.trust_state = 0.0
        self.sensor_state = 0.0

    def firstOrderTrustLag(self, input, dt):
        #Function to implement the discretization of a continuous time first
        # order lag H(s) = 1 /(tau*s + 1)
        tau = 0.0457
        #print "i: "+str(input)
        x = input
        y_d = ((x-self.trust_state))/tau
        self.trust_state +=(dt*y_d)
        #print "o: "+str(self.trust_state)

        return self.trust_state

    def update(self, thrust, dt):
        """Update the position and velocity based on the thrust.  Redraw the
        rocket at the new position.
        """
        v_dampning = 0.997
        self.thrust[1] = thrust[1] #self.firstOrderTrustLag(,dt)
        self.acc = (GRAVITY +self.thrust[1])
        self.vel[1] = self.vel[1]*v_dampning+self.acc*dt
        self.pos[1] += self.vel[1]*dt+0.5*self.acc*dt*dt

        if self.pos[0] > self._screen.get_width():
            self.pos[0] = self._screen.get_width()
            self.vel[0] = -self.vel[0]
        if self.pos[0] < 0:
            self.pos[0] = 0
            self.vel[0] = -self.vel[0]
        if self.pos[1] > self._screen.get_height()-self.HEIGHT:
            self.pos[1] = self._screen.get_height() - self.HEIGHT
            self.vel[1] = 0
        if self.pos[1] < 0:
            self.pos[1] = 0
            self.vel[1] = 0

    def draw(self):

        if self.thrust[0] != 0 or self.thrust[1] != 0:
            p1 = (self.pos[0], self.pos[1] + self.HEIGHT)
            p2 = (self.pos[0]+ self.WIDTH-1, self.pos[1] + self.HEIGHT)
            p3 = (self.pos[0]+ self.WIDTH/2,
                  self.pos[1] + self.HEIGHT + self.FLAME_HEIGHT)
            pygame.draw.polygon(self._screen, (255, 0, 0), (p1, p2, p3), 0)

        p1 = self.pos
        p2 = (self.pos[0]+ self.WIDTH-1, self.pos[1])
        p3 = (self.pos[0]+ self.WIDTH/2, self.pos[1] - self.CONE_HEIGHT)
        pygame.draw.polygon(self._screen, (0, 0, 0), (p1, p2, p3), 0)

        pygame.draw.rect(self._screen, (0, 0, 0), Rect(self.pos, (10,30)))




class RocketNode(object):
    def __init__(self):
        rospy.init_node('rocket_bot')
        rospy.Subscriber('thrust', Vector3, self.thrust_callback)
        self.loc_pub = rospy.Publisher('location', Point, queue_size=1)
        self.cur_thrust = np.array([0, 0])
        self.thrust_start = 0
        self.pub_rate = 20.0
        self.refresh_rate = 100.0
        self.old_time = rospy.get_time()
        self.trust_time = rospy.get_time()
        self.sensor_output_y = 0.1

        pygame.init()
        pygame.display.set_caption('RocketBot 101')
        self.width = 480
        self.screen = pygame.display.set_mode((self.width, self.width))
        #clock = pygame.time.Clock()
        self.rocket = Rocket(self.screen, (self.width/2, self.width), [0.0, 0.])
        rospy.Timer(rospy.Duration(1./self.refresh_rate), self.timer_callback)
        rospy.Timer(rospy.Duration(1./self.pub_rate), self.timer_draw)
        rospy.spin()

    def thrust_callback(self, thrust):
        self.trust_time = rospy.get_time()
        if thrust.y < 0.:
            thrust.y = 0.
        if thrust.y > 1000.:
            thrust.y = 1000.

        self.cur_thrust = np.array([thrust.x, -round(thrust.y/10,1)])

    def timer_callback(self, event):
        time = rospy.get_time()
        dt = time-self.old_time
        #print 'Time step ' + str(dt) #remove when running live with real data
        self.rocket.update(self.cur_thrust, dt)
        self.old_time = time

    def timer_draw(self, event):
        pygame.display.flip()
        d = (self.width - self.rocket.pos[1] - self.rocket.HEIGHT)
        point = Point(self.rocket.pos[0], self.sensor_output_y, 0.0)
        self.loc_pub.publish(point)
        self.sensor_output_y = round(d,2)

        self.screen.fill((255, 255, 255))
        self.rocket.draw()

        if((rospy.get_time()-self.trust_time) > 0.5):
            self.cur_thrust = np.array([0, 0])

if __name__ == "__main__":
    node = RocketNode()
    #node.run()
