from Lidar.LidarSrc import bresenham
from math import sin, cos, log
import math
#import tf
#import rospy
import numpy as np


class localmap:
    def __init__(self, height, width, resolution,morigin):
        self.height=height
        self.width=width
        self.resolution=resolution
        self.punknown=int(10)
        self.localmap=np.array([self.punknown]*int(self.width/self.resolution)*int(self.height/self.resolution), dtype=int)
        self.logodds=[0.0]*int(self.width/self.resolution)*int(self.height/self.resolution)
        self.origin=int(math.ceil(morigin[0]/resolution))+int(math.ceil(width/resolution)*math.ceil(morigin[1]/resolution))
        self.pfree=log(0.3/0.7)
        self.pocc=log(0.9/0.1)
        self.prior=log(0.5/0.5)
        self.max_logodd=100.0
        self.max_logodd_belief=10.0
        self.max_scan_range=1.0
        self.map_origin=morigin



    def updatemap(self,scandata,range_min,range_max,pose):
        # scandata[[angle, distance], ... ]

        self.localmap=np.array([self.punknown]*int(self.width/self.resolution)*int(self.height/self.resolution), dtype=int)
        robot_origin=int(pose[0])+int(math.ceil(self.width/self.resolution)*pose[1])
        centreray=len(scandata)/2+1
        # print(scandata[0])
        for i in range(len(scandata)):
            if not math.isnan(scandata[i][0]):
                #beta=(i-centreray)*angle_increment
                beta = scandata[i][0]
                px=int(float(scandata[i][1])*cos(beta-pose[2])/self.resolution)
                py=int(float(scandata[i][1])*sin(beta-pose[2])/self.resolution)

                l = bresenham.bresenham([0, 0], [px, py])
                for j in range(len(l.path)):                    
                    lpx=self.map_origin[0]+pose[0]+l.path[j][0]*self.resolution
                    lpy=self.map_origin[1]+pose[1]+l.path[j][1]*self.resolution

                    if (0<=lpx<self.width and 0<=lpy<self.height):
                        index=self.origin+int(l.path[j][0]+math.ceil(self.width/self.resolution)*l.path[j][1])
                        if scandata[i][1]<self.max_scan_range*range_max:
                            if(j<len(l.path)-1):self.logodds[index]+=self.pfree
                            else:self.logodds[index]+=self.pocc
                        else:self.logodds[index]+=self.pfree                        
                        if self.logodds[index]>self.max_logodd:self.logodds[index]=self.max_logodd
                        elif self.logodds[index]<-self.max_logodd:self.logodds[index]=-self.max_logodd
                        if self.logodds[index]>self.max_logodd_belief:self.localmap[index]=int(100)
                        else:self.localmap[index]=int(1) 
                        self.localmap[self.origin]=int(100)
