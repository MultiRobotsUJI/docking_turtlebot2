#!/usr/bin/env python
import numpy as np


class CameraParameters: 
    def __init__(self):
        u0 = 570.3422241210938
        v0 = 570.3422241210938
        lx = 320 #
        ly = 240 #
        kud =0.00683 
        kdu = -0.01424  
        self.u0 = u0
        self.v0 = v0
        self.lx = lx
        self.ly = ly
        self.kud = kud
        self.kdu = kdu


    def convert2meter(self, pt):
        return (pt[0]-self.u0)/self.lx, (pt[1]-self.v0)/self.ly

    def convertListPoint2meter (self, points):
        # global u0,v0,lx, ly
        
        if(np.shape(points)[0] > 1):
            n = int(np.shape(points)[0]/2)
            point_reshaped = (np.array(points).reshape(n,2))
            point_meter = []
            for pt in point_reshaped:
                pt_meter = self.convert2meter(pt)
                point_meter.append(pt_meter)
            point_meter = np.array(point_meter).reshape(-1)
            return point_meter