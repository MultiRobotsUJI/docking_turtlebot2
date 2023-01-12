#!/usr/bin/env python3
import math
from os import kill
import string
import numpy as np

import utils as ut
from tf.transformations import quaternion_from_matrix, quaternion_matrix

class Transform:
    def __init__(self):
        # self.cam = cam
        self.ut = ut
        pass

    def rodrigues_to_quaternion(self, rvec):
        angle = np.linalg.norm(rvec)
        ax, ay, az = rvec / angle
        qx = ax * np.sin(angle/2)
        qy = ay * np.sin(angle/2)
        qz = az * np.sin(angle/2)
        qw = np.cos(angle/2)
        return np.array([qx, qy, qz, qw])

    def transform_position(self, position, rotation, as_quat=False):
    # Create a quaternion from the rotation part of the transform matrix
        if not as_quat:
            quat = quaternion_from_matrix(rotation)
        else:
            quat = rotation
        transformed_matrix = quaternion_matrix(quat)
        transformed_matrix[:3, 3] = position
        
        return transformed_matrix


    def rotXYZ(self, rx,ry,rz, degrees =True):
        if(degrees):
            rx *= math.pi/180
            ry *= math.pi/180
            rz *= math.pi/180
        rotx = np.array([[1,0,0],
                        [0,math.cos(rx),-math.sin(rx)],
                        [0,math.sin(rx),math.cos(rx)]])
        
        roty = np.array([[math.cos(ry),0,math.sin(ry)],
                        [0,1,0],
                        [-math.sin(ry),0, math.cos(ry)]])
        
        rotz = np.array([[math.cos(rz),-math.sin(rz),0],
                        [math.sin(rz),math.cos(rz),0],
                        [0,0,1]])
        rotxyz = rotx.dot(roty.dot(rotz))
    # print('r',rotxyz)
        rotxyz = np.round (rotxyz,15)
        #print('round',rotxyz)
        return rotxyz
        

    # define the transformations from one frame c to frameo
    def homogenousMatrix(self, tx,ty,tz,rx,ry,rz):
        cRo = self.rotXYZ(rx,ry,rz, degrees =True)
        cTo = np.array([[tx,ty,tz]])
        M = np.concatenate(
            (np.concatenate((cRo,cTo.T), axis=1),
            np.array([[0,0,0,1]])),axis=0 )
        return M

    # skew a 3x1 vector in a matrix
    def skew_vec (self, vec):
        mat = np.array([[0,-vec[2], vec[1]],[vec[2],0,-vec[0]],[-vec[1], vec[0], 0]])
        return mat


    # expressed a velocity in frame a knowing velocity in b an
    # change of frame aMb
    def velocityTwistMatrix(self, tx,ty,tz,rx=0,ry=0,rz=0, aRb=None):
        aRb = self.rotXYZ(rx,ry,rz, degrees=True) if aRb is None else aRb
        aTb_x = self.skew_vec(np.array([tx,ty,tz]))
        aTb_xaRb = aTb_x.dot(aRb)
        aVb1 = np.concatenate((aRb,aTb_xaRb),axis=1)
        aVb2 = np.concatenate((np.zeros((3,3)),aRb),axis=1) 
        aVb = np.concatenate ((aVb1,aVb2))
        return aVb