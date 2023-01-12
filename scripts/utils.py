#!/usr/bin/env python
import numpy as np
    

        
# the interactionMatrix for a point with rho theta representation
def interactionMatrixFeaturePointRhoTheta(rho,theta):
    L = np.zeros(2,6)
    return L

# the interactionMatrix for a segment
def interactionMatrixFeaturePointRhoTheta(xm,ym,l,alpha):
    L = np.zeros(2,6)
    return L
    

# the interactionMatrix for point coordinates
def interactionMatrixFeaturePoint2D(x,y,Z=1):
    Lx = np.array([ -1 / Z, 0, x / Z, x * y,-(1 + x * x), y])
    Ly = np.array([0, -1 / Z, y / Z, 1 + y * y, -x * y, -x ])
    L = np.stack((Lx.T,Ly.T),axis=0)
    return L


#the stack of interactionMatrices for list of points
def interactionMatrixFeaturePoint2DList(points, Zs=None):
    n = int(np.shape(points)[0]/2)
    #if(len(Zs)!=n):
    if Zs is None:
       Zs = np.ones(n)
        
    iter = 0 
    L = [[]]
    # for all the points
    point_reshaped = (np.array(points).reshape(n,2))
    for p in point_reshaped:
        Lp = interactionMatrixFeaturePoint2D(p[0],p[1],Zs[iter])
        if(iter == 0) : 
            L = Lp
        else: 
            L = np.concatenate((L,Lp))
        iter += 1
    return L


