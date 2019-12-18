import numpy as np
PI = np.pi
G = 1 #newtons gravitational Constant
#===============================================================================
def distance(pos1,pos2):
    """
    Overview: Calculate distance between two bodies given 2 lists of coordinates
    """
    try:
        coord = []
        for i in range(0,len(pos1)):
            coord.append((pos1[i] - pos2[i])**2)
        return np.sqrt(sum(coord))
    except:
        return np.sqrt((pos1 - pos2) ** 2)
#===============================================================================

#===============================================================================
def distanceCoord(pos1,pos2):
    """
    Overview: Calculate distance between two bodies given 2 lists of coordinates
    """
    coord = [(p1 - p2) ** 2 for p1,p2 in zip(pos1, pos2)]
    return coord
#===============================================================================

#===============================================================================
def position(a,t,cVel):
    """
    Overview: Calculate change in position of a body due to a change in time
    """
    return (cVel * t) + (0.5 * a * (t**2))
#===============================================================================

#===============================================================================
def acceleration(f,m):
    """
    Overview: Calcualte acceleration of a body
    """
    return f / m
#===============================================================================

#===============================================================================
def velocity(cVel, a, t):
    """
    Overview: Update velocity of a body based on current velocity
    """
    return cVel + (a * t)
#===============================================================================

#===============================================================================
def force(mA,mB,d):
    """
    Overview: Calcualte the gravitational force of one body on another
    """
    return (G * mA * mB) / (d ** 2)
#===============================================================================

#===============================================================================
def makePlot(ax,x,y,z,col):
    """
    Overview: Make a plot of x vs y vs z
    """
    ax.scatter(x, y, z, c = col)
#===============================================================================

#===============================================================================
def getRad(r):
    """
    Overview: Perform pythageras therom on 3 lists of coordinates
    """
    return np.sqrt(r[0] ** 2 + r[1] ** 2 + r[2] ** 2)
#===============================================================================

#===============================================================================
def getTheta(z,r):
    """
    Overview: Return the altitude angle of the object
    """
    try:
        print(z,r)
        return np.arccos(z / r)
    except:
        return "Crash"
#===============================================================================

#===============================================================================
def getPhi(x,y):
    """
    Overview: Return the azimuth angle of the object
    """
    return np.arctan2(y,x)
#===============================================================================
