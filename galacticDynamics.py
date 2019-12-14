"""
Author: Ben Guilfoyle - github.com/bengfoyle
Overview: Calculating the gravitational efferty of the close passage of two
glaactic nuclei on a disk of stars. Done as part of Maynooth University
BSc Physics with Astrophysics, Module EP408.
"""

import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt #make plots
from tkinter import *

fig = plt.figure()
sin = np.sin; cos = np.cos;
G = 1 #newtons gravitational Constant
#===============================================================================
def distance(pos1,pos2):
    """
    Overview: Calculate distance between two bodies given 2 lists of coordinates
    """
    try:
        coord = [(p1 - p2) ** 2 for p1,p2 in zip(pos1, pos2)]
        return np.sqrt(sum(coord))
    except:
        return (pos1 - pos2) ** 2
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
def position(a,dt,cVel):
    """
    Overview: Calculate change in position of a body due to a change in time
    """
    return (cVel * dt) + (0.5 * a * (dt**2))
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
def makePlot(fig,x,y,z,col):
    """
    Overview: Make a plot of x vs y vs z
    """
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x, y, z, c = col)
#===============================================================================

#===============================================================================
def getRad(x,y,z):
    """
    Overview: Perform pythageras therom on 3 lists of coordinates
    """
    return np.sqrt(x ** 2 + y ** 2 + z ** 2)
#===============================================================================

#===============================================================================
def getTheta(z,r):
    """
    Overview: Return the altitude angle of the object
    """
    print(z,  "/"  ,r)
    try:
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

#===============================================================================
def simulation():
    """
    Overview: Run a simulation of galactic dynamics for a given system
    """
    #variables follow their conventional meaning eg v = velocity f = force
    #variables will have a number following them to denote which body they
    #refer to v1 = velocity of galaxy 1, v2 = velocity of galaxy 2,
    #v3 = velocity of star

    #lists of inital values
    m1,m2,m3 = float(mass1.get()),float(mass2.get()),0.001
    tRange = np.linspace(0,float(time.get()),float(steps.get()))
    dt = tRange[1] - tRange[0]

    #list of lists which will hold positional components of each body
    p1,p2,p3 = [],[],[]
    initP1,initP2,initP3  = pos1.get(),pos2.get(),pos3.get()
    p1.append(list(float(s) for s in initP1.strip("()").split(",")))
    p2.append(list(float(s) for s in initP2.strip("()").split(",")))
    p3.append(list(float(s) for s in initP3.strip("()").split(",")))

    #list of lists which will hold positional components of each body
    v1,v2,v3 = [],[],[]
    initV1,initV2,initV3  = vel1.get(),vel2.get(),vel3.get()
    v1.append(list(float(s) for s in initV1.strip("()").split(",")))
    v2.append(list(float(s) for s in initV2.strip("()").split(",")))
    v3.append(list(float(s) for s in initV3.strip("()").split(",")))

    for i in range(0,len(tRange)):
        print(i)
        #Center of mass
        comX = ((m1 * np.array(p1[i][0])) + (m2*np.array(p2[i][0]))) / (m1 + m2)
        comY = ((m1 * np.array(p1[i][1])) + (m2*np.array(p2[i][1]))) / (m1 + m2)
        comZ = ((m1 * np.array(p1[i][2])) + (m2*np.array(p2[i][2]))) / (m1 + m2)
        com = [comX,comY,comZ]

        dCom = distanceCoord(com,p3[i])
        d12 = distance(p1[i],p2[i])
        d13 = distance(p1[i],p3[i])
        d23 = distance(p2[i],p3[i])

        print(dCom)

        theta = getTheta(dCom[2], distance(com,p3[i]))
        # theta12 = getTheta(distance(p1[i][2],p2[i][2]), d12)
        # theta23 = getTheta(distance(p2[i][2],p3[i][2]), d23)
        # theta13 = getTheta(distance(p1[i][2],p3[i][2]), d13)

        if "Crash" == str(theta):
            print("2 or more bodies have colided")
            break

        phi = getPhi(dCom[0], dCom[1])
        # phi13 =getTheta(distance(p1[i][0],p2[i][0]),distance(p1[i][1],p2[i][1]))
        # phi23 =getTheta(distance(p2[i][0],p3[i][0]),distance(p1[i][1],p2[i][1]))
        # phi13 =getTheta(distance(p1[i][0],p3[i][0]),distance(p1[i][1],p2[i][1]))

        f12,f13,f23 = force(m1,m2,d12),force(m1,m3,d13),force(m2,m3,d23)

        a1 = acceleration((f13 + f12),m1)
        a2 = acceleration((f23 + f12),m2)
        a3 = acceleration((f13 + f23),m3)

        v1Temp = velocity(v1[i - 1],a1,dt)
        v2Temp = velocity(v2[i - 1],a2,dt)
        v3Temp = velocity(v3[i - 1],a3,dt)

        p1Temp = p1[i - 1] + position(a1,dt,v1[i - 1])
        p2Temp = p2[i - 1] + position(a2,dt,v2[i - 1])
        p3Temp = p3[i - 1] + position(a3,dt,v3[i - 1])


        #appeending values where appropriate
        # p1.append(list(float(s) for s in p1Temp))
        # p2.append(list(float(s) for s in p2Temp))
        # p3.append(list(float(s) for s in p3Temp))
        p1.append(list([p1Temp[0]*sin(theta)*cos(theta),p1Temp[0]*sin(theta)\
                        *sin(phi),p1Temp[0]*cos(theta)]))
        p2.append(list([p2Temp[0]*sin(theta)*cos(theta),p2Temp[0]*sin(theta)\
                        *sin(phi),p2Temp[0]*cos(theta)]))
        p3.append(list([p3Temp[0]*sin(theta)*cos(theta),p3Temp[0]*sin(theta)\
                        *sin(phi),p3Temp[0]*cos(theta)]))

        v1.append(list([v1Temp[0]*sin(theta)*cos(theta),v1Temp[0]*sin(theta)\
                        *sin(phi),v1Temp[0]*cos(theta)]))
        v2.append(list([v2Temp[0]*sin(theta)*cos(theta),v2Temp[0]*sin(theta)\
                        *sin(phi),v2Temp[0]*cos(theta)]))
        v3.append(list([v3Temp[0]*sin(theta)*cos(theta),v3Temp[0]*sin(theta)\
                        *sin(phi),v3Temp[0]*cos(theta)]))
        # vel1.append([v1x,v1y,v1z])
        # vel2.append([v2x,v2y,v2z])
        # vel3.append([v3x,v3y,v3z])

    p1x,p1y,p1z = [i[0] for i in p1],[i[1] for i in p1],[i[2] for i in p1]
    p2x,p2y,p2z = [i[0] for i in p2],[i[1] for i in p2],[i[2] for i in p2]
    p3x,p3y,p3z = [i[0] for i in p3],[i[1] for i in p3],[i[2] for i in p3]

    # makePlot(fig,p1x,p1y,p1z,"r")
    # makePlot(fig,p2x,p2y,p2z,"g")
    makePlot(fig,p3x,p3y,p3z,"b")
    plt.show()
#===============================================================================
#simulation()
# ani = FuncAnimation(fig, simulation, interval=10, blit=True, repeat=True,
#                     frames=np.linspace(0,2*np.pi,360, endpoint=False))
# plt.show()
#===============================================================================

#===============================================================================

"""
GUI
"""
#Define window, name, and parameters
window = Tk()
window.title("Galactic Dynamics")
window.geometry('450x300')

#Insert text with user input text field.
lbl1 = Label(window, text = "Mass of Galaxy 1")
lbl1.grid(column = 0, row = 1)

mass1 = Entry(window, width = 20)
mass1.grid(column = 1, row = 1)

lbl2 = Label(window, text = "Mass of Galaxy 2 relative to Galaxy 1")
lbl2.grid(column = 0, row = 2)

mass2 = Entry(window, width = 10)
mass2.grid(column = 1, row = 2)

lbl3 = Label(window, text = "Galaxy 1 Position eg: x,y,z")
lbl3.grid(column = 0, row = 3)

pos1 = Entry(window, width = 20)
pos1.grid(column = 1, row = 3)

lbl4 = Label(window, text = "Galaxy 2 Position eg: x,y,z")
lbl4.grid(column = 0, row = 4)

pos2 = Entry(window, width = 20)
pos2.grid(column = 1, row = 4)

lbl5 = Label(window, text = "Star Position eg: x,y,z")
lbl5.grid(column = 0, row = 5)

pos3 = Entry(window, width = 20)
pos3.grid(column = 1, row = 5)

lbl6 = Label(window, text = "Galaxy 1 Velocity eg: x,y,z")
lbl6.grid(column = 0, row = 6)

vel1 = Entry(window, width = 20)
vel1.grid(column = 1, row = 6)

lbl7 = Label(window, text = "Galaxy 2 Velocity eg: x,y,z")
lbl7.grid(column = 0, row = 7)

vel2 = Entry(window, width = 20)
vel2.grid(column = 1, row = 7)

lbl8 = Label(window, text = "Star Velocity eg: x,y,z")
lbl8.grid(column = 0, row = 8)

vel3 = Entry(window, width = 20)
vel3.grid(column = 1, row = 8)

lbl9 = Label(window, text = "Total Time")
lbl9.grid(column = 0, row = 9)

time = Entry(window, width = 20)
time.grid(column = 1, row = 9)

lbl10 = Label(window, text = "Number of Time Steps")
lbl10.grid(column = 0, row = 10)

steps = Entry(window, width = 20)
steps.grid(column = 1, row = 10)

btn1 = Button(window, command = simulation, text = "Submit Values")
btn1.grid(column = 0, row = 11)
#loop until closed
window.mainloop()
#===============================================================================
