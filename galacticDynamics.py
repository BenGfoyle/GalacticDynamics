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
G = 1 #newtons gravitational Constant
#===============================================================================
def distance(pos1,pos2):
    """
    Overview: Calculate distance between two bodies given 2 lists of coordinates
    """
    return (pos1 - pos2)**2
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
    f = (-G * mA * mB) / (d ** 2)
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
    return np.arccos(z / r)
#===============================================================================

#===============================================================================
def getPhi(x,y,z):
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
        #change in x direction
        d12x = distance(p1[i][0],p2[i][0])
        d13x = distance(p1[i][0],p3[i][0])
        d23x = distance(p2[i][0],p3[i][0])

        f12x = force(m1,m2,d12x)
        f13x = force(m1,m3,d13x)
        f23x = force(m2,m3,d23x)

        a1x = acceleration((f13x + f12x),m1)
        a2x = acceleration((f23x + f12x),m2)
        a3x = acceleration((f13x + f23x),m3)

        v1x = velocity(v1[i - 1][0],a1x,tRange[i])
        v2x = velocity(v2[i - 1][0],a2x,dt)
        v3x = velocity(v3[i - 1][0],a3x,dt)

        p1x = p1[i - 1][0] + position(a1x,dt,v1[i - 1][0])
        p2x = p2[i - 1][0] + position(a2x,dt,v2[i - 1][0])
        p3x = p3[i - 1][0] + position(a3x,dt,v3[i - 1][0])

        #change in y direction
        d12y = distance(p1[i][1],p2[i][1])
        d13y = distance(p1[i][1],p3[i][1])
        d23y = distance(p2[i][1],p3[i][1])

        f12y = force(m1,m2,d12y)
        f13y = force(m1,m3,d13y)
        f23y = force(m2,m3,d23y)

        a1y = acceleration((f13y + f12y),m1)
        a2y = acceleration((f23y + f12y),m2)
        a3y = acceleration((f13y + f23y),m3)

        v1y = velocity(v1[i - 1][1],a1y,dt)
        v2y = velocity(v2[i - 1][1],a2y,dt)
        v3y = velocity(v3[i - 1][1],a3y,dt)

        p1y = p1[i - 1][1] + position(a1y,dt,v1[i - 1][1])
        p2y = p2[i - 1][1] + position(a2y,dt,v2[i - 1][1])
        p3y = p3[i - 1][1] + position(a3y,dt,v3[i - 1][1])

        #change in z direction
        d12z = distance(p1[i][2],p2[i][2])
        d13z = distance(p1[i][2],p3[i][2])
        d23z = distance(p2[i][2],p3[i][2])

        f12z = force(m1,m2,d12z)
        f13z = force(m1,m3,d13z)
        f23z = force(m2,m3,d23z)

        a1z = acceleration((f13z + f12z),m1)
        a2z = acceleration((f23z + f12z),m2)
        a3z = acceleration((f13z + f23z),m3)

        v1z = velocity(v1[i - 1][2],a1z,dt)
        v2z = velocity(v2[i - 1][2],a2z,dt)
        v3z = velocity(v3[i - 1][2],a3z,dt)

        p1z = p1[i - 1][2] + position(a1z,dt,v1[i - 1][2])
        p2z = p2[i - 1][2] + position(a2z,dt,v2[i - 1][2])
        p3z = p3[i - 1][2] + position(a3z,dt,v3[i - 1][2])

        #appeending values where appropriate
        p1.append([p1x,p1y,p1z])
        p2.append([p2x,p2y,p2z])
        p3.append([p3x,p3y,p3z])

        v1.append([v1x,v1y,v1z])
        v2.append([v2x,v2y,v2z])
        v3.append([v3x,v3y,v3z])

    p1x,p1y,p1z = [i[0] for i in p1],[i[1] for i in p1],[i[2] for i in p1]
    p2x,p2y,p2z = [i[0] for i in p2],[i[1] for i in p2],[i[2] for i in p2]
    p3x,p3y,p3z = [i[0] for i in p3],[i[1] for i in p3],[i[2] for i in p3]

    makePlot(fig,p1x,p1y,p1z,"r")
    makePlot(fig,p2x,p2y,p2z,"g")
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
