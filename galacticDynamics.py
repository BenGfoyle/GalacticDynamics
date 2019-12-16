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
PI = np.pi
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
        print(z,"/",r)
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
    m1,m2,m3 = 1,1e6,1e-2#float(mass1.get()),float(mass2.get()),0.001
    tRange = np.linspace(0,5,200)#float(time.get()),float(steps.get()))
    dt = tRange[1] - tRange[0]

    #list of lists which will hold positional components of each body
    p1,p2,p3 = [],[],[]
    initP1,initP2,initP3  = "(1,0,0)","(-1,0,0)","(4,0,0)"#pos1.get(),pos2.get(),pos3.get()
    p1.append(list(float(s) for s in initP1.strip("()").split(",")))
    p2.append(list(float(s) for s in initP2.strip("()").split(",")))
    p3.append(list(float(s) for s in initP3.strip("()").split(",")))

    #list of lists which will hold positional components of each body
    v1,v2,v3 = [],[],[]
    initV1,initV2,initV3  = "(1,0,0)","(-1,0,0)","(0,0,0)"#vel1.get(),vel2.get(),vel3.get()
    v1.append(list(float(s) for s in initV1.strip("()").split(",")))
    v2.append(list(float(s) for s in initV2.strip("()").split(",")))
    v3.append(list(float(s) for s in initV3.strip("()").split(",")))

    f1,f2,f3 = [],[],[]

    for i in range(0,len(tRange)):
        print("\n***********************")
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
        dComp3 = distance(dCom,p3[i])

        theta = getTheta(distance(dCom[2],p3[i][2]), distance(dCom,p3[i]))
        print("Theta Params:",distance(dCom[2],p3[i][2]), distance(dCom,p3[i]))
        print("Theta:",theta)
        theta12 = getTheta(distance(p1[i][2],p2[i][2]), d12)
        theta21 = (2 * PI) - theta12
        theta23 = getTheta(distance(p2[i][2],p3[i][2]), d23)
        theta32 = (2 * PI) - theta 23
        theta13 = getTheta(distance(p1[i][2],p3[i][2]), d13)
        theta31 = (2 * PI) - theta13

        if "Crash" == str(theta):
            print("2 or more bodies have colided")
            break

        phi = getPhi(distance(dCom[0],p3[i][0]), distance(dCom[1],p3[i][1]))
        print("Phi Params:",distance(dCom[0],p3[i][0]), distance(dCom[1],p3[i][1]))
        print("Phi:",phi)
        phi12 =getPhi(distance(p1[i][0],p2[i][0]),distance(p1[i][1],p2[i][1]))
        phi21 = (2 * PI) - phi12
        phi23 = getPhi(distance(p2[i][0],p3[i][0]),distance(p2[i][1],p3[i][1]))
        phi32 = (2 * PI) - phi23
        phi13 = getPhi(distance(p1[i][0],p3[i][0]),distance(p1[i][1],p3[i][1]))
        phi31 = (2 * PI) - phi13

        f12,f13,f23 = force(m1,m2,d12),force(m1,m3,d13),force(m2,m3,d23)
        f1Temp = [sin(theta) * cos(theta), sin(theta) * sin(phi), cos(theta)]
        f1.append([f * (f13 + f12) for f in f1Temp])
        # NOTE: Fic angles theta and phi for f2 f3
        f2Temp = [sin(theta) * cos(theta), sin(theta) * sin(phi), cos(theta)]
        f2.append([f * (f12 + f23) for f in f1Temp])
        f3Temp = [sin(theta) * cos(theta), sin(theta) * sin(phi), cos(theta)]
        f3.append([f * (f13 + f23) for f in f1Temp])


        a1 = acceleration((f13 + f12),m1)
        a2 = acceleration((f23 + f12),m2)
        a3 = acceleration((f13 + f23),m3)

        # v1Temp = velocity(v1[i - 1],a1,dt)
        # v2Temp = velocity(v2[i - 1],a2,dt)
        v3Temp = velocity(v3[i - 1],a3,dt)

        # p1Temp = p1[i - 1] + position(a1,dt,getRad(v1[i - 1]))
        # p2Temp = p2[i - 1] + position(a2,dt,getRad(v2[i - 1]))
        p3Temp = p3[i - 1] + position(a3,dt,getRad(v3[i - 1]))

        # v1.append(list([v1Temp[0]*sin(theta13)*cos(theta13),v1Temp[0]\
        #                 *sin(theta13)*sin(phi13),v1Temp[0]*cos(theta13)]))
        # v2.append(list([v2Temp[0]*sin(theta23)*cos(theta23),v2Temp[0]\
        #                 *sin(theta23)*sin(phi23),v2Temp[0]*cos(theta23)]))
        v3.append(list([v3Temp[0]*sin(theta)*cos(theta),v3Temp[0]\
                        *sin(theta)*sin(phi),v3Temp[0]*cos(theta)]))

        p1.append(list([position(0,tRange[i],v1[0][0]),position(0,v1[0][1],\
                            v1[0][1]),position(0,tRange[i],v1[0][2])]))
        p2.append(list([position(0,tRange[i],v2[0][0]),position(0,tRange[i],\
                            v2[0][1]),position(0,tRange[i],v2[0][2])]))
        p3.append(list([p3Temp[0]*sin(theta)*cos(theta),p3Temp[0]*\
                        sin(theta)*sin(phi),p3Temp[0]*cos(phi)]))
        # p2.append(list([p2Temp[0]*sin(theta23)*cos(theta23),p2Temp[0]*\
        #                 sin(theta23)*sin(phi23),p2Temp[0]*cos(theta23)]))
        # p3.append(list([p3Temp[0]*sin(theta)*cos(theta),p3Temp[0]*sin(theta)\
        #                 *sin(phi),p3Temp[0]*cos(theta)]))



    p1x,p1y,p1z = [i[0] for i in p1],[i[1] for i in p1],[i[2] for i in p1]
    p2x,p2y,p2z = [i[0] for i in p2],[i[1] for i in p2],[i[2] for i in p2]
    p3x,p3y,p3z = [i[0] for i in p3],[i[1] for i in p3],[i[2] for i in p3]

    v1x,v1y,v1z = [i[0] for i in v1],[i[1] for i in v1],[i[2] for i in v1]
    v2x,v2y,v2z = [i[0] for i in v2],[i[1] for i in v2],[i[2] for i in v2]
    v3x,v3y,v3z = [i[0] for i in v3],[i[1] for i in v3],[i[2] for i in v3]

    print(tRange)

    ax = fig.add_subplot(111, projection='3d')
    makePlot(ax,p1x,p1y,p1z,"r")
    makePlot(ax,p2x,p2y,p2z,"g")
    makePlot(ax,p3x,p3y,p3z,"b")
    plt.show()
#===============================================================================
simulation()
# ani = FuncAnimation(fig, simulation, interval=10, blit=True, repeat=True,
#                     frames=np.linspace(0,2*np.pi,360, endpoint=False))
# plt.show()
#===============================================================================

#===============================================================================
#
# """
# GUI
# """
# #Define window, name, and parameters
# window = Tk()
# window.title("Galactic Dynamics")
# window.geometry('450x300')
#
# #Insert text with user input text field.
# lbl1 = Label(window, text = "Mass of Galaxy 1")
# lbl1.grid(column = 0, row = 1)
#
# mass1 = Entry(window, width = 20)
# mass1.grid(column = 1, row = 1)
#
# lbl2 = Label(window, text = "Mass of Galaxy 2 relative to Galaxy 1")
# lbl2.grid(column = 0, row = 2)
#
# mass2 = Entry(window, width = 10)
# mass2.grid(column = 1, row = 2)
#
# lbl3 = Label(window, text = "Galaxy 1 Position eg: x,y,z")
# lbl3.grid(column = 0, row = 3)
#
# pos1 = Entry(window, width = 20)
# pos1.grid(column = 1, row = 3)
#
# lbl4 = Label(window, text = "Galaxy 2 Position eg: x,y,z")
# lbl4.grid(column = 0, row = 4)
#
# pos2 = Entry(window, width = 20)
# pos2.grid(column = 1, row = 4)
#
# lbl5 = Label(window, text = "Star Position eg: x,y,z")
# lbl5.grid(column = 0, row = 5)
#
# pos3 = Entry(window, width = 20)
# pos3.grid(column = 1, row = 5)
#
# lbl6 = Label(window, text = "Galaxy 1 Velocity eg: x,y,z")
# lbl6.grid(column = 0, row = 6)
#
# vel1 = Entry(window, width = 20)
# vel1.grid(column = 1, row = 6)
#
# lbl7 = Label(window, text = "Galaxy 2 Velocity eg: x,y,z")
# lbl7.grid(column = 0, row = 7)
#
# vel2 = Entry(window, width = 20)
# vel2.grid(column = 1, row = 7)
#
# lbl8 = Label(window, text = "Star Velocity eg: x,y,z")
# lbl8.grid(column = 0, row = 8)
#
# vel3 = Entry(window, width = 20)
# vel3.grid(column = 1, row = 8)
#
# lbl9 = Label(window, text = "Total Time")
# lbl9.grid(column = 0, row = 9)
#
# time = Entry(window, width = 20)
# time.grid(column = 1, row = 9)
#
# lbl10 = Label(window, text = "Number of Time Steps")
# lbl10.grid(column = 0, row = 10)
#
# steps = Entry(window, width = 20)
# steps.grid(column = 1, row = 10)
#
# btn1 = Button(window, command = simulation, text = "Submit Values")
# btn1.grid(column = 0, row = 11)
# #loop until closed
# window.mainloop()
#===============================================================================
