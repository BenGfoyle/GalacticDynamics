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
from dynamicsFormula import distance,distanceCoord,position,acceleration,\
velocity,force,makePlot,getRad,getTheta,getPhi

fig = plt.figure()
sin = np.sin; cos = np.cos;

#==============================================================================
def fileRead():
    """
    Overview: This function reads the text file pendulum.txt and passes two
    lists back to the main method
    """

    #define lists and and file namet(
    inFile = open(viewFile.get(),'r')
    xData,yData = [],[]
    col = viewCol.get().split(",")
    xCol,yCol = int(col[0]), int(col[1])

    header = inFile.readline()
    header = header.split(",")
    #get values from each line
    for line in inFile:
        values = line.split()
        xData.append(float(values[xCol]))
        yData.append(float(values[yCol]))

    #close file
    inFile.close()
    print(header)
    print(xData)
    print(yData)
    plt.plot(xData,yData)
    plt.xlabel(header[xCol])
    plt.ylabel(header[yCol])
    plt.show()
#==============================================================================

#==============================================================================
def fileWrite(fx,fy,fz,ft,vx,vy,vz,vt,px,py,pz,time):
    """
    Overview: This function writes two arrays to the file pendulum.txt
    """
    import csv
    #name and open file
    outFile = open(saveAs.get(),'w')
    print("{0:>30}".format("Force(x),"),\
          "{0:>30}".format("Force(y),"),\
          "{0:>30}".format("Force(z),"),\
          "{0:>30}".format("Force(Net),"),\
          "{0:>30}".format("Velocity(x),"),\
          "{0:>30}".format("Velocity(y),"),\
          "{0:>30}".format("Velocity(z),"),\
          "{0:>30}".format("Velocity(Net),"),\
          "{0:>30}".format("Position(x),"),\
          "{0:>30}".format("Position(y),"),\
          "{0:>30}".format("Position(z),"),\
          "{0:>30}".format("Time,"), file = outFile)
    #print lsits to file
    for i in range(0, len(fx)):
        print("{0:>30}".format(fx[i]),\
              "{0:>30}".format(fy[i]),\
              "{0:>30}".format(fz[i]),\
              "{0:>30}".format(ft[i]),\
              "{0:>30}".format(vx[i]),\
              "{0:>30}".format(vy[i]),\
              "{0:>30}".format(vz[i]),\
              "{0:>30}".format(vt[i]),\
              "{0:>30}".format(px[i]),\
              "{0:>30}".format(py[i]),\
              "{0:>30}".format(pz[i]),\
              "{0:>30}".format(time[i]), file = outFile)

    #close file
    outFile.close()
#==============================================================================


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
    initP1 = list(float(s) for s in initP1.strip("()").split(","))
    initP2 = list(float(s) for s in initP2.strip("()").split(","))
    p3.append(list(float(s) for s in initP3.strip("()").split(",")))

    #list of lists which will hold positional components of each body
    v1,v2,v3 = [],[],[]

    initV1,initV2,initV3  = vel1.get(),vel2.get(),vel3.get()
    v1.append(list(float(s) for s in initV1.strip("()").split(",")))
    v2.append(list(float(s) for s in initV2.strip("()").split(",")))
    v3.append(list(float(s) for s in initV3.strip("()").split(",")))

    for t in tRange:
        #assume no galactic acceleration on each other
        p1.append([initP1[0] +position(0,t,v1[0][0]),initP1[1] + \
                position(0,t,v1[0][1]),initP1[2] + position(v1[0][2],0,t)])
        p2.append([initP2[0] + position(0,t,v2[0][0]),initP2[1] + \
                position(0,t,v2[0][1]),initP2[2] + position(0,t,v2[0][2])])

    f1,f2,f3 = [],[],[]

    for i in range(0,len(tRange)):
        print("\n***********************")
        print(i)
        d12,d13,d23 = distance(p1[i],p2[i]),distance(p1[i],p3[i]),\
                      distance(p2[i],p3[i])



        #Center of mass
        comX = ((m1 * np.array(p1[i][0])) + (m2*np.array(p2[i][0]))) / (m1 + m2)
        comY = ((m1 * np.array(p1[i][1])) + (m2*np.array(p2[i][1]))) / (m1 + m2)
        comZ = ((m1 * np.array(p1[i][2])) + (m2*np.array(p2[i][2]))) / (m1 + m2)
        com = [comX,comY,comZ]

        theta = getTheta(distance(com[2],p3[i][2]), getRad(distanceCoord(com,p3[i])))
        phi = getPhi(distance(com[0],p3[i][0]), distance(com[1],p3[i][1]))

        f12,f13,f23 = force(m1,m2,d12),force(m1,m3,d13),force(m2,m3,d23)

        f3Temp = [sin(theta)*cos(theta), sin(theta)*sin(phi), cos(theta)]
        f3.append([f * (f13 + f23) for f in f3Temp])


        # f31Temp = [sin(theta1)*cos(theta1), sin(theta1)*sin(phi1), cos(theta1)]
        # f31Temp = [f * f13 for f in f31Temp]
        # f32Temp = [sin(theta2)*cos(theta2), sin(theta2)*sin(phi2), cos(theta2)]
        # f32Temp = [f * f23 for f in f32Temp]

        a3 = [acceleration((f3[i][0]),m3),acceleration((f3[i][1]),m3),\
              acceleration((f3[i][2]),m3)]

        v3Temp = [velocity(v3[i-1][0],a3[0],dt),velocity(v3[i-1][1],a3[1],dt)\
                 ,velocity(v3[i-1][2],a3[2],dt)]

        p3Temp = [p3[i - 1][0] + position(a3[0],dt,v3[i - 1][0]),\
                  p3[i - 1][1] + position(a3[1],dt,v3[i - 1][1]),\
                  p3[i - 1][2] + position(a3[2],dt,v3[i - 1][2])]

        v3.append(v3Temp)
        p3.append(p3Temp)

    p1x,p1y,p1z = [i[0] for i in p1],[i[1] for i in p1],[i[2] for i in p1]
    p2x,p2y,p2z = [i[0] for i in p2],[i[1] for i in p2],[i[2] for i in p2]
    p3x,p3y,p3z = [i[0] for i in p3],[i[1] for i in p3],[i[2] for i in p3]

    v1x,v1y,v1z = [i[0] for i in v1],[i[1] for i in v1],[i[2] for i in v1]
    v2x,v2y,v2z = [i[0] for i in v2],[i[1] for i in v2],[i[2] for i in v2]
    v3x,v3y,v3z = [i[0] for i in v3],[i[1] for i in v3],[i[2] for i in v3]

    f1x,f1y,f1z = [i[0] for i in f1],[i[1] for i in f1],[i[2] for i in f1]
    f2x,f2y,f2z = [i[0] for i in f2],[i[1] for i in f2],[i[2] for i in f2]
    f3x,f3y,f3z = [i[0] for i in f3],[i[1] for i in f3],[i[2] for i in f3]

    print(v3)
    print(f3)
    fNet = [(x)**2+(y)**2+(z)**2 for x,y,z in [*f3]]
    vNet = [(x)**2+(y)**2+(z)**2 for x,y,z in [*v3]]

    fileWrite(f3x,f3y,f3z,fNet,v3x,v3y,v3z,vNet,p3x,p3y,p3z,tRange)

    ax = fig.add_subplot(111, projection='3d')
    makePlot(ax,p1x,p1y,p1z,"r")
    makePlot(ax,p2x,p2y,p2z,"g")
    makePlot(ax,p3x,p3y,p3z,"b")
    plt.show()
#===============================================================================

#===============================================================================

"""
GUI
"""
#Define window, name, and parameters
window = Tk()
window.title("Galactic Dynamics")
window.geometry('450x350')

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

lbl11 = Label(window, text = "Save Data As:")
lbl11.grid(column = 0, row = 11)
saveAs = Entry(window, width = 20)
saveAs.grid(column = 1, row = 11)

btn1 = Button(window, command = simulation, text = "Submit Values")
btn1.grid(column = 0, row = 13)

lbl12 = Label(window, text = "File to analyse eg: plutoData.txt")
lbl12.grid(column = 0, row = 15)

viewFile = Entry(window, width = 20)
viewFile.grid(column = 1, row = 15)

lbl13 = Label(window, text = "Index of columns to compare eg:0,7")
lbl13.grid(column = 0, row = 16)

viewCol = Entry(window, width = 20)
viewCol.grid(column = 1, row = 16)

btn1 = Button(window, command = fileRead, text = "Construct Plot")
btn1.grid(column = 0, row = 18)

#loop until closed
window.mainloop()
#===============================================================================
