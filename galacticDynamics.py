"""
Author: Ben Guilfoyle - github.com/bengfoyle
Overview: Calculating the gravitational efferty of the close passage of two
glaactic nuclei on a disk of stars. Done as part of Maynooth University
BSc Physics with Astrophysics, Module EP408.
"""

import numpy as np
from tkinter import *



#===============================================================================
def makePlot():
    """
    Overview: Make a plot of results
    """
    return
#===============================================================================

#===============================================================================
def position(p,v):
    """
    Overview: Calcualte position of body over a change in time
    """
    return p + (v * dt)
#===============================================================================

#===============================================================================
def distance():
    """
    Overview: Calculate distance between two bodies
    """
    return
#===============================================================================

#===============================================================================
def acceleration(vA,vB):
    """
    Overview: Calcualte acceleration of a body
    """
    return (vA - vB) / dt
#===============================================================================

#===============================================================================
def velocity(pA,pB):
    """
    Overview: Calcualte the velocity of a body
    """
    return (pA - pB) / dt
#===============================================================================

#===============================================================================
def pytha(x1,x2,y1,y2,z1,z2):
    return np.sqrt((x1 - x2)**2 + (y1 - y2)**2 + (z1 - z2)**2)
#===============================================================================

#===============================================================================
def simulation():
    """
    Overview: Run a simulation of galactic dynamics for a given system
    """
    #lists of inital values
    initMass = [txt1.get(),txt2.get()]
    initPos = [txt3.get(),txt4.get(),txt5.get()]
    initVel = [txt6.get(),txt7.get(),txt8.get()]
    initTime = txt9.get()
    dt = txt10.get()

    #lists to hold updated positions and velocities
    xPos, yPos, zPos, nPos = []
    xVel, yVel, zVel, nVel = []

    #adding inital values to lsits
    xPos.append(initPos[0]), yPos.append(initPos[1]), zPos.append(initPos[2])
    xVel.append(initVel[0]), yVel.append(initVel[1]), zVel.append(initVel[2])
    nPos.append(pytha(xPos[0],0,yPos[0],0,zPos[0],0))
    nVel.append(pytha(xVel[0],0,yVel[0],0,zVel[0],0))

    #list of times
    time = list(np.arange(0,initTime,dt))

    for t in range(0,len(time)):
        xPos.append(initPos[0]), yPos.append(initPos[1]), zPos.append(initPos[2])
        xVel.append(initVel[0]), yVel.append(initVel[1]), zVel.append(initVel[2])
        nPos.append(pytha(xPos[t],xPos[t-1],yPos[t],yPos[t-1],zPos[t],zPos[t-1]))
        nVel.append(pytha(xVel[t],xVel[t-1],yVel[t],yVel[t-1],zVel[t],zVel[t-1]))

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

txt1 = Entry(window, width = 20)
txt1.grid(column = 1, row = 1)

lbl2 = Label(window, text = "Mass of Galaxy 2 relative to Galaxy 1")
lbl2.grid(column = 0, row = 2)

txt2 = Entry(window, width = 10)
txt2.grid(column = 1, row = 2)

lbl3 = Label(window, text = "Galaxy 1 Position eg: x,y,z")
lbl3.grid(column = 0, row = 3)

txt3 = Entry(window, width = 20)
txt3.grid(column = 1, row = 3)

lbl4 = Label(window, text = "Galaxy 2 Position eg: x,y,z")
lbl4.grid(column = 0, row = 4)

txt4 = Entry(window, width = 20)
txt4.grid(column = 1, row = 4)

lbl5 = Label(window, text = "Star Position eg: x,y,z")
lbl5.grid(column = 0, row = 5)

txt5 = Entry(window, width = 20)
txt5.grid(column = 1, row = 5)

lbl6 = Label(window, text = "Galaxy 1 Velocity eg: x,y,z")
lbl6.grid(column = 0, row = 6)

txt6 = Entry(window, width = 20)
txt6.grid(column = 1, row = 6)

lbl7 = Label(window, text = "Galaxy 2 Velocity eg: x,y,z")
lbl7.grid(column = 0, row = 7)

txt7 = Entry(window, width = 20)
txt7.grid(column = 1, row = 7)

lbl8 = Label(window, text = "Star Velocity eg: x,y,z")
lbl8.grid(column = 0, row = 8)

txt8 = Entry(window, width = 20)
txt8.grid(column = 1, row = 8)

lbl9 = Label(window, text = "Total Time")
lbl9.grid(column = 0, row = 9)

txt9 = Entry(window, width = 20)
txt9.grid(column = 1, row = 9)

lbl10 = Label(window, text = "Time Steps")
lbl10.grid(column = 0, row = 10)

txt10 = Entry(window, width = 20)
txt10.grid(column = 1, row = 10)

btn1 = Button(window, command = simulation, text = "Submit Values")
btn1.grid(column = 0, row = 11)
#loop until closed
window.mainloop()
#===============================================================================
