# -*- coding: utf-8 -*-
"""
Created on Mon Mar 17 10:15:40 2025

@author: kopuj
"""

import matplotlib.pyplot as plt
import math as mth
import numpy as np
ax = 0
ay = -9.81 # m/s^2 (painovoiman kiihtyvyys)
kax = 0 #
kay = 0

e = 0.8 #sysäyskerroin

dt = 0.2 # s (laskennan aikaväli)
k = 0.0014 # kg/m (ilmanvastus)
m = 0.16 # kg (partikkelin paino)

shapeList = [(3,0),(-2,2),(-2,-2),(0,-3)] # [(x,y)]

w = 90 * (mth.pi/180) # rad/s (kulmanopeus)
dphi = dt * w # rad (kulma-aseman muutos)
phi = 0 # rad (kulma-asema)

xlist = [0]
ylist = [20.0] # m (partikkelin lähtösijainti)
sx = []
sy = []

J = m*(1.8)**2 # kolmion hitausmomentti

for x, y in shapeList:
    xR = (x * mth.cos(phi)) - (y * mth.sin(phi))
    yR = (y * mth.sin(phi)) + (y * mth.cos(phi))
    sx.append(xR + xlist[-1])
    sy.append(yR + ylist[-1])
sx.append(sx[0])
sy.append(sy[0])
plt.plot(sx, sy, color='steelblue')  # monikulmio 1

vax = 10 # m/s (partikkelin lähtönopeus)
vay = 0 # m/s

loop = True
timer = 10

while loop:
    dphi = dt * w # rad (kulma-aseman muutos)
    timer -= dt
    if timer <= 0:
        loop = False
    if xlist[-1] > 60 or xlist[-1] < -20 or ylist[-1] < -10:
        loop = False
    phi += dphi
    kax = ax - (k/m) * (mth.sqrt(vax**2 + vay**2)) * vax # ilmanvastus
    kay = ay - (k/m) * (mth.sqrt(vax**2 + vay**2)) * vay

    vlx = vax + kax*dt# päivitetään nopeus
    vly = vay + kay*dt
    xlist.append(xlist[-1] + vlx*dt) # päivitetään sijainti
    ylist.append(ylist[-1] + vly*dt)
    sx = []
    sy = []
    for x, y in shapeList:
        xR = (x * mth.cos(phi)) - (y * mth.sin(phi))
        yR = (x * mth.sin(phi)) + (y * mth.cos(phi))
        sx.append(xR + xlist[-1])
        sy.append(yR + ylist[-1])
    sx.append(sx[0])
    sy.append(sy[0])
    for i in range(0, len(sy)-1):
        floor_y = sx[i]*mth.tan(mth.radians(25))
        plt.plot([sx[i]],[floor_y],'o', color='orange')
        # pisteen paikkavektori suhteessa monikulmion keskipisteeseen ->
        rp = [sx[i] - xlist[-1], sy[i] - ylist[-1]]
        vp = [(-w * rp[1]) + vlx, w * rp[0] + vly] # Vcm + w x rp
        #plt.plot(sx[i],sy[i],'o', color='black')
        if sy[i] < floor_y and vp[1] < 0: # Törmäystarkistus
            plt.plot([sx[i]],[sy[i]],'o', color='green')
            
            t = [sx[i], floor_y]
            nV = [floor_y, -sx[i]]
            nL = mth.sqrt(nV[0]**2 + nV[1]**2)
            n = [nV[0]/nL, nV[1]/nL]
            
            #impulssin suuruus
            vn = np.dot(vp, n)
            vector_cross = np.cross(rp, n)
            bottom_part = 1/m+((vector_cross)**2)/J
            impulse = -(1+e)*(vn/bottom_part)
            # Nopeuden muutos
            vly = vay + impulse/m * n[1]
            vlx = vax + impulse/m * n[0]
            
            #kulmanopeuden muutos
            w = (w + (impulse/J)*(np.cross(rp, n)))
            break
        else:
            pass
    vax = vlx # loppunopeus = seuraavan aikavälin lähtönopeus
    vay = vly
    print(w)
    plt.plot(sx, sy, color='steelblue')  # monikulmio

#plt.plot(xlist,ylist,'o', color='royalblue') # keskipiste
plt.plot(xlist,ylist, color='royalblue') # keskipiste
plt.xlabel("x (m)")
plt.ylabel("y (m)")
axe = plt.gca()
axe.set_aspect('equal', adjustable='box')
plt.axline((0, 0), slope=mth.tan(mth.radians(25)), color='red', label='axline')
plt.show()