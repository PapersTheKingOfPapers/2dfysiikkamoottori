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

e = -1 #sysäyskerroin

dt = 0.1 # s (laskennan aikaväli)
k = 0.0014 # kg/m (ilmanvastus)
m = 0.16 # kg (partikkelin paino)

shapeList = [(-2,-2),(-2,2),(3,0)] # [(x,y)]

w = 270 * (mth.pi/180) # rad/s (kulmanopeus)
dphi = dt * w # rad (kulma-aseman muutos)
phi = 0 # rad (kulma-asema)

xlist = [0.0]
ylist = [8.0] # m (partikkelin lähtösijainti)

sx = []
sy = []

J = m*(xlist[-1]-(shapeList[0])[0]) # kolmion hitausmomentti

for x, y in shapeList:
    xR = (x * mth.cos(phi)) - (y * mth.sin(phi))
    yR = (y * mth.sin(phi)) + (y * mth.cos(phi))
    sx.append(xR + xlist[-1])
    sy.append(yR + ylist[-1])
sx.append(sx[0])
sy.append(sy[0])
plt.plot(sx, sy, color='steelblue')  # monikulmio 1

vax = 5.0 # m/s (partikkelin lähtönopeus)
vay = 8.0 # m/s

loop = True
timer = 20

while loop:
    timer -= dt
    if timer <= 0:
        loop = False
    if xlist[-1] > 60:
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
    for i in sy:
        floor_y = sx[-1]*mth.tan(mth.radians(-25)) 
        # pisteen paikkavektori suhteessa monikulmion keskipisteeseen ->
        rp = [sx[-1] - xlist[-1], i - ylist[-1]]
        vp = [(-w * rp[1]) + vlx, w * rp[0] + vly] # Vcm + w x rp
        if i < floor_y and vp[1] < 0: # Törmäystarkistus
            n_len = mth.sqrt(sx[-1]**2+floor_y**2)
            n = [-floor_y/n_len, sx[-1]/n_len]
            #impulssin suuruus
            vn = np.dot(vp, n)
            vector_cross = np.cross(rp, n)
            vector_len = mth.sqrt(vector_cross**2)
            bottom_part = 1/m+((vector_len)**2)/J
            impulse = -(1-e)*(vn/bottom_part)
            # Nopeuden muutos
            vly = vay + impulse/m * n[1]
            vlx = vax + impulse/m * n[0]
            
            #kulmanopeuden muutos
            w = w + (impulse/J)*(np.cross(rp, n))
            
    vax = vlx # loppunopeus = seuraavan aikavälin lähtönopeus
    vay = vly
    plt.plot(sx, sy, color='steelblue')  # monikulmio

#plt.plot(xlist,ylist,'o', color='royalblue') # keskipiste
plt.xlabel("x (m)")
plt.ylabel("y (m)")
axe = plt.gca()
axe.set_aspect('equal', adjustable='box')
plt.axline((0, 0), slope=mth.tan(mth.radians(-25)), color='red', label='axline')
plt.show()