# -*- coding: utf-8 -*-
"""
Created on Mon Mar 17 10:15:40 2025

@author: kopuj
"""

import matplotlib.pyplot as plt
import math as mth

ax = 0
ay = -9.81 # m/s^2 (painovoiman kiihtyvyys)
kax = 0 #
kay = 0

dt = 0.1 # s (laskennan aikaväli)
k = 0.0014 # kg/m (ilmanvastus)
m = 0.16 # kg (partikkelin paino)

shapeList = [(-2,-2),(-2,2),(3,0)] # [(x,y)]

w = 90 * (mth.pi/180) # rad/s (kulmanopeus)
dphi = dt * w # rad (kulma-aseman muutos)
phi = 0 # rad (kulma-asema)

xlist = [0.0]
ylist = [5.0] # m (partikkelin lähtösijainti)

sx = []
sy = []
for i in shapeList:
    R = (i[0] * mth.cos(phi)) - (i[1] * mth.sin(phi))
    sx.append(R + xlist[0])
sx.append(sx[0])
for i in shapeList:
    R = (i[0] * mth.sin(phi)) + (i[1] * mth.cos(phi))
    sy.append(R + ylist[0])
sy.append(sy[0])
plt.plot(sx, sy)  # monikulmio

vax = 10.0 # m/s (partikkelin lähtönopeus)
vay = 20.0 # m/s

loop = True

while loop:
    if ylist[-1] < 0:
        loop = False
    phi += dphi
    kax = ax - (k/m) * (mth.sqrt(vax**2 + vay**2)) * vax
    kay = ay - (k/m) * (mth.sqrt(vax**2 + vay**2)) * vay

    vlx = vax + kax*dt# päivitetään nopeus
    vly = vay + kay*dt
    xlist.append(xlist[-1] + vlx*dt) # päivitetään sijainti
    ylist.append(ylist[-1] + vly*dt)
    sx = []
    sy = []
    for i in shapeList:
        R = (i[0] * mth.cos(phi)) - (i[1] * mth.sin(phi))
        sx.append(R + xlist[-1])
    sx.append(sx[0])
    for i in shapeList:
        R = (i[0] * mth.sin(phi)) + (i[1] * mth.cos(phi))
        sy.append(R + ylist[-1])
    sy.append(sy[0])
    for i in sy:
        if i < 0 and vly < 0:
            plt.plot(sx, sy)  # monikulmio
            loop = False

    vax = vlx # loppunopeus = seuraavan aikavälin lähtönopeus
    vay = vly
    plt.plot(sx, sy)  # monikulmio

plt.plot(xlist,ylist,'o') # keskipiste
plt.xlabel("x (m)")
plt.ylabel("y (m)")
axe = plt.gca()
axe.set_aspect('equal', adjustable='box')
plt.axhline(y=0, color='r', linestyle='-')
plt.show()