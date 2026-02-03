# -*- coding: utf-8 -*-
"""
Created on Mon Mar 17 10:15:40 2025

@author: kopuj
"""

import matplotlib.pyplot as plt
import math as mth

ax = 0
ay = -9.81 # m/s^2 (painovoiman kiihtyvyys)
kax = 0
kay = 0



dt = 0.01 # s (laskennan aikaväli)
k = 0.0014 # kg/m (ilmanvastus)
m = 0.16 # kg (partikkelin paino)

xlist = [0.0]
ylist = [2.0] # m (partikkelin lähtösijainti)

vax = 10.0 # m/s (partikkelin lähtönopeus)
vay = 7.0 # m/s

while ylist[-1] > 0:
    kax = ax - (k/m) * (mth.sqrt(vax**2 + vay**2)) * vax
    kay = ay - (k/m) * (mth.sqrt(vax**2 + vay**2)) * vay

    vlx = vax + kax*dt# päivitetään nopeus
    vly = vay + kay*dt
    xlist.append(xlist[-1] + vlx*dt) # päivitetään sijainti
    ylist.append(ylist[-1] + vly*dt)
    vax = vlx # loppunopeus = seuraavan aikavälin lähtönopeus
    vay = vly
    
    
plt.plot(xlist,ylist,'o')
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.show()