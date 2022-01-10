#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jan  9 12:31:54 2022

@author: liourenshiuan
"""


import numpy as np

T = 10 
x1Input = [0 ,0 ,25 ,0]
x2Input = [20 ,4 ,25 ,0]
deltaS_plum1 = [-10,-5,0,5,10]
deltaD1 = 4
deltaD2 = [0,4]
WD = 1
R = 10


def possibleManeuvers(x0, deltaS_plum, deltaD):
    #This function will output all the meneuvers of a car,
    #it depends on the initial state and the change of velocity and lane coordinate
    
    meneuvers= []
    
    # solve b = A x which x is sigma
    A = np.array([[4 * (T**3), 3 * (T**2)], [12 * (T**2), 6 * T]])
    # solve c = B y which c is delta
    B = np.array([[20 * (T**3), 12 * (T**2), 6 * T], 
                  [T**5, T**4, T**3], 
                  [5 * (T**4), 4 * (T**3), 3 * (T**2)]])
    c = np.array([0 ,deltaD ,0])
    y = np.linalg.inv(B).dot(c)
    for i in deltaS_plum:
        b = np.array([i ,0])
        x = np.linalg.inv(A).dot(b)
        meneuvers.append([[x0[0] ,x0[2] ,x[0] ,x[1]], 
                          [x0[1], y[2], y[1], y[0]]])
    return meneuvers

def cost(maneuver):
    m_long = maneuver[0]
    m_lat = maneuver[1]

    f_long = 1 / 1 + WD
    f_lat = WD/ 1 + WD

    # sigma3 = m_long[2], sigma4 = m_long[3]
    cost_long = f_long * ( ( 144 / 5 * np.power(T, 5) * np.power(m_long[3], 2) )  + ( 36 * np.power(T, 4) * m_long[2] * m_long[3]) + (12 * np.power(T,3) * np.power(m_long[2], 2)))
    # delta3 = m_lat[1], delta4 = m_lat[2], delta5 = m_lat[3]
    cost_lat = f_lat * ( ( 400 / 7 * np.power(T, 7) * np.power(m_lat[3], 2)) + (144 / 5 * np.power(T, 5) * np.power(m_lat[2], 2)) + (36 / 3 * np.power(T, 3)* np.power(m_lat[1], 2)) 
                        + (80 * np.power(T, 6) * m_lat[3]* m_lat[2]) + (24 * np.power(T, 5) * m_lat[3] * m_lat[1]) + (18 * np.power(T, 4) * m_lat[2] * m_lat[1]))
    
    return cost_long + cost_lat

def collision(maneuver1, maneuver2, t):
    # This method is to see whether two vehicle will collide at time t 
    # position of car1 at time t
    m_long_1 = maneuver1[0]
    m_lat_1 = maneuver1[1]
    s1 = m_long_1[3] * np.power(t,4) + m_long_1[2] * np.power(t,3) + m_long_1[2] * t + m_long_1[0]
    d1 = m_lat_1[3] * np.power(t,5) + m_lat_1[2] * np.power(t,4) + m_lat_1[2] * np.power(t, 3) + m_lat_1[0]
    # position of car2 at time t
    m_long_2 = maneuver2[0]
    m_lat_2 = maneuver2[1]
    s2 = m_long_2[3] * np.power(t,4) + m_long_2[2] * np.power(t,3) + m_long_2[2] * t + m_long_2[0]
    d2 = m_lat_2[3] * np.power(t,5) + m_lat_2[2] * np.power(t,4) + m_lat_2[2] * np.power(t, 3) + m_lat_2[0]
    
    if(np.power((s1 - s2), 2) + np.power((d1 - d2), 2)) < np.power(2 * R, 2):
        return True
    return False

def optimal(maneuver1_list, maneuver2_list):
    # FThis method will return the best meneuver for both vehicle
    
    opt_m1 = []
    opt_m2 = []
    min_cost = np.inf
    
    for maneuver1 in maneuver1_list:
        for maneuver2 in maneuver2_list:
            c = False
            for t in range(1, T + 1):
                
                if collision(maneuver1, maneuver2, t):
                    c = True
                    break
            if (not c) and (cost(maneuver1) + cost(maneuver2))/10 < min_cost:
                    min_cost = (cost(maneuver1) + cost(maneuver2))/10
                    
                    opt_m1 = maneuver1
                    opt_m2 = maneuver2
                    
    return opt_m1, opt_m2 ,min_cost ,cost(maneuver2)




x1Possible = possibleManeuvers(x1Input, deltaS_plum1, deltaD1)
x2Possible = []
for i in deltaD2:
    x2Possible = x2Possible + possibleManeuvers(x2Input, deltaS_plum1, i)

print(optimal(x1Possible, x2Possible))