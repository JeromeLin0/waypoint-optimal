# -*- coding: utf-8 -*-
import math
import numpy as np
import matplotlib.pyplot as plt
import csv
import copy
from scipy.special import fresnel

MAXCURVATURE = 1/3.

def ClothoidTurn(d, delta):
    if abs(delta) < math.pi/180.:
        curvature = 0
        slop = 0
        L = d
    else:
        t = []
        num = int(1000 * abs(delta))
        for i in range(num):
            t.append(math.sqrt(2*abs(i/1000.)/math.pi))
        ss, cc = fresnel(t)
        d2= math.sqrt(2*math.pi*abs(delta))*(math.cos(delta)*cc[-1] + math.copysign(1.0,delta)*math.sin(delta)*ss[-1])
        curvature = d2 / math.copysign(d,delta)
        slop = curvature*curvature/(2*delta)
        L = abs(curvature / (slop))
        #print(curvature, slop, L)
    return curvature, slop, L

def SKtoXY(d,L,delta,curvature,slop):
    x=[]
    y=[]
    if d == 0:
        return x,y

    unit = 100.
    downSample = 10
    if delta == 0:
        for i in range(int(2*unit*L)):
            x.append(i/unit)
            y.append(0)
    else:
        #---------------
        if abs(curvature) >  MAXCURVATURE:
            curveAngle = 2*delta - np.copysign(1.0, delta)*MAXCURVATURE*L
            if abs(curveAngle) < abs(delta):
                fixCurve = True
        fixCurve = False
        #---------------
        x.append(0)
        y.append(0)

        for i in range(int(unit*L) -1):
            theta = 0.5*(i/unit)*(i/unit)*slop
            x.append((1/unit)*math.cos(theta - delta) + x[-1])
            y.append((1/unit)*math.sin(theta - delta) + y[-1])
         #---------
            if fixCurve:
                heading = math.atan2(y[-1]-y[-2],x[-1]-x[-2]) + np.copysign(math.pi/2, delta)
                centerY = math.tan(heading)*d + y[-2] - math.tan(heading)*x[-2]
                R = math.sqrt((d-x[-2])*(d-x[-2]) + (centerY - y[-2])*(centerY - y[-2]))
                if R - abs(1/MAXCURVATURE) <=0:
                    break

        if fixCurve:
            num = int(unit*(1/MAXCURVATURE)*math.atan(abs(d - x[-1])/abs(centerY - y[-1])))
            startAngle = math.atan2(y[-1] - centerY, x[-1] - d)
            for j in range(num):
                x.append((1/MAXCURVATURE)*math.cos(np.copysign(j,delta)*math.atan(abs(d - x[i])/abs(centerY - y[i]))/num + startAngle)+d)
                y.append((1/MAXCURVATURE)*math.sin(np.copysign(j,delta)*math.atan(abs(d - x[i])/abs(centerY - y[i]))/num + startAngle)+centerY)
        #---------

        tmpx = list(np.copy(x))
        tmpy = list(np.copy(y))
        tmpx.reverse()
        tmpy.reverse()
        #plt.plot(x,y,'r*')
        for i in range(len(tmpx)-1):
            x.append(2*tmpx[0] - tmpx[i+1])
            y.append(tmpy[i+1])
        x = x[::downSample]
        y = y[::downSample]

        if abs(curvature) >  MAXCURVATURE and abs(curveAngle) > abs(delta):
            x = []
            y = []
    return x,y

def PathRotation(x,y,theta,xini,yini):
    rx=[]
    ry=[]
    for i in range(len(x)):
        rx.append(x[i]*math.cos(theta) - y[i]*math.sin(theta) + xini)
        ry.append(x[i]*math.sin(theta) + y[i]*math.cos(theta) + yini)
    return rx,ry


def line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1]) #Typo was here

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
        x = []
        y = []
    else:
        d = (det(*line1), det(*line2))
        x = det(d, xdiff) / div
        y = det(d, ydiff) / div
    return x, y

def FindIntersection(x0, y0, xf, yf, lineAngle0, lineAnglef):
    L = 10
    line1 = [[x0,y0], [x0+L*math.cos(lineAngle0), y0+L*math.sin(lineAngle0)]]
    line2 = [[xf,yf], [xf+L*math.cos(lineAnglef), yf+L*math.sin(lineAnglef)]]
    x, y = line_intersection(line1, line2)
    if x != []:
        vector0 = [line1[1][0] - line1[0][0], line1[1][1] - line1[0][1]]
        vectorf = [line2[1][0] - line2[0][0], line2[1][1] - line2[0][1]]
        vectorp0 = [x - line1[0][0], y - line1[0][1]]
        vectorpf = [x - line2[0][0], y - line2[0][1]]

        if np.dot(vector0,vectorp0) < 0 or np.dot(vectorf,vectorpf) < 0 :
            x = []
            y = []
    return x,y

def GenericTurn(x0, y0, theta0, xf , yf , thetaf, delta0):
    phi = math.atan2(yf-y0, xf-x0)
    def getDegree(x):
        degree = x/math.pi * 180
        while (degree < 0):
            degree += 360
        while degree >360:
            degree -= 360
        return degree/180.0 * math.pi

    theta0 = getDegree(theta0)
    thetaf = getDegree(thetaf)

    lineAngle0 = (theta0 + delta0)
    commonAngle = theta0 + 2*delta0

    if abs(getDegree(theta0 + 2*delta0) - thetaf) > 2*math.pi -  abs(getDegree(theta0 + 2*delta0) - thetaf):
        cAngle = -math.copysign(2*math.pi -  abs(getDegree(theta0 + 2*delta0) - thetaf),
                                getDegree(theta0 + 2*delta0) - thetaf)
    else:
        cAngle = math.copysign(abs(getDegree(theta0 + 2*delta0) - thetaf),getDegree(theta0 + 2*delta0) - thetaf)
    lineAnglef = (cAngle + 2*thetaf) * 0.5 + math.pi


    if theta0 == thetaf:
        intersextionx = 0.5 * (x0 + xf)
        intersectiony = 0.5 * (y0 + yf)
        delta0 = phi - theta0
        deltaf = -phi + theta0
        lineAngle0 = theta0 + delta0
        commonAngle = theta0 + 2*delta0
        lineAnglef = (theta0 + 2*delta0 + thetaf)/2 + math.pi
    else:
        intersextionx, intersectiony = FindIntersection(x0, y0, xf, yf, lineAngle0, lineAnglef)
        deltaf = (-cAngle)/2

    if intersextionx != []:
        d1 = math.sqrt((x0 - intersextionx)*(x0 - intersextionx)+(y0 - intersectiony)*(y0 - intersectiony))/2
        d2 = math.sqrt((xf - intersextionx)*(xf - intersextionx)+(yf - intersectiony)*(yf - intersectiony))/2

        if d1 ==0 or d2==0:
            if delta0 != deltaf:
                d1=0
                d2=0

        curvature, slop, L = ClothoidTurn(d1, delta0)
        Xsegment1, Ysegment1 = SKtoXY(d1,L,delta0,curvature,slop)
        Xsegment1, Ysegment1 = PathRotation(Xsegment1, Ysegment1, lineAngle0, x0, y0)

        curvature, slop, L = ClothoidTurn(d2, deltaf)
        Xsegment2, Ysegment2 = SKtoXY(d2,L,deltaf,curvature,slop)
        Xsegment2, Ysegment2 = PathRotation(Xsegment2, Ysegment2, lineAnglef - math.pi, intersextionx, intersectiony)

        if (Xsegment1 == [] and d1!=0) or (Xsegment2 == [] and d2!=0):
            Xsegment1 = []
            Ysegment1 = []
            Xsegment2 = []
            Ysegment2 = []
    else:
        Xsegment1 = []
        Ysegment1 = []
        Xsegment2 = []
        Ysegment2 = []

    Xsegment1.extend(Xsegment2)
    Ysegment1.extend(Ysegment2)
    return Xsegment1,Ysegment1

def ReadPathFile(file):
    x = []
    y = []
    h = []
    with open(file, mode='r') as csv_file:
        csv_reader = csv.reader(csv_file)
        for row in csv_reader:
            x.append(float(row[0]))
            y.append(float(row[1]))
            h.append(float(row[2]))
    return x,y,h

def GetRadOfCurvature(prevX, prevY, targetX, targetY, nextX, nextY):
    minConst = 0.000000005
    sideA = math.hypot(targetX - prevX, targetY - prevY)
    sideB = math.hypot(nextX - prevX, nextY - prevY)
    sideC = math.hypot(nextX - targetX, nextY - targetY)
    s = (sideA + sideB + sideC) / 2

    if ((s - sideA == 0) or (s - sideB == 0) or (s - sideC == 0)):
       s += minConst

    triangleArea = math.sqrt(math.fabs(s * (s - sideA) * (s - sideB) * (s - sideC)))
    radius = (sideA * sideB * sideC) / (4 * triangleArea)
    return radius

def GetCurvatureOfPath(x,y):
    curvature = []
    curvature.append(0)
    for i in range(len(x)-2):
        # print(i)
        curvature.append(1/GetRadOfCurvature(x[i], y[i], x[i+1], y[i+1], x[i+2], y[i+2]))
    curvature.append(0)
    return curvature

def running_mean(l, N):
    sum = 0
    result = list( 0 for x in l)
    for i in range( 0, N ):
        sum = sum + l[i]
        result[i] = sum / (i+1)
    for i in range( N, len(l) ):
        sum = sum - l[i-N] + l[i]
        result[i] = sum / N
    return result

# def CHeading()
if __name__ == "__main__":
    d = 40

    angle = 1*math.pi/2
    origin = 0
    filter1 = 0
    filter1andfiler2 = 0
    for t in range(-18, 18):
        print("=======================")
        x,y = GenericTurn(-2988043.077,
                        4964734.5690000001,
                        221.4079965393,
                        -2988045.9639999997,
                        4964733.9289999995,
                        36.5696164492,
                        5*t*math.pi/180)
        plt.plot(x,y,'m')
    plt.show()
