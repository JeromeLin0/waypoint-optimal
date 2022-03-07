import numpy as np
import matplotlib.pyplot as plt
import json
from wp_reg import *
from visualize import *

with open('data/waypoints.json') as json_file:
    label = json.load(json_file)

with open('data/car_state_201911281756.json') as json_file:
    human = json.load(json_file)

waypointsdict = label['waypoints']
labelx = list()
labely = list()
for lane in waypointsdict:
    if lane['lane_id'] in [173, 362, 149]:
        for i in lane['points']:
            labelx.append(i['x'])
            labely.append(i['y'])
labelx = np.array(labelx)
labely = np.array(labely)

poseX = list()
poseY = list()
poseZ = list()
for i in human['car_state']:
    poseX.append(i['pose']['pose']['position']['x'])
    poseY.append(i['pose']['pose']['position']['y'])
    poseZ.append(i['pose']['pose']['position']['z'])
xo = np.array(poseX)
yo = np.array(poseY)
zo = np.array(poseZ)

dist = np.hypot(np.diff(xo), np.diff(yo))
dist = np.cumsum(dist)
frenetS = np.concatenate(([0.], dist))

x = xo[0]
y = yo[0]
z = zo[0]

lanes = dict()
lanes[0] = dict()
lanes[0]['x'] = [x]
lanes[0]['y'] = [y]
lanes[0]['z'] = [z]

lookahead = 5.0
S = 0
window = 100
diff_ang_old = None
#while float(len(lanes[0]['x'])) < frenetS[-1] and np.hypot(xo[-1] - x, yo[-1] - y) > 0.5 * 1.0:
for i in range(500):
    S = S + lookahead
    ind = np.argmin(np.abs(frenetS - S))

    diff_ang = np.arctan2(yo[ind]-y,xo[ind]-x)
    diff_dist = np.hypot(yo[ind]-y,xo[ind]-x)
    if diff_ang_old != None:
        if np.cos(diff_ang - diff_ang_old) > 0.8:
            diff_ang_old = diff_ang
            x += np.cos(diff_ang)
            y += np.sin(diff_ang)
            lanes[0]['x'].append(x)
            lanes[0]['y'].append(y)
            lanes[0]['z'].append(zo[ind])
            if diff_dist > 2.0:
                lookahead = 0.
            else:
                lookahead = 1.0
        else:
            lookahead = np.minimum(lookahead + 1.0, frenetS[-1] - S)
    else:
        diff_ang_old = diff_ang

s = np.cumsum((np.diff(lanes[0]['x'])**2 + np.diff(lanes[0]['y'])**2)**0.5)
s = np.append(0., s)
lanes[0]['s'] = s
lanes[0]['prev'] = []
lanes[0]['next'] = []
set_curv_xy(lanes[0])

all_lane_id = [0]
plt.figure('vis_curv')
visualize_curv(lanes, all_lane_id)
plt.xlabel('distance(m)')
plt.ylabel('curvature(1/m)')
plt.figure('vis_s')
plt.xlabel('distance(m)')
plt.ylabel('interval(m)')
visualize_s(lanes, all_lane_id)
plt.figure('vis_xy')
plt.xlabel('X(m)')
plt.ylabel('Y(m)')
plt.grid()
visualize_xy(lanes, all_lane_id)
plt.plot(xo, yo,'k.-')

waypoints_regulation_fix_start_end(1.0, lanes, [0])

plt.figure('vis_curv')
plt.gca().set_prop_cycle(None)
visualize_curv(lanes, all_lane_id, True)
plt.figure('vis_s')
plt.gca().set_prop_cycle(None)
visualize_s(lanes, all_lane_id, True)
plt.figure('vis_xy')
plt.gca().set_prop_cycle(None)
visualize_xy(lanes, all_lane_id, True, True)


sss = []
xxyy = []
for xi, yi in zip(xo, yo):
    dx = xi - np.array(lanes[0]['x'])
    dy = yi - np.array(lanes[0]['y'])
    idx = np.argmin(np.hypot(dx, dy))
    h = lanes[0]['h'][idx]
    ds = dx[idx] * np.cos(h) + dy[idx] * np.sin(h)
    sss.append(lanes[0]['s'][idx] + ds)
    xxyy.append((xi, yi))

sss, xxyy = (list(t) for t in zip(*sorted(zip(sss, xxyy))))

lanes[0]['splxx'] = np.array(sss)
xx = []
yy = []
for xxyyi in xxyy:
    xx.append(xxyyi[0])
    yy.append(xxyyi[1])

lanes[0]['sply_xy'] = np.array(xx)
lanes[0]['sply_yy'] = np.array(yy)

waypoints_regulation_fix_start_end2(1.0, lanes, [0])

plt.figure('vis_xy')
plt.gca().set_prop_cycle(None)
plt.plot(lanes[0]['x'], lanes[0]['y'], 'r.-')

plt.show()
