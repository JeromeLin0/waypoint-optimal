#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import math
from create_map_function import *
from replan_waypoints import *

FileName = 'data/new_waypoints1.json'

human_waypoint = waypoint_opt(FileName)

with open('data/waypoints.json', 'r') as f:
    data = json.load(f)
waypointsdict = data['waypoints']

'''create new map'''
newWaypoint = ReplanWaypoint(waypointsdict, human_waypoint)
newWaypoint.set_OriginalLaneId([173, 362, 149])
newWaypoint.set_CutPointsId([dict(lane_id = 173, point_id = 107), dict(lane_id = 149, point_id = 5)])
replan_id, new_lanes = newWaypoint.replan_waypoint()

'''view old waypoints'''
labelx, labely = list(), list()
lane173, lane362, lane149 = [], [], []
for lane in waypointsdict:
    if lane['lane_id'] == 173:
        lane173 = lane['points']
    elif lane['lane_id'] == 362:
        lane362 = lane['points']
    elif lane['lane_id'] == 149:
        lane149 = lane['points']
labelx = []
labely = []
for i in lane173:
    labelx.append(i['x'])
    labely.append(i['y'])
plt.plot(labelx, labely, 'r.-', label='lane 173')
labelx = []
labely = []
for i in lane362:
    labelx.append(i['x'])
    labely.append(i['y'])
plt.plot(labelx, labely, 'g.-', label='lane 362')
labelx = []
labely = []
for i in lane149:
    labelx.append(i['x'])
    labely.append(i['y'])
plt.plot(labelx, labely, 'b.-', label='lane 149')
plt.legend(('new lane 173', 'new lane 362', 'new lane 149', 'lane 173', 'lane 362', 'lane 149'), loc='upper left')

'''output json'''
out_lanes = []
for ori_lane in waypointsdict:
    out_points = []
    if ori_lane['lane_id'] in replan_id:
        ori_point = ori_lane['points']
        lane = [lane for lane in new_lanes if lane['lane_id'] == ori_lane['lane_id']][0]
        for i, (_x, _y, _z, heading, curve) in enumerate(zip(lane['x'], lane['y'], lane['z'], lane['heading'], lane['curve'])):
            if i < len(ori_point):
                index = i
            else:
                index = len(ori_point)-1
            out_points.append({
                "road_marker_id":ori_point[index]['road_marker_id'],
                "distToRightLine":ori_point[index]['distToRightLine'],
                "lane_id":lane['lane_id'],
                "point_id":i+1,
                "curve":curve,
                "heading":heading,
                "leftline_id":ori_point[index]['leftline_id'],
                "rightpoint_id":ori_point[index]['rightpoint_id'],
                "rightline_id":ori_point[index]['rightline_id'],
                "leftpoint_id":ori_point[index]['leftpoint_id'],
                "y":_y,
                "x":_x,
                "z":_z,
                "distToLeftLine":ori_point[index]['distToLeftLine'],
                "road_width":ori_point[index]['road_width'],
                "road_marker_type":ori_point[index]['road_marker_type']})
    else:
        out_points = ori_lane['points']

    out_lanes.append({
        "lane_id":ori_lane['lane_id'],
        "points":out_points})

with open('data/new_waypoints_test.json', 'w') as outfile:
    json.dump({"wayoints": out_lanes}, outfile, indent=4, separators=(',', ':'))


plt.show()
