import numpy as np
import math
import matplotlib.pyplot as plt
import os
from cloitholdpath import GetCurvatureOfPath
from scipy import interpolate

class ReplanWaypoint:
    def __init__(self, waypointsdict, human):
        self.waypointsdict = waypointsdict
        self.human = human
        self.original_lane_id = []
        self.cut_points_id = []
        self.new_lanes = []

    def set_OriginalLaneId(self, id):
        self.original_lane_id = id
    
    def set_CutPointsId(self, id):
        self.cut_points_id = id
        
    def headingGen(self, x, y):
        heading = []
        for i in range(len(x)-1):
            heading.append(math.atan2(y[i+1] - y[i], x[i+1] - x[i]))
        heading.append(heading[-1])
        return heading

    def find_closest_index(self, ind, _x, _y, points):
        for i in range(len(points['x'])):
            diff_dis = np.hypot(_y-points['y'][i], _x-points['x'][i])
            ind.append(np.argmin(diff_dis))

    def replan_waypoint(self):
        points_size_of_lane = []
        cut_points = dict(x=[], y=[]) 
        lanes = dict()
        for i in range(len(self.original_lane_id)):
            lanes[i] = dict(x=[], y=[], z=[])   
        connection_point = dict(x=[], y=[]) #original connection point
    
        human_x, human_y = [], []
        for i, (x, y, z) in enumerate(zip(self.human['x'], self.human['y'], self.human['z'])):
            human_x.append(x)
            human_y.append(y)
        human_x = np.array(human_x)
        human_y = np.array(human_y)
    
        '''separate lanes according to cut points'''
        ld = 0
        for ind in self.original_lane_id:
            lane = [lane for lane in self.waypointsdict if lane['lane_id'] == ind][0]
        
            points_size_of_lane.append(len(lane['points']))
            connection_point['x'].append(lane['points'][len(lane['points'])-1]['x'])
            connection_point['y'].append(lane['points'][len(lane['points'])-1]['y'])
        
            for i in lane['points']:
                if (lane['lane_id'] == self.cut_points_id[0]['lane_id'] and i['point_id'] == self.cut_points_id[0]['point_id']) or \
                    (lane['lane_id'] == self.cut_points_id[1]['lane_id'] and i['point_id'] == self.cut_points_id[1]['point_id']): 
                    cut_points['x'].append(i['x'])
                    cut_points['y'].append(i['y'])
                    ld += 1
                lanes[ld]['x'].append(i['x'])
                lanes[ld]['y'].append(i['y'])
                lanes[ld]['z'].append(i['z'])
    
        '''interpolate the new z'''
        old_x = []
        old_y = []
        old_z = []
        for i in range(len(lanes)):
            old_x += lanes[i]['x']
            old_y += lanes[i]['y']
            old_z += lanes[i]['z']
        f = interpolate.interp2d(np.array(old_x), np.array(old_y), np.array(old_z), kind='linear')
        human_z = []
        for i, (x, y) in enumerate(zip(human_x, human_y)):
            human_z.append(f(x, y)[0])
    

        '''find closest point in the human points'''
        closest_ind = []
        self.find_closest_index(closest_ind, human_x, human_y, cut_points)

        '''combine two condidtion'''
        total_lane_x, total_lane_y, total_lane_z = [], [], []
        total_lane_x += lanes[0]['x']
        total_lane_y += lanes[0]['y']
        total_lane_z += lanes[0]['z']
        for i in range(closest_ind[0], closest_ind[1]):
            total_lane_x.append(human_x[i])
            total_lane_y.append(human_y[i])
            total_lane_z.append(human_z[i])
        total_lane_x += lanes[2]['x']
        total_lane_y += lanes[2]['y']
        total_lane_z += lanes[2]['z']
        total_lane_x = np.array(total_lane_x)
        total_lane_y = np.array(total_lane_y)
        total_lane_z = np.array(total_lane_z)

        '''separate lanes according original lanes connection points'''
        closest_ind = [0]
        self.find_closest_index(closest_ind, total_lane_x, total_lane_y, connection_point)
    
        for i in range(len(self.original_lane_id)):
            new_lane = dict(x=[], y=[], z=[], lane_id = self.original_lane_id[i])
            for j in range(closest_ind[i], closest_ind[i+1]+1):
                new_lane['x'].append(total_lane_x[j])
                new_lane['y'].append(total_lane_y[j])
                new_lane['z'].append(total_lane_z[j])
            new_lane['curve'] = GetCurvatureOfPath(new_lane['x'], new_lane['y'])
            new_lane['heading'] = self.headingGen(new_lane['x'], new_lane['y'])
            self.new_lanes.append(new_lane)

        '''view new lanes'''
        plt.figure('vis_xy')
        plt.xlabel('X(m)')
        plt.ylabel('Y(m)')
        plt.grid()
        plt.plot(self.new_lanes[0]['x'], self.new_lanes[0]['y'], 'm.-', label='new lane 173')
        plt.plot(self.new_lanes[1]['x'], self.new_lanes[1]['y'], 'y.-', label='new lane 362')
        plt.plot(self.new_lanes[2]['x'], self.new_lanes[2]['y'], 'k.-', label='new lane 149')
    

        return self.original_lane_id, self.new_lanes


