import numpy as np
import json
from scipy.optimize import Bounds
from scipy.interpolate import splev, splrep, UnivariateSpline
from scipy.optimize import minimize
import matplotlib.pyplot as plt

def set_curv_xy(lane):
    xd = np.diff(lane['x'])
    xd = np.append(xd, xd[-1])
    yd = np.diff(lane['y'])
    yd = np.append(yd, yd[-1])
    xdd = np.diff(xd)
    xdd = np.append(xdd, xdd[-1])
    ydd = np.diff(yd)
    ydd = np.append(ydd, ydd[-1])
    curv =  ((np.multiply(xd, ydd) - np.multiply(yd , xdd)) / np.power(xd**2 + yd**2, 1.5))[0: -2]
    lane['curv'] = np.append(curv[0], np.append(curv, curv[-1]))

def set_curv_sp(lane, k=3):
    xd = splev(lane['splx'], (lane['tx'], lane['cx'], k), der=1)
    yd = splev(lane['splx'], (lane['ty'], lane['cy'], k), der=1)
    xdd = splev(lane['splx'], (lane['tx'], lane['cx'], k), der=2)
    ydd = splev(lane['splx'], (lane['ty'], lane['cy'], k), der=2)
    lane['curv'] = (np.multiply(xd, ydd) - np.multiply(yd , xdd)) / np.power(xd**2 + yd**2, 1.5)
    lane['h'] = np.arctan2(yd, xd)

def eval_wp(res, lane, c_opt, k=3):
    lane['cx'] = c_opt[lane['cx_range'][0]: lane['cx_range'][1]]
    lane['cy'] = c_opt[lane['cy_range'][0]: lane['cy_range'][1]]
    lane['splx'] =  np.linspace(0., lane['splx'][-1], 1 + int(np.around(lane['splx'][-1] / res)))
    lane['x'] = splev(lane['splx'], (lane['tx'], lane['cx'], k))
    lane['y'] = splev(lane['splx'], (lane['ty'], lane['cy'], k))
    lane['s'] = np.append(0., np.cumsum((np.diff(lane['x'])**2 + np.diff(lane['y'])**2)**0.5))
    set_curv_sp(lane)

def guess(x, y, k):
    return UnivariateSpline(x, y, k=k, s=1.)._eval_args
    #return splrep(x, y, k=k)

def guess2(x, y, k):
    return splrep(x, y, k=k)

def get_cost(lane, k=3):
    w_shape = 0.1
    w_smooth = 1. - w_shape
    err_x = lane['sply_x'] - splev(lane['splx'], (lane['tx'], lane['cx'], k))
    err_y = lane['sply_y'] - splev(lane['splx'], (lane['ty'], lane['cy'], k))
    xddd = splev(lane['splx2'], (lane['tx'], lane['cx'], k), der=3)
    yddd = splev(lane['splx2'], (lane['ty'], lane['cy'], k), der=3)
    cost = (w_shape *(np.abs(np.einsum('...i,...i', err_x, err_x)) +
            np.abs(np.einsum('...i,...i', err_y, err_y))) +
        w_smooth * (np.abs(np.einsum('...i,...i', xddd, xddd)) +
            np.abs(np.einsum('...i,...i', yddd, yddd))))
    return cost

def get_cost2(lane, k=3):
    w_shape = 0.1
    w_smooth = 1. - w_shape
    err_x = lane['sply_xy'] - splev(lane['splxx'], (lane['tx'], lane['cx'], k))
    err_y = lane['sply_yy'] - splev(lane['splxx'], (lane['ty'], lane['cy'], k))
    xddd = splev(lane['splx2'], (lane['tx'], lane['cx'], k), der=3)
    yddd = splev(lane['splx2'], (lane['ty'], lane['cy'], k), der=3)
    cost = (w_shape *(np.abs(np.einsum('...i,...i', err_x, err_x)) +
            np.abs(np.einsum('...i,...i', err_y, err_y))) +
        w_smooth * (np.abs(np.einsum('...i,...i', xddd, xddd)) +
            np.abs(np.einsum('...i,...i', yddd, yddd))))
    return cost


def cost_fun(c, lanes, target_id, disp=False):
    cost = 0.
    for i in target_id:
        lane = lanes[i]
        lane['cx'] = c[lane['cx_range'][0]: lane['cx_range'][1]]
        lane['cy'] = c[lane['cy_range'][0]: lane['cy_range'][1]]
        cost = cost + get_cost(lane)
    if disp:
        print(target_id ,cost)
    return cost

def cost_fun2(c, lanes, target_id, disp=False):
    cost = 0.
    for i in target_id:
        lane = lanes[i]
        lane['cx'] = c[lane['cx_range'][0]: lane['cx_range'][1]]
        lane['cy'] = c[lane['cy_range'][0]: lane['cy_range'][1]]
        cost = cost + get_cost2(lane)
    if disp:
        print(target_id ,cost)
    return cost


def curv_con(lane, k=3):
    xd = splev(lane['splx2'], (lane['tx'], lane['cx'], k), der=1)
    yd = splev(lane['splx2'], (lane['ty'], lane['cy'], k), der=1)
    xdd = splev(lane['splx2'], (lane['tx'], lane['cx'], k), der=2)
    ydd = splev(lane['splx2'], (lane['ty'], lane['cy'], k), der=2)
    return 0.04 - (np.multiply(xd, ydd) - np.multiply(yd , xdd))**2 / np.power(xd**2 + yd**2, 3)

def ineq_con_fun(c, lanes, target_id):
    con = np.array([])
    for i in target_id:
        lane = lanes[i]
        lane['cx'] = c[lane['cx_range'][0]: lane['cx_range'][1]]
        lane['cy'] = c[lane['cy_range'][0]: lane['cy_range'][1]]
        con = np.concatenate((con, curv_con(lane)))
    return con

def fix_point_con_fun(lane, k=3):
    err_x = lane['sply_x'][[0,-1]] - splev(lane['splx'][[0,-1]], (lane['tx'], lane['cx'], k))
    err_y = lane['sply_y'][[0,-1]] - splev(lane['splx'][[0,-1]], (lane['ty'], lane['cy'], k))
    return np.concatenate((err_x, err_y))

def eq_con_fun(c, lanes, target_id):
    con = np.array([])
    for i in target_id:
        lane = lanes[i]
        lane['cx'] = c[lane['cx_range'][0]: lane['cx_range'][1]]
        lane['cy'] = c[lane['cy_range'][0]: lane['cy_range'][1]]
        con = np.concatenate((con, fix_point_con_fun(lane)))
    return con

def waypoints_regulation_fix_start_end(res, lanes, target_id, k=3):
    c0 = np.array([])
    for i in target_id:
        lane = lanes[i]
        lane['tx'], lane['cx'], k = guess(lane['s'], lane['x'], k)
        lane['ty'], lane['cy'], k = guess(lane['s'], lane['y'], k)
        print(len(lane['tx']), len(lane['ty']))
        lane['splx'] = np.array(lane['s'])
        lane['splx2'] = np.linspace(0., lane['s'][-1], 1+int(np.around(lane['s'][-1] / res)))
        lane['sply_x'] = np.array(lane['x'])
        lane['sply_y'] = np.array(lane['y'])
        lane['cx_range'] = [len(c0), len(c0) + len(lane['cx'])] 
        c0 = np.concatenate((c0, lane['cx']))
        lane['cy_range'] = [len(c0), len(c0) + len(lane['cy'])] 
        c0 = np.concatenate((c0, lane['cy']))
    con_ineq = {'type': 'ineq', 'fun': ineq_con_fun, 'args': [lanes, target_id]} 
    con_eq = {'type': 'eq', 'fun': eq_con_fun, 'args': [lanes, target_id]} 
    bounds = Bounds(c0 - 0.5, c0 + 0.5)
    opt = minimize(
        cost_fun, c0, (lanes, target_id, False),
        method = 'SLSQP',
        constraints = [con_ineq, con_eq],
        bounds = bounds,
        options={'disp': False, 'maxiter': 100})
    c_opt = opt.x
    print(target_id, opt.fun)
    for i in target_id:
        lane = lanes[i]
        eval_wp(res, lane, c_opt)
        lane['optimal'] = True

def waypoints_regulation_fix_start_end2(res, lanes, target_id, k=3):
    c0 = np.array([])
    for i in target_id:
        lane = lanes[i]
        lane['tx'], lane['cx'], k = guess2(lane['s'], lane['x'], k)
        lane['ty'], lane['cy'], k = guess2(lane['s'], lane['y'], k)
        lane['splx'] = np.array(lane['s'])
        lane['splx2'] = np.linspace(0., lane['s'][-1], 1+int(np.around(lane['s'][-1] / res)))
        lane['sply_x'] = np.array(lane['x'])
        lane['sply_y'] = np.array(lane['y'])
        lane['cx_range'] = [len(c0), len(c0) + len(lane['cx'])] 
        c0 = np.concatenate((c0, lane['cx']))
        lane['cy_range'] = [len(c0), len(c0) + len(lane['cy'])] 
        c0 = np.concatenate((c0, lane['cy']))
    con_ineq = {'type': 'ineq', 'fun': ineq_con_fun, 'args': [lanes, target_id]} 
    con_eq = {'type': 'eq', 'fun': eq_con_fun, 'args': [lanes, target_id]} 
    bounds = Bounds(c0 - 0.5, c0 + 0.5)
    opt = minimize(
        cost_fun2, c0, (lanes, target_id, False),
        method = 'SLSQP',
        constraints = [con_ineq, con_eq],
        bounds = bounds,
        options={'disp': False, 'maxiter': 100})
    c_opt = opt.x
    print(target_id, opt.fun)
    for i in target_id:
        lane = lanes[i]
        eval_wp(res, lane, c_opt)
        lane['optimal'] = True

