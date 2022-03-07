import numpy as np
import matplotlib.pyplot as plt

def visualize_xy(lanes, target_id, new_plot=False, show_text=False):
    for i in target_id:
        if new_plot:
            plt.plot(lanes[i]['x'], lanes[i]['y'], '.-')
            if not lanes[i]['optimal']:
                plt.plot(lanes[i]['x'], lanes[i]['y'], 'k.-')
        else:
            plt.plot(lanes[i]['x'], lanes[i]['y'], '.-', alpha=0.5)
        if show_text:
            plt.text(lanes[i]['x'][len(lanes[i]['x'])//2], lanes[i]['y'][len(lanes[i]['y'])//2], i)

def visualize_s(lanes, target_id, new_plot=False):
    range_offset = 0.
    for i in target_id:
        if new_plot:
            plt.plot(lanes[i]['s'][1:] + range_offset, np.diff(lanes[i]['s']), '.-')
            if not lanes[i]['optimal']:
                plt.plot(lanes[i]['s'][1:] + range_offset, np.diff(lanes[i]['s']), 'k.-')
        else:
            plt.plot(lanes[i]['s'][1:] + range_offset, np.diff(lanes[i]['s']), '.-', alpha=0.3)
        range_offset = range_offset + np.around(lanes[i]['s'][-1] - 1e-3) + 1.

def visualize_curv(lanes, target_id, new_plot=False):
    range_offset = 0.
    for i in target_id:
        if new_plot:
            plt.plot(lanes[i]['s'] + range_offset, lanes[i]['curv'], '.-')
            if not lanes[i]['optimal']:
                plt.plot(lanes[i]['s'] + range_offset, lanes[i]['curv'], 'k.-')
        else:
            plt.plot(lanes[i]['s'] + range_offset, lanes[i]['curv'], '.-', alpha=0.3)
        range_offset = range_offset + np.around(lanes[i]['s'][-1] - 1e-3) + 2.
