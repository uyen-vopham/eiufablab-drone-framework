import numpy as np
from scipy.optimize import minimize

def compute_min_snap_coeff_(waypoints, times):
    #tính hệ số đa thức bậc 7 qua waypoints.
    # - waypoints: list/array vị trí [p0,p1,...,pn]
    # - times: list/array thời gian [t0 = 0, t1,...,tn=T]
    # trả về list coeffs, mỗi coeffs là array chứa 8 hệ số a0 -> a7

    waypoints = np.asarray(waypoints)
    times = np.asarray(times)
    n_segments = len(waypoints) - 1
    if len(times) != len(waypoints): #từng khoản thời gian tương ứng với từng khoản segment, 
                                    # do đó số lần lấy điểm tính phải bằng số lượng waypoints
        raise ValueError("Time and waypoints must have same length")
    if not np.all(np.diff(times) > 0):
        raise ValueError ("Time must be increasing")

    n_vars = 8 * n_segments #mỗi segment gồm 8 hệ số
                            #với n segment, thì số lượng hệ số cần tính là 8*n_segment
    
    H = np.zeros((n_vars, n_vars))
    for i in range(n_segments):
        ti, ti1 = times[i], times[i+1]
        base = 8*i
        for j in range(4):
            deg_j = 7 - j
            prod_j = np.prod([deg_j - l for l in range(4)])