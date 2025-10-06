import numpy as np
from scipy.optimize import minimize

# class MinimumSnap():
#     def __init__(self):

def compute_min_snap_coefficients_multi(waypoints, times):

    waypoints = np.asarray(waypoints)
    times = np.asarray(times)
    n_segments = len(waypoints) - 1
    if len(times) != len(waypoints):
        raise ValueError("Times and waypoints must have same length")
    if not np.all(np.diff(times) > 0):
        raise ValueError("Times must be increasing")
    
    n_vars = 8 * n_segments
    
    # Xây dựng H cho cost 1/2 x^T H x
    H = np.zeros((n_vars, n_vars))
    for i in range(n_segments):
        ti, ti1 = times[i], times[i+1]
        base = 8 * i
        for j in range(4):
            deg_j = 7 - j
            prod_j = np.prod([deg_j - l for l in range(4)])
            for k in range(4):
                deg_k = 7 - k
                prod_k = np.prod([deg_k - l for l in range(4)])
                ex = (deg_j - 4) + (deg_k - 4)
                if ex + 1 != 0:
                    integral = (ti1**(ex + 1) - ti**(ex + 1)) / (ex + 1)
                else:
                    integral = np.log(ti1 / ti) if ti > 0 else 0
                H[base + j, base + k] = prod_j * prod_k * integral
    
    # Hàm lấy row cho đạo hàm deriv tại t cho seg
    def get_deriv_row(seg, t, deriv):
        row = np.zeros(n_vars)
        base = 8 * seg
        for j in range(8):
            deg = 7 - j
            if deg < deriv: continue
            prod = np.prod([deg - l for l in range(deriv)])
            row[base + j] = prod * (t ** (deg - deriv))
        return row
    
    # Xây dựng A và b cho constraints A x = b
    A = np.zeros((0, n_vars))
    b = np.zeros(0)
    
    # Position constraints
    for i in range(n_segments):
        ti, ti1 = times[i], times[i+1]
        A = np.vstack((A, get_deriv_row(i, ti, 0)))
        b = np.append(b, waypoints[i])
        A = np.vstack((A, get_deriv_row(i, ti1, 0)))
        b = np.append(b, waypoints[i+1])
    
    # Continuity v/a/j
    for i in range(1, n_segments):
        ti = times[i]
        for deriv in range(1, 4):
            row = get_deriv_row(i-1, ti, deriv) - get_deriv_row(i, ti, deriv)
            A = np.vstack((A, row))
            b = np.append(b, 0)
    
    # Boundary at start and end
    for deriv in range(1, 4):
        A = np.vstack((A, get_deriv_row(0, times[0], deriv)))
        b = np.append(b, 0)
    for deriv in range(1, 4):
        A = np.vstack((A, get_deriv_row(n_segments-1, times[-1], deriv)))
        b = np.append(b, 0)
    
    eigenvalues = np.linalg.eigvals(H)

    # Objective and constraint functions
    def objective(x):
        return 0.5 * np.dot(x.T, np.dot(H, x))
    
    def constraint(x):
        return np.dot(A, x) - b
    
    # Optimize
    x0 = np.zeros(n_vars)
    cons = {'type': 'eq', 'fun': constraint}
    res = minimize(objective, x0, method='SLSQP', constraints=[cons], options={'disp': False, 'maxiter': 1000})
    if not res.success:
        raise ValueError(f"Optimization failed: {res.message}")
    
    # Extract coeffs
    return [res.x[8*i:8*(i+1)] for i in range(n_segments)]
    """
    Tính hệ số đa thức bậc 7 cho minimum snap trajectory qua nhiều waypoints (smooth, continuous v/a/j).
    - waypoints: list/array vị trí [p0, p1, ..., pn]
    - times: list/array thời gian [t0=0, t1, ..., tn=T]
    Trả về list coeffs, mỗi coeffs là array [c0 (t^7), c1 (t^6), ..., c7 (const)] cho từng segment.
    """
 

def evaluate_position_multi(coeffs_list, times, t):
    """
    Đánh giá vị trí x(t) tại thời gian t (t in [times[0], times[-1]]).
    """
    n_segments = len(coeffs_list)
    for i in range(n_segments):
        if times[i] <= t <= times[i+1]:
            return np.polyval(coeffs_list[i], t)
    raise ValueError("t out of range")

def interpolate_points_multi(coeffs_list, times, num_points=50):
    """
    Nội suy num_points điểm trên toàn quỹ đạo, trả về list tuple (t, x(t)).
    """
    t_values = np.linspace(times[0], times[-1], num_points)
    points = []
    for t in t_values:
        x = evaluate_position_multi(coeffs_list, times, t)
        points.append((t, x))
    return points

def compute_euclidean_distances(waypoints):
    """
    Tính quãng đường Euclidean giữa các waypoints liên tiếp.
    - waypoints: list hoặc array của các điểm, mỗi điểm là số (1D) hoặc tuple/list (2D/3D).
    Trả về: list các khoảng cách s_i và tổng quãng đường s_total.
    """
    waypoints = np.asarray(waypoints)
    if waypoints.ndim == 1:  # 1D
        s_segments = np.abs(np.diff(waypoints))
    else:  # 2D/3D
        s_segments = np.sqrt(np.sum(np.diff(waypoints, axis=0)**2, axis=1))
    s_total = np.sum(s_segments)
    print(s_total)
    return s_segments.tolist(), s_total

def cal_time_for_fly(s, v_max):
    T = s/(v_max*0.619) # , nhằm hiểu rằng vận tốc không được vượt quá vận tốc tối đa
    return T

def check_constrains_v (coeffs_list_x, times, v_max = 5.0, a_max = 2.0, num_points = 100):
    t_values = np.linspace(times[0], times[-1], num_points)
    print("t_values: ",len(t_values))
    for i in range(len(coeffs_list_x)):
        t_seg = t_values[(t_values >= times[i]) & (t_values <= times[i+1])]
        coeffs_x = coeffs_list_x[i][::-1]
        # print("t_values: ",len(t_values))
        # coeffs_y = coeffs_list_y[i][::-1]
        # coeffs_z = coeffs_list_z[i][::-1]
        v_coeffs_x = np.polyder(coeffs_x)
        a_coeffs_x = np.polyder(v_coeffs_x)
        for t in t_seg:
            v_total = np.sqrt(np.polyval(v_coeffs_x, t)**2 )
            a_total = np.sqrt(np.polyval(a_coeffs_x, t)**2 )
            if v_max is not None and v_total > v_max:
                print(f"Constraint violated: v_total={v_total:.2f} > v_max={v_max} at t={t:.2f}")
                return False
            if a_max is not None and a_total > a_max:
                print(f"Constraint violated: a_total={a_total:.2f} > a_max={a_max} at t={t:.2f}")
                return False
    return True


def main ():
    waypoints = [[0, 0,0], [1,2,5], [3,4,5]]  # Ví dụ 4 điểm
    # waypoints_y = [0,2,4,1]
    # waypoints_z = [0,5,5,5]
    # T = 3  # Tổng thời gian
    wp_x = [i[0] for i in waypoints]
    wp_y = [i[1] for i in waypoints]
    wp_z = [i[2] for i in waypoints]
    print(wp_x)

    s_segments, total_s = compute_euclidean_distances(waypoints)
    total_time = cal_time_for_fly(total_s, 10)
    
    # times = np.linspace(0, total_time, len(waypoints))  # Phân bổ thời gian đều
    times = [0]
    for s_i in s_segments:
        times.append(times[-1] + total_time *s_i / total_s)

    coeffs_list_x = compute_min_snap_coefficients_multi(wp_x, times)
    coeffs_list_y = compute_min_snap_coefficients_multi(wp_y, times)
    coeffs_list_z = compute_min_snap_coefficients_multi(wp_z, times)
    # print("Coeffs for each segment:", coeffs_list_x)
    points_x = interpolate_points_multi(coeffs_list_x, times, num_points=20)
    points_y = interpolate_points_multi(coeffs_list_y, times, num_points=20)
    points_z = interpolate_points_multi(coeffs_list_z, times, num_points=20)
    # print("times", times)
    check_constrains_v(coeffs_list_x, times)
    
    # for t, x in points_x:
    #     print(f"t={t:.2f}, x(t)={x:.3f}")
  
# Ví dụ sử dụng
if __name__ == "__main__":
    main()
    # Danh sách điểm (waypoints)
    



# eiufablab-drone-framework