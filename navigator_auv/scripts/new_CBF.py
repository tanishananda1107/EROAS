#!/usr/bin/env python3
# Pure Python simulation — no ROS dependency.
# Only change: matplotlib CV_AVX flag removed (not valid in OpenCV Python).
import numpy as np
import casadi as ca
import matplotlib.pyplot as plt

dt = 0.01
kp, ki, kd = 2.0, 0.1, 0.5
v_const = 0.5
omega_max = np.pi
obs = [4.0, 4.0]
r_obs = 0.5
r_safe = 2.0
R = r_obs + r_safe
goal = [8.0, 8.0]
x, y, theta = 0.0, 0.0, 0.0
E, old_e = 0.0, 0.0
trajectory, cbf_trajectory, h_values = [], [], []
h_dot_plus_kh_values, control_inputs, dist_to_obstacle_values = [], [], []

def pid_controller(x, y, theta, goal, E, old_e):
    dx, dy = goal[0] - x, goal[1] - y
    g_theta = np.arctan2(dy, dx)
    alpha = g_theta - theta
    err = np.arctan2(np.sin(alpha), np.cos(alpha))
    ei = E + err * dt
    ed = (err - old_e) / dt
    omega = np.clip(kp*err + ki*ei + kd*ed, -omega_max, omega_max)
    return omega, ei, err

def cbf_filter(x, y, theta, omega_pid, obs, R, return_debug=False):
    opti = ca.Opti()
    omega = opti.variable()
    xc = opti.parameter(); yc = opti.parameter(); tc = opti.parameter()
    opti.set_value(xc, x); opti.set_value(yc, y); opti.set_value(tc, theta)
    dist = ca.sqrt((xc - obs[0])**2 + (yc - obs[1])**2)
    h = dist - R
    dh_dx = ca.gradient(dist, xc)
    dh_dy = ca.gradient(dist, yc)
    h_dot = dh_dx*v_const*ca.cos(tc) + dh_dy*v_const*ca.sin(tc) + omega
    opti.minimize(0.5*(omega - omega_pid)**2)
    opti.subject_to(h_dot + 5.0*h >= 0)
    opti.subject_to(opti.bounded(-omega_max, omega, omega_max))
    opti.solver('ipopt', {"verbose": False, "ipopt.print_level": 0, "print_time": 0})
    try:
        sol = opti.solve()
        ov = sol.value(omega)
        return (ov, sol.value(h), sol.value(h_dot) + 5.0*sol.value(h)) if return_debug else ov
    except RuntimeError:
        return (omega_pid, 0, 0) if return_debug else omega_pid

for step in range(10000):
    if np.hypot(goal[0]-x, goal[1]-y) < 0.1:
        print("Goal reached!"); break
    omega_pid, E, old_e = pid_controller(x, y, theta, goal, E, old_e)
    dist_to_obs = np.hypot(obs[0]-x, obs[1]-y)
    if dist_to_obs <= R + 1.0:
        omega_cbf, h, h_dot_kh = cbf_filter(x, y, theta, omega_pid, obs, R, return_debug=True)
        cbf_trajectory.append([x, y]); h_values.append(h); h_dot_plus_kh_values.append(h_dot_kh)
    else:
        omega_cbf = omega_pid
    x += v_const*np.cos(theta)*dt; y += v_const*np.sin(theta)*dt
    theta = np.arctan2(np.sin(theta + omega_cbf*dt), np.cos(theta + omega_cbf*dt))
    trajectory.append([x, y]); control_inputs.append(omega_cbf); dist_to_obstacle_values.append(dist_to_obs)

trajectory = np.array(trajectory)
cbf_trajectory = np.array(cbf_trajectory) if len(cbf_trajectory) > 0 else None

plt.figure(figsize=(12, 12))
plt.subplot(2,3,1)
plt.plot(trajectory[:,0], trajectory[:,1], label='Trajectory', color='blue')
plt.scatter(goal[0], goal[1], c='red', marker='*', s=100, label='Goal')
plt.scatter(obs[0], obs[1], c='orange', s=100, label='Obstacle')
plt.gca().add_patch(plt.Circle((obs[0], obs[1]), r_obs, color='orange', alpha=0.5))
plt.gca().add_patch(plt.Circle((obs[0], obs[1]), R, color='red', alpha=0.3, linestyle='--'))
plt.xlabel('X'); plt.ylabel('Y'); plt.title('CBF Simulation'); plt.legend(); plt.grid(True)
plt.subplot(2,3,2); plt.plot(h_values, label='h'); plt.title('Barrier h'); plt.grid(True); plt.legend()
plt.subplot(2,3,3); plt.plot(h_dot_plus_kh_values, label='h_dot+k*h'); plt.title('h derivative'); plt.grid(True); plt.legend()
plt.subplot(2,3,4); plt.plot(control_inputs, label='Omega'); plt.title('Control Input'); plt.grid(True); plt.legend()
plt.subplot(2,3,5); plt.plot(dist_to_obstacle_values, label='Dist'); plt.title('Distance to Obstacle'); plt.grid(True); plt.legend()
plt.tight_layout(); plt.show()
