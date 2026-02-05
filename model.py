import control as ct
import matplotlib.pyplot as plt
import numpy as np
np.set_printoptions(precision=5)

g = 9.81

######mujoco real segway
# wheel
m_wheel = 1.8
r_wheel = 0.17 / 2.0
r_wheel_stator = 0.111 / 2.0
J_wheel = 0.5 * m_wheel * (r_wheel_stator**2 + r_wheel**2)

# body
m_body = 22.0 - 2.0 * m_wheel   # 18.4
com_body = 0.426    # (10 - 2 * m_wheel) * x = 10 * (0.7 - x),
J_body = m_body * com_body**2 / 12.0


# # wheel
# m_wheel = 2*3.75*0.6
# r_wheel = 0.173 / 2
# J_wheel = 2*3.75*0.6*0.065**2

# # body
# m_body = 16.0
# com_body = 0.40
# J_body = 1.22

# m_wheel=2*3.75*0.6,      # kg
# m_body=16.0,      # kg  
# wheel_radius=0.173/2, # m
# body_com_length=0.40,#0.51,  # m
# I_wheel=2*3.75*0.6*0.065**2,  # kg·m²
# I_body_com=1.22,   # kg·m²
# g=9.81,
# b_viscous=0.    # N·m·s/rad



# ode E qdd + F qd + G q = H u
E = np.array([
    [m_body * com_body * r_wheel,   m_body * com_body**2 + J_body],
    [m_body * r_wheel**2 + 2.0 * m_wheel * r_wheel**2 + 2.0 * J_wheel, m_body * com_body * r_wheel]
]);

F = np.array([
    [0, 0],
    [0, 0]
])

G = np.array([
    [0, -m_body * com_body * g],
    [0, 0]
])

H = np.array([
    [-2],
    [2]
])


invE = np.linalg.inv(E)

EG = invE @ G
EF = invE @ F
EH = invE @ H
EG = -1.0 * EG
EF = -1.0 * EF

# x = [theta, psi, dtheta, dpsi]
A = np.array([   
    [0, 0, 1, 0],
    [0, 0, 0, 1],
    [0, EG[0,1], EF[0,0], EF[0,1]],
    [0, EG[1,1], EF[1,0], EF[1,1]],
])

B = np.array([
    [0 ],
    [0],
    [EH[0,0]],
    [EH[1,0]],
])

C = np.eye(4)
D = np.zeros((4,1))

print(A)
print(B)

c_matrix = np.hstack((B,A@B,A@A@B, A@A@A@B))
o_matrix = np.vstack((C,C@A,C@A@A, C@A@A@A))

# print(np.linalg.matrix_rank(c_matrix))
# print(np.linalg.matrix_rank(o_matrix))

#########################
##### modal control
time = 1.4
w0 = 7.8 / time
roots = [-w0] * 4
coeffs = np.poly(roots)

I = np.eye(4)
A2 = A @ A
A3 = A2 @ A
A4 = A3 @ A
akp = A4 + coeffs[1] * A3 + coeffs[2] * A2 + coeffs[3] * A + coeffs[4] * I

K1 = np.array([np.array([0, 0, 0, 1]) @ np.linalg.inv(c_matrix) @ akp])
print(np.array2string(K1.flatten(),separator=', '))

#########################
##### lqr control [theta, psi, dtheta, dpsi]
Q = np.diag([20,100,1,1])
R = 10
K2, S, E = ct.lqr(A, B, Q, R)
print(np.array2string(K2.flatten(), separator=', '))


# test: modal VS lqr
sys = ct.ss(A, B, C, D)

t = np.linspace(0, 5, 500)
x0 = np.array([0.2, 0, 0, 0])

sys_cl_modal = ct.ss(A - B @ K1, B, C, D)
t_out_m, y_out_m = ct.initial_response(sys_cl_modal, t, x0)

sys_cl_lqr = ct.ss(A - B @ K2, B, C, D)
t_out_lqr, y_out_lqr = ct.initial_response(sys_cl_lqr, t, x0)


fig, axes = plt.subplots(5, 1, figsize=(10, 12))
for ax in axes: ax.grid(True)

axes[0].plot(t_out_m, y_out_m[0], t_out_lqr, y_out_lqr[0])
axes[0].set_ylabel('psi'); 

axes[1].plot(t_out_m, y_out_m[2], t_out_lqr, y_out_lqr[2])
axes[1].set_ylabel('dpsi')

axes[2].plot(t_out_m, y_out_m[3], t_out_lqr, y_out_lqr[3])
axes[2].set_ylabel('theta')

axes[3].plot(t_out_m, y_out_m[1], t_out_lqr, y_out_lqr[1])
axes[3].set_ylabel('dtheta')

u_m = np.zeros(len(t_out_m))
u_lqr = np.zeros(len(t_out_m))
for i in range(len(t_out_m)):
    x = y_out_m[:, i].flatten() 
    u_m[i] = -K1.flatten() @ x

    x = y_out_lqr[:, i].flatten() 
    u_lqr[i] = -K2.flatten() @ x

axes[4].plot(t_out_m, u_m, t_out_lqr, u_lqr)
axes[4].set_ylabel('u')

plt.tight_layout()
plt.show()