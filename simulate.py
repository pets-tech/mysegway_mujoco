import time
import sys
import signal
import numpy as np
import mujoco
import mujoco.viewer

signal.signal(signal.SIGPIPE, signal.SIG_DFL)
np.set_printoptions(precision=5)

## ZMQ
import zmq
import struct
ctx = zmq.Context()
sock = ctx.socket(zmq.PUB)
sock.bind("tcp://127.0.0.1:5555")


MODEL = '/home/kika/path/segway/mysegway/segway_real.xml'
# K = np.array([ -1.24167, -36.26437,  -0.89145,  -7.14303])
K = np.array([ -1.41421, -44.92865,  -1.11508,  -9.05874])


m = mujoco.MjModel.from_xml_path(MODEL)
d = mujoco.MjData(m)


with mujoco.viewer.launch_passive(m, d) as viewer:
    
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
    viewer.cam.fixedcamid = m.camera("side_cam").id
    
    psi = 0.0
    prev_psi = 0
    prev_time = time.time()

    start = time.time()
    while viewer.is_running() and time.time() - start < 300:
        current_time = time.time()
        step_start = current_time

        t = current_time - start
        dt = current_time - prev_time; prev_time = current_time

        # system state
        theta_right = d.joint("right-wheel-joint").qpos[0]
        dtheta_right = d.joint("right-wheel-joint").qvel[0]
        dpsi = d.sensor("imu_gyro").data[0]
        psi += dpsi * dt

        x = np.array([theta_right, psi, dtheta_right, dpsi])

        # control
        x_des = np.array([0, 0, 0, 0])
        u = K @ (x_des - x)
        u = np.clip(u, -8.0, 8.0)

        if t > 0.0:
            d.ctrl[0] = u
            d.ctrl[1] = u
        
        mujoco.mj_step(m, d)
        
        ## ZMQ: send sate data using zmq protocol
        msg = struct.pack(
                "dfffff",
                t, theta_right, dtheta_right, psi, dpsi, u
        )
        sock.send(msg, zmq.NOBLOCK)

        with viewer.lock():
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)
        
        viewer.sync()
        
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
