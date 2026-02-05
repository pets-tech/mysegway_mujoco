import zmq
import struct
import pyqtgraph as pg
from PyQt6 import QtWidgets, QtCore
from collections import deque

WINDOW = 20000
UPDATE_MS = 20 
ADDR = "tcp://127.0.0.1:5555"
FMT = "dfffff"  # t theta dtheta psi dpsi u

ctx = zmq.Context()
sock = ctx.socket(zmq.SUB)
sock.connect(ADDR)
sock.setsockopt(zmq.SUBSCRIBE, b"")

app = QtWidgets.QApplication([])
win = pg.GraphicsLayoutWidget(title="rt_plotter")
win.resize(1200, 800)
win.show()

t_buf = deque(maxlen=WINDOW)
theta = deque(maxlen=WINDOW)
dtheta = deque(maxlen=WINDOW)
psi = deque(maxlen=WINDOW)
dpsi = deque(maxlen=WINDOW)
u = deque(maxlen=WINDOW)

p1 = win.addPlot(title="theta / dtheta")
p1.addLegend()
p1.showGrid(x=True, y=True)
c_theta  = p1.plot(pen=pg.mkPen('y', width=2), name="theta")
c_dtheta = p1.plot(pen=pg.mkPen('c', width=1), name="dtheta")

win.nextRow()

p2 = win.addPlot(title="psi / dpsi")
p2.addLegend()
p2.showGrid(x=True, y=True)
p2.setXLink(p1)
c_psi  = p2.plot(pen=pg.mkPen('m', width=2), name="psi")
c_dpsi = p2.plot(pen=pg.mkPen('g', width=1), name="dpsi")

win.nextRow()

p3 = win.addPlot(title="u")
p3.showGrid(x=True, y=True)
p3.setXLink(p1)
c_u = p3.plot(pen=pg.mkPen('r', width=2), name="u")

def update():
    updated = False

    while True:
        try:
            msg = sock.recv(zmq.NOBLOCK)
            ti, th, dth, ps, dps, ui = struct.unpack(FMT, msg)

            t_buf.append(ti)
            theta.append(th)
            dtheta.append(dth)
            psi.append(ps)
            dpsi.append(dps)
            u.append(ui)

            updated = True
        except zmq.Again:
            break

    if not updated:
        return

    c_theta.setData(t_buf, theta)
    c_dtheta.setData(t_buf, dtheta)
    c_psi.setData(t_buf, psi)
    c_dpsi.setData(t_buf, dpsi)
    c_u.setData(t_buf, u)

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(UPDATE_MS)

app.exec()
