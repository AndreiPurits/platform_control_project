import sys, json, math, numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from PyQt5 import QtWidgets, QtCore
from PyQt5.QtGui import QPainter, QColor, QPen
from PyQt5.QtCore import Qt, QPointF, pyqtSignal

# ---------------- Models ----------------
class ScanModel(QtCore.QObject):
    changed = QtCore.pyqtSignal()
    def __init__(self):
        super().__init__()
        self.points = np.zeros((0,2), dtype=float)

# ---------------- ROS Node ----------------
class UINode(Node):
    def __init__(self, scan_model: ScanModel):
        super().__init__('platform_gui')
        self.scan_model = scan_model
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.pub_cmd  = self.create_publisher(String, '/mission/cmd', 10)
        self.pub_goal = self.create_publisher(String, '/route/goal', 10)

    def send_cmd(self, cmd: str):
        msg = String(); msg.data = cmd
        self.pub_cmd.publish(msg)

    def send_goal(self, x, y):
        msg = String(); msg.data = json.dumps({"x":x,"y":y})
        self.pub_goal.publish(msg)

    def on_scan(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0: return
        ang = np.linspace(msg.angle_min, msg.angle_max, n, endpoint=True)
        rng = np.array(msg.ranges, dtype=float)
        x = rng * np.cos(ang); y = rng * np.sin(ang)
        self.scan_model.points = np.stack([x,y], axis=1)
        self.scan_model.changed.emit()

# ---------------- Widgets ----------------
class RadarView(QtWidgets.QWidget):
    def __init__(self, model: ScanModel):
        super().__init__(); self.model = model
        self.model.changed.connect(self.update)
        self.setMinimumSize(250,250)
    def paintEvent(self, e):
        qp=QPainter(self); qp.fillRect(self.rect(), QColor(18,18,18))
        w,h=self.width(),self.height(); cx,cy=w/2,h/2; scale=min(w,h)/20
        qp.setPen(QPen(QColor(180,180,180),2))
        for (x,y) in self.model.points:
            qp.drawPoint(QPointF(cx+x*scale, cy-y*scale))

class CircleMap(QtWidgets.QWidget):
    goalPicked = pyqtSignal(float,float)
    def __init__(self):
        super().__init__(); self.setMinimumSize(400,400)
    def paintEvent(self,e):
        qp=QPainter(self); qp.fillRect(self.rect(),QColor(14,14,14))
        w,h=self.width(),self.height(); cx,cy=w/2,h/2; r=min(w,h)/2-20
        qp.setPen(QPen(QColor(100,200,255),3)); qp.drawEllipse(QPointF(cx,cy),r,r)
    def mousePressEvent(self,e):
        w,h=self.width(),self.height(); cx,cy=w/2,h/2; r=min(w,h)/2-20
        x=(e.x()-cx)/r*10; y=-(e.y()-cy)/r*10
        self.goalPicked.emit(x,y)

class CenterPanel(QtWidgets.QWidget):
    startClicked=pyqtSignal(); stopClicked=pyqtSignal()
    def __init__(self):
        super().__init__(); self.running=False
        self.btn=QtWidgets.QPushButton("Пуск"); self.btn.setMinimumSize(200,80)
        self.btn.clicked.connect(self.toggle)
        self.label=QtWidgets.QLabel("Скорость") # TODO: подписка на /speed
        self.label.setAlignment(Qt.AlignCenter); self.label.setStyleSheet("font-size:24px; color:white;")
        lay=QtWidgets.QVBoxLayout(self); lay.addWidget(self.btn,0,Qt.AlignCenter); lay.addWidget(self.label)
    def toggle(self):
        if not self.running: self.running=True; self.btn.setText("Стоп"); self.startClicked.emit()
        else: self.running=False; self.btn.setText("Пуск"); self.stopClicked.emit()
# ---------------- Main Window ----------------
class MainWin(QtWidgets.QWidget):
    def __init__(self,node:UINode,model:ScanModel):
        super().__init__(); self.node=node
        self.map=CircleMap(); self.center=CenterPanel(); self.radar=RadarView(model)
        self.center.startClicked.connect(lambda:node.send_cmd("START"))
        self.center.stopClicked.connect(lambda:node.send_cmd("STOP"))
        self.map.goalPicked.connect(lambda x,y:node.send_goal(x,y))
        lay=QtWidgets.QHBoxLayout(self); lay.addWidget(self.map,1); lay.addWidget(self.center,0); lay.addWidget(self.radar,0)

def qt_main():
    app=QtWidgets.QApplication(sys.argv); rclpy.init()
    model=ScanModel(); node=UINode(model); execu=SingleThreadedExecutor(); execu.add_node(node)
    win=MainWin(node,model); win.resize(1000,600); win.show()
    timer=QtCore.QTimer(); timer.timeout.connect(lambda:execu.spin_once(timeout_sec=0.0)); timer.start(10)
    ret=app.exec_(); execu.shutdown(); node.destroy_node(); rclpy.shutdown(); sys.exit(ret)

if __name__=="__main__": qt_main()
