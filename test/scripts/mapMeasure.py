#! /usr/bin/python3

"""
 Load the initial map and set the metric, then obtain the coordinates 
 of the interested points with mouse clicks. 

 Input: 
    The original map. 
 Output: 
    The coordinates of the interested points.
 
 Pipeline: 
    1. Load the map image.
    2. Set the metric, i.e. how many meters is one pixel distance representing.
    3. Choose the interested points and label them with the coordinates in meter. 
       By default, East-North-Up frame is applied. 


 Note:
    1. The opencv and python installation are not settled.

 TODO: 
    1. Optionally settle the opencv and python installation. 
       (Might be unnecessary, as I will upgrade my ubuntu soon.)
    2. Add a image select and load GUI.

"""


from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtWidgets import QWidget, QApplication, QLabel, QVBoxLayout, QHBoxLayout, QMainWindow
from PyQt5.QtGui import QPixmap, QImage, QColor
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt,QObject
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') # in order to import cv2 under python3
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages') # append back in order to import rospy


class App(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Point coordinates select and compute")
        self.disply_width = 640
        self.display_height = 480
        # create the label that holds the image
        self.image_label = QLabel(self)
        # create a text label
        self.textLabel = QLabel('First set metric then click to select the interested points.')


        # load the test image - we really should have checked that this worked!
        src_path = "../data/mapImg/lawnGmap.png"
        cv_img = cv2.imread(src_path)
        # convert the image to Qt format
        qt_img = self.convert_cv_qt(cv_img)
        # display it
        self.image_label.resize(self.disply_width, self.display_height)
        self.image_label.setPixmap(qt_img)


        button_metric = QtWidgets.QPushButton(self)
        button_metric.clicked.connect(self.metric_update)
        button_metric.setText("Set metric")
        button_metric.resize(160,50)
        # button_metric.move(40, 40)
         
        button_PoI = QtWidgets.QPushButton(self)
        button_PoI.clicked.connect(self.PoI_select)
        button_PoI.setText("Set interested points")
        button_PoI.resize(160,50)
        # button_PoI.move(40, 120)


        # create a vertical box layout that contains image and instrutions.
        vbox1 = QVBoxLayout()
        vbox1.addWidget(self.image_label)
        vbox1.addWidget(self.textLabel)

        # create a vertical box layout that contains buttons.
        vbox2 = QVBoxLayout()
        vbox2.addWidget(button_metric)
        vbox2.addWidget(button_PoI)

        hbox = QHBoxLayout()
        hbox.addLayout(vbox1)
        hbox.addLayout(vbox2)
        hbox.addStretch(1)

        self.setLayout(hbox)

    
    def convert_cv_qt(self, cv_img):
        """Convert from an opencv image to QPixmap"""
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        self.disply_width = w 
        self.display_height = h
        # p = convert_to_Qt_format.scaled(self.disply_width, self.display_height, Qt.KeepAspectRatio)
        return QPixmap.fromImage(convert_to_Qt_format)


    def metric_update(self):
        self.textLabel.setText("Set the metric")
        return 0


    def PoI_select(self):
        self.textLabel.setText("Select interested points")
        return 0
    

if __name__=="__main__":
    app = QApplication(sys.argv)
    a = App()
    a.show()
    sys.exit(app.exec_())