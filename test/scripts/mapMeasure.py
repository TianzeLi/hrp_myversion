#! /usr/bin/python3

"""
 Load the map image and set the metric and origin, then obtain the 
 coordinates of the interested points with mouse clicks. 

 Input: 
    The map image. From Google map fpr example.
 Output: 
    The coordinates of the interested points.
 
 Pipeline: 
    1. Load the map image.
    2. Set the metric, i.e. how many meters is one pixel in length.
    3. Select the original point on the image. 
    4. Choose the interested points and computes the coordinates in meter. 
       By default, East-North-Up frame is applied. 

 Note:
    1. The opencv and python installation are not settled.
    2. Opencv is imported for potential processing.

 TODO: 
    1. Optionally settle the opencv and python installation. 
       (Might be unnecessary, as I will upgrade my ubuntu soon.)
    2. Add a image select and load GUI.
    3. Generate a metric at the lower-right corner.
    4. Maybe also connect with the path-follower to generate the input.

"""

COLORS = [
# 17 undertones https://lospec.com/palette-list/17undertones
'#000000', '#141923', '#414168', '#3a7fa7', '#35e3e3', '#8fd970', '#5ebb49',
'#458352', '#dcd37b', '#fffee5', '#ffd035', '#cc9245', '#a15c3e', '#a42f3b',
'#f45b7a', '#c24998', '#81588d', '#bcb0c2', '#ffffff',
]


import sys
from math import sqrt

from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtWidgets import (QWidget, QApplication, QLabel, QVBoxLayout, 
    QHBoxLayout, QMainWindow, QTableWidget, QTableWidgetItem)
from PyQt5.QtGui import QPixmap, QImage, QColor, QPainter
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt,QObject, QPoint

# in order to import cv2 under python3
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') 
import cv2
# append back in order to import rospy
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages') 



class QPaletteButton(QtWidgets.QPushButton):

    def __init__(self, color):
        super().__init__()
        self.setFixedSize(QtCore.QSize(24,24))
        self.color = color
        self.setStyleSheet("background-color: %s;" % color)


class App(QWidget):
    
    def __init__(self):
        super().__init__()

        # Application specific parameters
        self.pixel2meter = 1.0
        self.points_tmp = []
        self.points_of_interest = []
        self.original_x = 0.0
        self.original_y = 0.0
        self.point_width = 12

        self.pen = QtGui.QPen()
        self.pen.setWidth(self.point_width)


        self.setWindowTitle("Point coordinates select and locate")
        # The label that holds the image
        self.image_label = QLabel(self)
        # The text label holds the instruction below the image
        self.textLabel = QLabel('First set metric then click to select the interested points.')
        # The table to contain the coordinates
        self.table = QTableWidget(self)
        self.table.setColumnCount(6)    
        self.table.setHorizontalHeaderLabels(\
            ['No.','pixel u','pixel v','spatial x','spatial y','note'])
        self.table.setColumnWidth(0, 7);
        self.table.setFixedWidth(self.get_table_width());
        self.table.verticalHeader().setVisible(False)


        # Load the test image
        src_path = "../data/mapImg/lawnGmap.png"
        cv_img = cv2.imread(src_path)
        # convert the image to Qt format
        self.qt_img_raw, self.w_img, self.h_img = self.convert_cv_qt(cv_img)
        # display it
        self.image_label.setPixmap(self.qt_img_raw)


        # The buttons
        button_metric = QtWidgets.QPushButton(self)
        button_metric.clicked.connect(self.metric_update)
        button_metric.setText("Set metric")

        button_zero = QtWidgets.QPushButton(self)
        button_zero.clicked.connect(self.set_original)
        button_zero.setText("Set original point")

        button_PoI = QtWidgets.QPushButton(self)
        button_PoI.clicked.connect(self.PoI_select)
        button_PoI.setText("Set interested points")

        palette = QtWidgets.QHBoxLayout()
        self.add_palette_buttons(palette)

        # A vertical box layout that contains image and instrutions.
        vbox1 = QVBoxLayout()
        vbox1.addWidget(self.image_label)
        vbox1.addLayout(palette)
        vbox1.addWidget(self.textLabel)
        # A vertical box layout that contains buttons.
        vbox2 = QVBoxLayout()
        vbox2.addWidget(self.table)
        vbox2.addWidget(button_metric)
        vbox2.addWidget(button_zero)
        vbox2.addWidget(button_PoI)

        # A overall horizonal box
        hbox = QHBoxLayout()
        hbox.addLayout(vbox1)
        hbox.addLayout(vbox2)

        self.setLayout(hbox)
        # self.setGeometry(300, 300, self.w_img+600, self.h_img+40)
        self.show()

    def get_table_width(self):
        w = self.table.verticalHeader().width()+2
        for i in range(self.table.columnCount()):
            w += self.table.columnWidth(i)
        return w

    def table_resize(self):
        header = self.table.horizontalHeader()       
        header.setSectionResizeMode(5, QtWidgets.QHeaderView.Stretch)

    def add_palette_buttons(self, layout):
        for c in COLORS:
            b = QPaletteButton(c)
            b.pressed.connect(lambda c=c: self.pen.setColor(QColor(c)))
            layout.addWidget(b)
    
    def convert_cv_qt(self, cv_img):
        """Convert from an opencv image to QPixmap"""
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, \
            w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        return QPixmap.fromImage(convert_to_Qt_format), w, h

    def metric_update(self):
        """Set up or update the metric"""
        self.textLabel.setText("Set the metric")
        # spatial_length = input
        # dis_x = px2 - px1
        # dis_y = py2 - py1
        # pixel_length = sqrt(dis_x**2 + dis_y**2)
        # self.pixel2meter = spatial_length/pixel_length
        return 0

    def set_original(self):
        """Select the origin on the map image"""
        self.textLabel.setText("Click to set the original point")
        # self.original_x = px
        # self.original_y = py
        return 0

    def PoI_select(self):
        """Select the interested point"""
        self.textLabel.setText("Select interested points")
        return 0

    def mousePressEvent(self, e):
        super().mousePressEvent(e)
        self.points_tmp.append(e.pos())
        self.paintPoint()
        self.update()

    def paintPoint(self):
        painter = QPainter(self.image_label.pixmap())
        # pen.setColor(QtGui.QColor('red'))
        painter.setPen(self.pen)
        offset = self.point_width
        if self.points_tmp:
            point = self.points_tmp[-1]
            painter.drawPoint(point.x() - offset, point.y() - offset)
            print(point.x(), point.y())

            font = QtGui.QFont()
            font.setFamily('Times')
            font.setBold(True)
            font.setPointSize(8)
            painter.setFont(font)
            painter.drawText(point.x()-2, point.y()-2, str(len(self.points_tmp)))

            # TODO: compute the spatial coordinates here.
            self.addTableRow([len(self.points_tmp),point.x(), point.y(), 0, 0])
        painter.end()
        self.table_resize()

    def addTableRow(self, row_data):
        row = self.table.rowCount()
        self.table.setRowCount(row+1)
        col = 0
        for item in row_data:
            cell = QTableWidgetItem(str(item))
            self.table.setItem(row, col, cell)
            item = self.table.item(row, col)
            item.setTextAlignment(QtCore.Qt.AlignCenter)
            col += 1

    def distanceCompute(self, px1, py1, px2, py2):
        self.pixel2meter = 1.0
        dis_x = self.pixel2meter*(px2 - px1)
        dis_y = self.pixel2meter*(py2 - py1)
        dis = sqrt(dis_x**2 + dis_y**2)
        return dis, dis_x, dis_y


if __name__=="__main__":
    app = QApplication(sys.argv)
    a = App()
    sys.exit(app.exec_())