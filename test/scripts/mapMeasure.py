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
    2. Add a image select and load.
    3. Generate a metric at the lower-right corner?
    4. Maybe also connect with the path-follower to generate the input.
    5. Store the metric and origin point from last use.

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
    QHBoxLayout, QMainWindow, QTableWidget, QTableWidgetItem, QInputDialog, 
    QAction, qApp, QMenuBar, QToolBar)
from PyQt5.QtGui import QPixmap, QImage, QColor, QPainter, QIcon
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

class MainWindow(QMainWindow):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)

        self.setWindowTitle("My Awesome App")

        # Menubar
        self.menubar = QMenuBar(self)
        fileMenu = self.menubar.addMenu("File")
        # fileMenu.addAction("New")
        # fileMenu.addAction("Open")
        # fileMenu.addAction("Save")
        exitAct = QAction(QIcon('exit.png'), '&Exit', self)
        exitAct.setShortcut('Ctrl+Q')
        exitAct.setStatusTip('Exit application')
        exitAct.triggered.connect(qApp.quit)
        fileMenu.addAction(exitAct)
        self.menubar.show()

        toolbar = QToolBar("My main toolbar")
        self.addToolBar(toolbar)

        certral_widget = MainWidget()
        self.setCentralWidget(certral_widget)


class MainWidget(QWidget):
    
    def __init__(self):
        super().__init__()
        # Application specific parameters
        self.pixel2meter = 1.0
        self.points_tmp = []
        self.points_of_interest = []
        self.original_x = 0.0
        self.original_y = 0.0
        self.circle_thickness = 4
        self.diameter = 12
        self.offset = 11

        # Painter parameters
        self.pen = QtGui.QPen()
        # self.pen.setColor(QColor('#ffd035'))
        self.pen.setWidth(self.circle_thickness)
        # Text font
        self.label_font = QtGui.QFont()
        self.label_font.setFamily('Times')
        self.label_font.setBold(True)
        self.label_font.setPointSize(8)

        # Load the test image
        src_path = "../data/mapImg/lawnGmap.png"
        cv_img = cv2.imread(src_path)
        # convert the image to Qt format
        self.qt_img_raw, self.w_img, self.h_img = self.convert_cv_qt(cv_img)

        # Window layout
        self.setWindowTitle("Point coordinates select and locate")
        # The label that holds the image
        self.image_label = QLabel(self)
        self.image_label.setCursor(Qt.CrossCursor)
        # The text label holds the instruction below the image
        self.textLabel = QLabel('First set metric then click to select the interested points.')
        self.statusLabel = QLabel('Metric and origin not set.')
        # The table to contain the coordinates
        self.table = QTableWidget(self)
        self.table.setColumnCount(6)    
        self.table.setHorizontalHeaderLabels(\
            ['No.','pixel u','pixel v','spatial x','spatial y','note'])
        self.table.setColumnWidth(0, 7);
        self.table.setFixedWidth(self.get_table_width());
        self.table.verticalHeader().setVisible(False)

        button_metric = QtWidgets.QPushButton(self)
        button_metric.clicked.connect(self.metric_update)
        button_metric.setText("Set metric")
        button_metric.resize(150, 50)

        button_zero = QtWidgets.QPushButton(self)
        button_zero.clicked.connect(self.set_original)
        button_zero.setText("Set original point")

        button_PoI = QtWidgets.QPushButton(self)
        button_PoI.clicked.connect(self.PoI_select)
        button_PoI.setText("Set interested points")

        button_points_clear = QtWidgets.QPushButton(self)
        button_points_clear.clicked.connect(self.points_clear)
        button_points_clear.setText("Clear drawn points.")

        palette = QtWidgets.QHBoxLayout()
        self.add_palette_buttons(palette)

        # A vertical box layout that contains image and instrutions.
        vbox1 = QVBoxLayout()
        # vbox1.addWidget(self.menubar)
        vbox1.addWidget(self.image_label)
        vbox1.addLayout(palette)
        vbox1.addWidget(self.statusLabel)
        # vbox1.addWidget(menubar)
        # A vertical box layout that contains buttons.
        vbox2 = QVBoxLayout()
        vbox2.addWidget(self.table)
        vbox2.addWidget(button_metric)
        vbox2.addWidget(button_zero)
        vbox2.addWidget(button_PoI)
        vbox2.addWidget(button_points_clear)
        vbox2.addWidget(self.textLabel)

        # A overall horizonal box
        hbox = QHBoxLayout()
        hbox.addLayout(vbox1)
        hbox.addLayout(vbox2)

        self.setLayout(hbox)
        self.image_label.setPixmap(self.qt_img_raw)
        self.show()

        # States: 0 - metric or original point not set
        self.status = 0
        self.count_tmp = 0
        


    def get_table_width(self):
        """Return the proper width to display the full table"""
        w = self.table.verticalHeader().width()+2
        for i in range(self.table.columnCount()):
            w += self.table.columnWidth(i)
        return w

    def table_resize(self):
        header = self.table.horizontalHeader()       
        header.setSectionResizeMode(5, QtWidgets.QHeaderView.Stretch)

    def add_palette_buttons(self, layout):
        """Generate palette buttons"""
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
        self.status = 1
        self.count_tmp = 0
        self.textLabel.setText("Setting the metric")

    def metric_compute(self, px1, py1, px2, py2):
        # print("In metric compute now")
        text, ok = QInputDialog.getText(self, 'Set length',
                                        'Enter the corresponding length:')
        spatial_length = float(text)
        dis_x = px2 - px1
        dis_y = py2 - py1
        pixel_length = sqrt(dis_x**2 + dis_y**2)
        self.pixel2meter = spatial_length/pixel_length
        self.statusLabel.setText("Each pixel is {0:.3f}m in length.".format(self.pixel2meter))
        self.points_tmp.pop(-1)
        self.points_tmp.pop(-1)
        self.status = 0

    def reset_map_img(self):
        painter = QPainter(self.image_label.pixmap())
        painter.drawPixmap(0, 0, self.w_img, self.h_img,self.qt_img_raw)
        painter.end()

    def set_original(self):
        """Select the origin on the map image"""
        self.status = 2
        self.textLabel.setText("Click to set the original point")
        self.update()

    def PoI_select(self):
        """Select the interested point"""
        self.points_tmp.clear()
        self.status = 3
        self.textLabel.setText("Click to add as the interested point")

    def points_clear(self):
        """Reset the map image and clear the labeled points"""
        self.reset_map_img()
        self.points_tmp.clear()
        self.textLabel.setText("Drawn points cleared.")
        self.update()

    def mousePressEvent(self, e):
        """Monitor the mouse press on the image"""
        super().mousePressEvent(e)
        self.points_tmp.append(e.pos())
        self.paintPoint()
        self.update()

    def paintPoint(self):
        """Paint labels on tha map"""
        painter = QPainter(self.image_label.pixmap())
        painter.setFont(self.label_font)        
        painter.setPen(self.pen)
        offset = self.offset + 0.5*self.diameter
        self.update()
        if self.status != 0:
            if self.points_tmp:
                point = self.points_tmp[-1]
                painter.drawEllipse(point.x() - offset, \
                    point.y() - offset, self.diameter, self.diameter)
                print(point.x(), point.y())

            if self.status == 1:
                self.count_tmp += 1
                if self.count_tmp == 2:
                    start = self.points_tmp[-2]
                    end = self.points_tmp[-1]
                    self.metric_compute(start.x(), start.y(), end.x(), end.y())
            
            if self.status == 2:
                self.original_x = point.x()
                self.original_y = point.y()
            
            if self.status == 3:
                dis, dis_x, dis_y = self.compute_coordinate(point.x(), point.y())
                self.addTableRow([len(self.points_tmp),point.x(), point.y(), format(dis_x, '.1f'), format(dis_y, '.1f')])
                painter.drawText(point.x(), point.y(), str(len(self.points_tmp)))
                self.table_resize()
        else:
            self.points_tmp.pop()
        painter.end()


    def addTableRow(self, row_data):
        """Append new row to the table"""
        row = self.table.rowCount()
        self.table.setRowCount(row+1)
        col = 0
        for item in row_data:
            cell = QTableWidgetItem(str(item))
            self.table.setItem(row, col, cell)
            item = self.table.item(row, col)
            item.setTextAlignment(QtCore.Qt.AlignCenter)
            col += 1

    def compute_coordinate(self, px, py):
        """Compute the x, y and direct distance"""
        dis_x = self.pixel2meter*(px - self.original_x)
        dis_y = self.pixel2meter*(-py + self.original_y)
        dis = sqrt(dis_x**2 + dis_y**2)
        return dis, dis_x, dis_y



if __name__=="__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())