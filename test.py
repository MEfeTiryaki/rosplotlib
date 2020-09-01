
from __future__ import unicode_literals
import sys
import os
import random
import matplotlib
# Make sure that we are using QT5
matplotlib.use('Qt5Agg')
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import QApplication, QWidget, QListWidget, QVBoxLayout, QListWidgetItem,QPushButton,QTableWidget,QTableWidgetItem,QHBoxLayout

from numpy import arange, sin, pi
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt

progname = os.path.basename(sys.argv[0])
progversion = "0.1"
import rospy
from std_msgs.msg import Float64MultiArray

import threading, time, random

# Python create mutex


class MyMplCanvas(FigureCanvas):
    """Ultimately, this is a QWidget (as well as a FigureCanvasAgg, etc.)."""

    def __init__(self, parent=None):
        pass

    def compute_initial_figure(self):
        pass



class MyDynamicMplCanvas(MyMplCanvas):
    """A canvas that updates itself every second with a new plot."""

    def __init__(self, parent, width=5, height=4, dpi=100):
        MyMplCanvas.__init__(self,parent)
        self.mutex = threading.Lock()
        self.fig = plt.figure(figsize=(width, height), dpi=dpi)
        FigureCanvas.__init__(self, self.fig)
        self.setParent(parent)

        FigureCanvas.setSizePolicy(self,
                                   QtWidgets.QSizePolicy.Expanding,
                                   QtWidgets.QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(self)

        self.isPlotting = True
        self.data = [[1,2,3,4],[5,10,10,1]]
        self.subplot_indeces = [1]
        self.x_indeces = [0]
        self.y_indeces = [1]
        self.color=['r']
        self.N = [4]
        self.M = 1
        self.update_subplots()
        self.compute_initial_figure()
        self.draw()

        timer = QtCore.QTimer(self)
        timer.timeout.connect(self.update_figure)
        timer.start(10)
        # self.sub = rospy.Subscriber("x", Float64MultiArray, self.callback)
#
    def update_subplots(self):
        self.axes = []
        for i in range(self.M):
            self.axes.append(plt.subplot(self.M,1,i+1))

    def compute_initial_figure(self):
        # self.mutex.acquire()
        # for i in range(0,self.M):
        #     self.axes[self.axes_numbers[i]].plot(self.x, self.y[i], self.label[i])
        # self.draw()
        # self.mutex.release()
        pass

    def update_figure(self):
        if self.isPlotting:
            self.mutex.acquire()
            for i in range(self.M):
                    self.axes[self.subplot_indeces[i]-1].cla()
            for i in range(self.M):
                # print(self.data)
                if len(self.data[self.x_indeces[i]]) ==len(self.data[self.y_indeces[i]]):
                    self.axes[self.subplot_indeces[i]-1].plot(self.data[self.x_indeces[i]], self.data[self.y_indeces[i]], self.color[i])

            self.draw()
            self.mutex.release()


    def callback(self,data,i):
        if self.isPlotting:
            self.mutex.acquire()
            self.data[i] = data.data
            self.mutex.release()

    def setIsPlotting(self,flag):
        self.isPlotting = flag
        if not self.isPlotting:
            for sub in self.subs:
                sub.unregister()

    def setPlots(self,indeces,x_subs,y_subs,color):
        self.mutex.acquire()
        self.M = len(x_subs)
        self.subs=[]
        self.x_indeces = []
        self.y_indeces = []
        self.data = []
        self.color = color
        subscribers = list(set(x_subs+y_subs))
        for idx, val in enumerate(subscribers):
            print(idx,val)
            self.subs.append(rospy.Subscriber(val, Float64MultiArray,  self.callback,idx))
            self.data.append([])

        for s in x_subs:
            idx = subscribers.index(s)
            self.x_indeces.append(idx)

        for s in y_subs:
            idx = subscribers.index(s)
            self.y_indeces.append(idx)

        self.subplot_indeces = indeces
        self.subplotNumber = max(set(indeces))

        self.axes= []
        for i in range(0,self.subplotNumber):
             self.axes.append(plt.subplot(self.subplotNumber,1,i+1))
        self.draw()
        self.mutex.release()

class PlotListWidget(QListWidget):
   def Clicked(self,item):
      QMessageBox.information(self, "ListWidget", "You clicked: "+item.text())

class ApplicationWindow(QtWidgets.QMainWindow):
    def __init__(self):
        QtWidgets.QMainWindow.__init__(self)
        self.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        self.setWindowTitle("application main window")

        self.file_menu = QtWidgets.QMenu('&File', self)
        self.file_menu.addAction('&Quit', self.fileQuit,
                                 QtCore.Qt.CTRL + QtCore.Qt.Key_Q)
        self.menuBar().addMenu(self.file_menu)

        self.help_menu = QtWidgets.QMenu('&Help', self)
        self.menuBar().addSeparator()
        self.menuBar().addMenu(self.help_menu)

        self.help_menu.addAction('&About', self.about)

        self.main_widget = QtWidgets.QWidget(self)

        l = QtWidgets.QVBoxLayout(self.main_widget)
        self.dc = MyDynamicMplCanvas(self.main_widget, width=5, height=4, dpi=100)
        l.addWidget(self.dc)

        self.main_widget.setFocus()
        self.setCentralWidget(self.main_widget)

        self.statusBar().showMessage("All hail matplotlib!", 2000)

        # LIST
        self.table = QTableWidget(1, 4)
        self.table.setFixedHeight(100)
        self.tableContent = {'subplot':['1'],
        'X subscriber':['x'],
        'Y subscriber':['y'],
        'color':['r']}
        horHeaders = []
        for n, key in enumerate(['subplot','X subscriber',   'Y subscriber',   'color']):
            horHeaders.append(key)
            for m, item in enumerate(self.tableContent[key]):
                newitem = QTableWidgetItem(item)
                self.table.setItem(m, n, newitem)
        self.table.setHorizontalHeaderLabels(horHeaders)

        # Buttons
        self.plot_button = QPushButton("start")
        self.isPlotting = False
        self.add_button = QPushButton("add")
        self.remove_button = QPushButton("remove")

        self.plot_button.clicked.connect(lambda:self.plotButtonCallback())
        self.add_button.clicked.connect(lambda:self.addButtonCallback())
        self.remove_button.clicked.connect(lambda:self.removeButtonCallback())

        l.addWidget(self.table)
        hbox = QHBoxLayout()
        hbox.addStretch(1)
        l.addLayout(hbox)
        hbox.addWidget(self.plot_button)
        hbox.addWidget(self.add_button)
        hbox.addWidget(self.remove_button)

    def plotButtonCallback(self):
        self.isPlotting = not self.isPlotting
        self.dc.setIsPlotting(self.isPlotting)
        if self.isPlotting:
            self.statusBar().showMessage("Started", 2000)
            suplotIndeces = []
            x = []
            y = []
            color = []
            for i in range(self.table.rowCount()):
                suplotIndeces.append(int(self.table.item(i,0).data(0)))
                x.append(str(self.table.item(i,1).text()))
                y.append(str(self.table.item(i,2).text()))
                color.append(str(self.table.item(i,3).text()))
            self.dc.setPlots(suplotIndeces,x,y,color)
            self.plot_button.setText("stop")
        else:
            self.statusBar().showMessage("Stopped", 2000)
            self.plot_button.setText("start")

    def addButtonCallback(self):
        n = self.table.rowCount()
        self.table.setRowCount(n+1)
    def removeButtonCallback(self):
        n = self.table.currentRow()
        self.table.removeRow(n)


    def fileQuit(self):
        self.close()

    def closeEvent(self, ce):
        self.fileQuit()

    def about(self):
        QtWidgets.QMessageBox.about(self, "About",
                                    """ This program is a simple ROS plotter based on matplotlib. It is not a time series plotter"""
                                )


qApp = QtWidgets.QApplication(sys.argv)
rospy.init_node('listener', anonymous=True)

aw = ApplicationWindow()
aw.setWindowTitle("%s" % progname)
aw.show()
sys.exit(qApp.exec_())
#qApp.exec_()
