﻿from PyQt5 import uic
from PyQt5.QtCore import QFile, QRegExp, QTimer, Qt, pyqtSlot
from PyQt5.QtWidgets import QApplication, QFileDialog, QMainWindow, QMenu, QMessageBox
from fieldManager import FieldManager
from vision import Vision
from s826 import S826
from subThread import SubThread
from realTimePlot import CustomFigCanvas
import syntax
from math import sqrt,pow
import time

#=========================================================
# UI Config
#=========================================================
qtCreatorFile = "mainwindow.ui"
Ui_MainWindow, QtBaseClass = uic.loadUiType(qtCreatorFile)
#=========================================================
# Creating instances of fieldManager and Camera
#=========================================================
field = FieldManager(S826())
vision = Vision(field,index=1,type='firewire',guid=2672909588927744,buffersize=12) # greyscale mode
#vision = Vision(field,index=1,type='firewire',guid=2672909587850323,buffersize=12) # greyscale mode
#vision = Vision(field,index=2,type='firewire',guid=2672909587849792,buffersize=12)

# to use usb camera, try
#vision = Vision(index=1,type='usb')
#vision = Vision(field,index=1,type='video',guid=0000000000,buffersize=12)
# to use 1 camera only, comment out this line:    vision2 = ...
#=========================================================
# Creating instances of PS3 controller
#=========================================================
# from PS3Controller import DualShock
# joystick = DualShock()
# to disable controller comment out these 2 lines
#=========================================================
# a class that handles the signal and callbacks of the GUI
#=========================================================
class GUI(QMainWindow,Ui_MainWindow):
    def __init__(self):
        QMainWindow.__init__(self,None,Qt.WindowStaysOnTopHint)
        Ui_MainWindow.__init__(self)
        self.updateRate = 15 # (ms) update rate of the GUI, vision, plot
        self.setupUi(self)
        self.setupTimer()
        try:
            joystick
        except NameError:
            self.setupSubThread(field,vision)
        else:
            self.setupSubThread(field,vision,joystick)
        self.setupRealTimePlot() # comment ou this line if you don't want a preview window
        self.connectSignals()
        self.linkWidgets()

    #=====================================================
    # [override] terminate the subThread and clear currents when closing the window
    #=====================================================
    def closeEvent(self,event):
        self.thrd.stop()
        self.timer.stop()
        vision.closeCamera()
        try:
            vision2
        except NameError:
            pass
        else:
            vision2.closeCamera()
        try:
            joystick
        except NameError:
            pass
        else:
            joystick.quit()
        self.clearField()
        event.accept()

    #=====================================================
    # QTimer handles updates of the GUI, run at 60Hz
    #=====================================================
    def setupTimer(self):
        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(self.updateRate) # msec

    def update(self):
        vision.updateFrame()
        try:
            vision2
        except NameError:
            pass
        else:
            vision2.updateFrame()
        try:
            self.realTimePlot
        except AttributeError:
            pass
        else:
            self.updatePlot()
        try:
            joystick
        except NameError:
            pass
        else:
            joystick.update()



    #=====================================================
    # Connect buttons etc. of the GUI to callback functions
    #=====================================================
    def connectSignals(self):
        # General Control Tab
        self.dsb_x.valueChanged.connect(self.setFieldXYZ)
        self.dsb_y.valueChanged.connect(self.setFieldXYZ)
        self.dsb_z.valueChanged.connect(self.setFieldXYZ)
        self.btn_clearCurrent.clicked.connect(self.clearField)
        self.dsb_xGradient.valueChanged.connect(self.setFieldXYZGradient)
        self.dsb_yGradient.valueChanged.connect(self.setFieldXYZGradient)
        self.dsb_zGradient.valueChanged.connect(self.setFieldXYZGradient)
        # Vision Tab
        self.highlighter = syntax.Highlighter(self.editor_vision.document())
        self.chb_bypassFilters.toggled.connect(self.on_chb_bypassFilters)
        self.btn_refreshFilterRouting.clicked.connect(self.on_btn_refreshFilterRouting)
        self.btn_snapshot.clicked.connect(self.on_btn_snapshot)
        # object detection
        self.chb_objectDetection.toggled.connect(self.on_chb_objectDetection)
        # Subthread Tab
        self.cbb_subThread.currentTextChanged.connect(self.on_cbb_subThread)
        self.cbb_subThread.currentTextChanged.connect(self.on_chb_changeSubthread)
        self.cbb_subThread.currentTextChanged.connect(self.on_chb_switchSubthread)
        self.chb_startStopSubthread.toggled.connect(self.on_chb_startStopSubthread)
        self.dsb_subThreadParam0.valueChanged.connect(self.thrd.setParam0)
        self.dsb_subThreadParam1.valueChanged.connect(self.thrd.setParam1)
        self.dsb_subThreadParam2.valueChanged.connect(self.thrd.setParam2)
        self.dsb_subThreadParam3.valueChanged.connect(self.thrd.setParam3)
        self.dsb_subThreadParam4.valueChanged.connect(self.thrd.setParam4)
        self.dsb_desiredSeparation.valueChanged.connect(self.setDesiredSeparation)


    #=====================================================
    # Link GUI elements
    #=====================================================
    def linkWidgets(self):
        # link slider to doubleSpinBox
        self.dsb_x.valueChanged.connect(lambda value: self.hsld_x.setValue(int(value*100)))
        self.dsb_y.valueChanged.connect(lambda value: self.hsld_y.setValue(int(value*100)))
        self.dsb_z.valueChanged.connect(lambda value: self.hsld_z.setValue(int(value*100)))
        self.hsld_x.valueChanged.connect(lambda value: self.dsb_x.setValue(float(value/100)))
        self.hsld_y.valueChanged.connect(lambda value: self.dsb_y.setValue(float(value/100)))
        self.hsld_z.valueChanged.connect(lambda value: self.dsb_z.setValue(float(value/100)))

        self.dsb_xGradient.valueChanged.connect(lambda value: self.hsld_xGradient.setValue(int(value*100)))
        self.dsb_yGradient.valueChanged.connect(lambda value: self.hsld_yGradient.setValue(int(value*100)))
        self.dsb_zGradient.valueChanged.connect(lambda value: self.hsld_zGradient.setValue(int(value*100)))
        self.hsld_xGradient.valueChanged.connect(lambda value: self.dsb_xGradient.setValue(float(value/100)))
        self.hsld_yGradient.valueChanged.connect(lambda value: self.dsb_yGradient.setValue(float(value/100)))
        self.hsld_zGradient.valueChanged.connect(lambda value: self.dsb_zGradient.setValue(float(value/100)))
    #=====================================================
    # Thread Example
    #=====================================================
    def setupSubThread(self,field,vision,joystick=None):
        if joystick:
            self.thrd = SubThread(field,vision,joystick)
        else:
            self.thrd = SubThread(field,vision)
        self.thrd.statusSignal.connect(self.updateSubThreadStatus)
        self.thrd.finished.connect(self.finishSubThreadProcess)

    # updating GUI according to the status of the subthread
    @pyqtSlot(str)
    def updateSubThreadStatus(self, receivedStr):
        print('Received message from subthread: ',receivedStr)
        # show something on GUI

    # run when the subthread is termianted
    @pyqtSlot()
    def finishSubThreadProcess(self):
        print('Subthread is terminated.')

        vision.clearDrawingRouting()
        self.clearField()
        # disable some buttons etc.

    #=====================================================
    # Real time plot
    # This is showing actual coil current that is stored in field.x, field.y, field.z
    # Note: the figure is updating at the speed of self.updateRate defined in _init_
    #=====================================================
    def setupRealTimePlot(self):
        self.realTimePlot = CustomFigCanvas()
        self.LAYOUT_A.addWidget(self.realTimePlot, *(0,0)) # put the preview window in the layout
        self.btn_zoom.clicked.connect(self.realTimePlot.zoom) # connect qt signal to zoom funcion

    def updatePlot(self):
        #print(field.y)
        self.realTimePlot.addDataX(field.x)
        self.realTimePlot.addDataY(field.y)
        self.realTimePlot.addDataZ(field.z)

    #=====================================================
    # Callback Functions
    #=====================================================
    # General control tab
    def setFieldXYZ(self):
        field.setX(self.dsb_x.value())
        field.setY(self.dsb_y.value())
        field.setZ(self.dsb_z.value())
        field.setMagnitude(round(sqrt(pow(self.dsb_x.value(),2)+pow(self.dsb_y.value(),2)+pow(self.dsb_z.value(),2)),2))

    def clearField(self):
        self.dsb_x.setValue(0)
        self.dsb_y.setValue(0)
        self.dsb_z.setValue(0)
        self.dsb_xGradient.setValue(0)
        self.dsb_yGradient.setValue(0)
        self.dsb_zGradient.setValue(0)
        field.setXYZ(0,0,0)

    def setFieldXYZGradient(self):
        field.setXGradient(5,self.dsb_xGradient.value())
        field.setYGradient(5,self.dsb_yGradient.value())
        field.setZGradient(5,self.dsb_zGradient.value())
        field.setMagnitude(0)

    # vision tab
    def on_chb_bypassFilters(self,state):
        vision.setStateFiltersBypassed(state)

    def on_btn_refreshFilterRouting(self):
        vision.createFilterRouting(self.editor_vision.toPlainText().splitlines())

    def on_btn_snapshot(self):
        vision.setStateSnapshotEnabled(True)

    def on_chb_objectDetection(self,state):
        algorithm = self.cbb_objectDetectionAlgorithm.currentText()
        vision.setStateObjectDetection(state,algorithm)
        self.cbb_objectDetectionAlgorithm.setEnabled(not state)
        vision.clearAgentDrawing(state)

    # subthread
    def on_cbb_subThread(self,subThreadName):
        # an array that stores the name for params. Return param0, param1, ... if not defined.
        labelNames = self.thrd.labelOnGui.get(subThreadName,self.thrd.labelOnGui['default'])
        minVals = self.thrd.minOnGui.get(subThreadName,self.thrd.minOnGui['default'])
        maxVals = self.thrd.maxOnGui.get(subThreadName,self.thrd.maxOnGui['default'])
        defaultVals = self.thrd.defaultValOnGui.get(subThreadName,self.thrd.defaultValOnGui['default'])
        for i in range(5):
            targetLabel = 'lbl_subThreadParam' + str(i)
            targetSpinbox = 'dsb_subThreadParam' + str(i)
            getattr(self,targetLabel).setText(labelNames[i])
            getattr(self,targetSpinbox).setMinimum(minVals[i])
            getattr(self,targetSpinbox).setMaximum(maxVals[i])
            getattr(self,targetSpinbox).setValue(defaultVals[i])


    def on_chb_startStopSubthread(self,state):
        subThreadName = self.cbb_subThread.currentText()
        if state:
            self.cbb_subThread.setEnabled(True)
            self.thrd.setup(subThreadName)
            self.thrd.start()
            print('Subthread "{}" starts.'.format(subThreadName))
        else:
            self.cbb_subThread.setEnabled(True)
            field.setFrequency(0)
            field.setMagnitude(0)
            vision.setOrientation(None)
            vision.setSeparation(None)
            self.thrd.stop()

    def on_chb_changeSubthread(self): #use this fcn to stop current subthread
        subThreadName = self.cbb_subThread.currentText()
        if self.chb_startStopSubthread.isChecked() == True:
            self.thrd.stop()
            #print('Subthread "{}" starts.'.format(subThreadName))

    def on_chb_switchSubthread(self): #use this fcn to start new subthread
        subThreadName = self.cbb_subThread.currentText()
        if self.chb_startStopSubthread.isChecked() == True:
            time.sleep(0.1) #use time lag to achieve stop -> restart in order
            self.thrd.setup(subThreadName)
            self.thrd.start()
            print('Subthread "{}" starts.'.format(subThreadName))

    def setDesiredSeparation(self,val):
        vision.setSeparation(val)
