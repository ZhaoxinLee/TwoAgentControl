
"""
=============================================================================
subThread.py
----------------------------------------------------------------------------
Tips
If you are using Atom, use Ctrl+Alt+[ to fold all the funcitons.
Make your life easier.
----------------------------------------------------------------------------
[GitHub] : https://github.com/atelier-ritz
=============================================================================
"""
import sys
import time
from mathfx import *
from math import pi, sin, cos, sqrt, atan2, degrees,radians
from PyQt5.QtCore import pyqtSignal, QMutexLocker, QMutex, QThread
from numpy import sign

def subthreadNotDefined():
    print('Subthread not defined.')
    return

class SubThread(QThread):
    statusSignal = pyqtSignal(str)

    def __init__(self,field,vision,joystick=None,parent=None,):
        super(SubThread, self).__init__(parent)   # what is this line for Jason>>>?
        self.stopped = False
        self.mutex = QMutex()
        self.field = field
        self.vision = vision
        self.joystick = joystick
        self._subthreadName = ''
        self.params = [0,0,0,0,0]
        self.counter = 0
        self.state = None
        self.prex0 = None
        self.prey0 = None
        self.prex1 = None
        self.prey1 = None
        self.labelOnGui = {'twistField': ['Frequency (Hz)','Magniude (mT)','AzimuthalAngle (deg)','PolarAngle (deg)','SpanAngle (deg)'],
                        'rotateFBControl': ['Frequency (Hz)','Magniude (mT)','Separation(μm)','PGain','k'],
                        'rotateXY':['Frequency (Hz)','Magniude (mT)','N/A','N/A','N/A'],
                        'rotateYZ': ['Frequency (Hz)','Magniude (mT)','N/A','N/A','N/A'],
                        'rotateXZ': ['Frequency (Hz)','Magniude (mT)','N/A','N/A','N/A'],
                        'osc_saw': ['Frequency (Hz)','bound1 (mT)','bound2 (mT)','Azimuth [0,360] (deg)','Polar [-90,90] (deg)'],
                        'osc_triangle': ['Frequency (Hz)','bound1 (mT)','bound2 (mT)','Azimuth [0,360] (deg)','Polar [-90,90] (deg)'],
                        'osc_square': ['Frequency (Hz)','bound1 (mT)','bound2 (mT)','Azimuth [0,360] (deg)','Polar [-90,90] (deg)'],
                        'osc_sin': ['Frequency (Hz)','bound1 (mT)','bound2 (mT)','Azimuth [0,360] (deg)','Polar [-90,90] (deg)'],
                        'oni_cutting': ['Frequency (Hz)','Magnitude (mT)','angleBound1 (deg)','angleBound2 (deg)','N/A'],
                        'examplePiecewiseFunction': ['Frequency (Hz)','Magnitude (mT)','angle (deg)','period1 (0-1)','period2 (0-1)'],
                        'ellipse': ['Frequency (Hz)','Azimuthal Angle (deg)','B_horzF (mT)','B_vert (mT)','B_horzB (mT)'],
                        'drawing': ['pattern ID','offsetX','offsetY','N/A','N/A'],
                        'swimmerPathFollowing': ['Frequency (Hz)','Magniude (mT)','temp angle','N/A','N/A'],
                        'swimmerBenchmark': ['bias angle (deg)','N/A','N/A','N/A','N/A'],
                        'tianqiGripper': ['N/A','Magnitude (mT)','Frequency (Hz)','Direction (deg)','N/A'],
                        'bangbang':['Frequency (Hz)','Magniude (mT)','N/A','N/A','N/A'],
                        'binaryControllerXY':['Separation(μm)','PairAngle(degrees)','PGain','N/A','N/A'],
                        'rotateControl':['Frequency(Hz)','Magnitude(mT)','Separation(μm)','PairAngle(degrees)','N/A'],
                        'default':['param0','param1','param2','param3','param4'],
                        'procedure':['N/A','N/A','N/A','N/A','N/A']}
        self.defaultValOnGui = {
                        'twistField': [0,0,0,0,0],
                        'drawing': [0,0,0,1,0],
                        'swimmerPathFollowing': [-20,2,0,0,0],
                        'tianqiGripper': [0,15,0.5,0,0],
                        'rotateFBControl':[8,8,120,2,-0.1],
                        'rotateXY':[20,8,0,0,0],
                        'rotateXZ':[2,2,0,0,0],
                        'rotateYZ':[3,3,0,0,0],
                        'bangbang':[5,0,0,0,0],
                        'binaryControllerXY':[120,0,25,0,0],
                        'rotateControl':[8,8,120,0,0],
                        'default':[0,0,0,0,0],
                        'procedure':[0,0,0,0,0]
                        }
        self.minOnGui = {'twistField': [-100,0,-1080,0,0],
                        'rotateFBControl': [-100,0,50,0,-1],
                        'rotateXY': [-100,0,0,0,0],
                        'rotateYZ': [-100,0,0,0,0],
                        'rotateXZ': [-100,0,0,0,0],
                        'osc_saw': [-100,-20,-20,0,-90],
                        'osc_triangle': [-100,-20,-20,0,-90],
                        'osc_square': [-100,-20,-20,0,-90],
                        'osc_sin': [-100,-20,-20,0,-90],
                        'oni_cutting': [-100,-14,-720,-720,0],
                        'ellipse': [-100,-720,0,0,0],
                        'examplePiecewiseFunction': [-20,0,-360,0,0],
                        'swimmerPathFollowing': [-100,0,0,0,0],
                        'tianqiGripper': [0,0,0,-720,0],
                        'bangbang':[-100,0,0,0,0],
                        'binaryControllerXY':[15,0,0,0,0],
                        'rotateControl':[-100,0,50,0,0],
                        'default':[0,0,0,0,0],
                        'procedure':[0,0,0,0,0]}
        self.maxOnGui = {'twistField': [100,14,1080,180,360],
                        'rotateFBControl': [100,14,200,4,1],
                        'rotateXY': [100,14,0,0,0],
                        'rotateYZ': [100,14,0,0,0],
                        'rotateXZ': [100,14,0,0,0],
                        'osc_saw': [100,20,20,360,90],
                        'osc_triangle': [100,20,20,360,90],
                        'osc_square': [100,20,20,360,90],
                        'osc_sin': [100,20,20,360,90],
                        'oni_cutting': [100,14,720,720,0],
                        'ellipse': [100,720,20,20,20],
                        'examplePiecewiseFunction': [20,20,360,1,1],
                        'drawing':[2,1000,1000,10,0],
                        'swimmerPathFollowing': [100,20,360,0,0],
                        'swimmerBenchmark': [360,0,0,0,0],
                        'tianqiGripper': [10,20,120,720,0],
                        'bangbang':[100,14,0,0,0],
                        'binaryControllerXY':[200,360,100,0,0],
                        'rotateControl':[100,14,200,360,0],
                        'default':[0,0,0,0,0],
                        'procedure':[0,0,0,0,0]}

    def setup(self,subThreadName):
        self._subthreadName = subThreadName
        self.stopped = False

    def stop(self):
        with QMutexLocker(self.mutex):
            self.stopped = True

    def run(self):
        subthreadFunction = getattr(self,self._subthreadName,subthreadNotDefined)
        subthreadFunction()

    def setParam0(self,val): self.params[0] = val
    def setParam1(self,val): self.params[1] = val
    def setParam2(self,val): self.params[2] = val
    def setParam3(self,val): self.params[3] = val
    def setParam4(self,val): self.params[4] = val

    #=========================================
    # Start defining your subthread from here
    #=========================================
    def drawing(self):
        """
        An example of drawing lines and circles in a subThread
        (Not in object detection)
        """
        #=============================
        # reference params
        # 0 'Path ID'
        # 1 'offsetX'
        # 2 'offsetY'
        # 3 'scale'
        #=============================
        startTime = time.time()
        # video writing feature
        self.vision.startRecording('drawing.avi')
        while True:
            self.vision.clearDrawingRouting() # if we don't run this in a while loop, it freezes!!! (because *addDrawing* keeps adding new commands)
            self.vision.addDrawing('pathUT', self.params)
            self.vision.addDrawing('circle',[420,330,55])
            self.vision.addDrawing('arrow',[0,0,325,325])
            # you can also do somthing like:
            # drawing an arrow from "the robot" to "the destination point"
            t = time.time() - startTime # elapsed time (sec)
            self.field.setX(0)
            self.field.setY(0)
            self.field.setZ(0)
            if self.stopped:
                self.vision.stopRecording()
                return

    def swimmerPathFollowing(self):
        '''
        An example of autonomous path following of a sinusoidal swimmer at air-water interfaceself.
        This example follows the path "M".
        '''
        #=============================
        # reference params
        # 0 'Frequency (Hz)'
        # 1 'Magnitude (mT)'
        # 3 'temp angle'
        #=============================
        # video writing feature
        self.vision.startRecording('path.avi')
        startTime = time.time()
        state = 0 # indicates which goal point the robot is approaching. e.g. state0 -> approaching goalsX[0], goalsY[0]
        rect = [640,480] # size of the image. Format: width, height.
        pointsX = [0.2,0.3,0.4,0.5,0.6,0.7,0.8] # normalized position [0,1]
        pointsY = [0.7,0.3,0.3,0.7,0.3,0.3,0.7] # normalized position [0,1]
        goalsX = [int(rect[0]* i) for i in pointsX] # actual position (pixel)
        goalsY = [int(rect[1]* i) for i in pointsY] # actual position (pixel)
        tolerance = 10
        # It is considered that the robot has reached the point once the distance is less than *tolerance*
        toleranceDeviation = 30
        # Path correction is necessary when deviation exceeds this value.
        magnitudeCorrection = 1
        # Slow down the speed of the robot t oavoid overshoot when it is close to goal points
        while True:
            # obtain positions
            x = self.vision.agent1.x # curent position of the robot
            y = self.vision.agent1.y
            goalX = goalsX[state] # must be int
            goalY = goalsY[state] # must be int
            goalXPrevious = goalsX[state-1] # must be int
            goalYPrevious = goalsY[state-1] # must be int

            # draw reference lines
            self.vision.clearDrawingRouting() # if we don't run this in a while loop, it freezes!!! (because *addDrawing* keeps adding new commands)
            self.vision.addDrawing('closedPath',[goalsX,goalsY])
            self.vision.addDrawing('circle',[goalX,goalY,5])
            self.vision.addDrawing('line',[x,y,goalX,goalY])

            #=======================================================
            # calculate the heading angle
            #=======================================================
            distance = distanceBetweenPoints(x,y,goalX,goalY)
            footX, footY = perpendicularFootToLine(x,y,goalXPrevious,goalYPrevious,goalX,goalY)
            deviation = distanceBetweenPoints(x,y,footX,footY)
            if deviation > toleranceDeviation:
                # moving perpendicular to the line
                angle = degrees(atan2(-(footY-y),footX-x))
            else:
                angleRobotToGoal = atan2(-(goalY-y),goalX-x)
                angleRobotToFoot = atan2(-(footY-y),footX-x)
                angleCorrectionOffset = normalizeAngle(angleRobotToFoot - angleRobotToGoal) * deviation / toleranceDeviation
                angle = degrees(angleRobotToGoal + angleCorrectionOffset)
                # print(angleRobotToGoal,angle)

            if distance <= tolerance * 3:
                magnitudeCorrection = 0.5
            else:
                magnitudeCorrection = 1
            # check if it has reached the goal point
            if distance <= tolerance:
                state += 1
                print('>>> Step to point {} <<<'.format(state))


            # apply magnetic field
            t = time.time() - startTime # elapsed time (sec)
            theta = 2 * pi * self.params[0] * t
            fieldX = magnitudeCorrection * self.params[1] * cos(theta) * cosd(angle+self.params[2])
            fieldY = magnitudeCorrection * self.params[1] * cos(theta) * sind(angle+self.params[2])
            fieldZ = magnitudeCorrection * self.params[1] * sin(theta)
            self.field.setX(fieldX)
            self.field.setY(fieldY)
            self.field.setZ(fieldZ)
            if self.stopped or state == len(pointsX):
                self.vision.stopRecording()
                return

    def tianqiGripper(self):
        #=============================
        # reference params
        # 0 'N/A'
        # 1 'Magnitude (mT)'
        # 2 'Frequency (Hz)'
        #=============================

        # ''' Video Recording '''
        # self.vision.startRecording('TianqiGripper.avi')
        ''' Init '''
        startTime = time.time()
        paramSgnMagZ = 1 # use R1 button to change the sign of Z magnitude
        paramFieldScale = 1 # change the field strength with R2
        ''' Rotating the gripper '''
        paramRotationOffsetTime = 0 # used to avoid sudden changes while switching to rotating mode
        paramRotationPhase = 0 # used for MODE3 - Fine rotation control
        ''' Modes '''
        mode = 0 # change the mode with buttons on PS3 controller
        BUTTON_RESPONSE_TIME = 0.2 # at least 0.2 sec between button triggers
        lastButtonPressedTimeMode = 0
        lastButtonPressedTimeR1 = 0 # the last time that the user changing the mode

        while True:
            t = time.time() - startTime # elapsed time (sec)
            # =======================================================
            # Detect Button Pressed to Change the MODE
            # =======================================================
            if t - lastButtonPressedTimeMode > BUTTON_RESPONSE_TIME:
                if self.joystick.isPressed('CROSS') and not mode == 0:
                    lastButtonPressedTimeMode = t
                    mode = 0
                    print('[MODE] Standby')
                elif self.joystick.isPressed('CIRCLE') and not mode == 1:
                    lastButtonPressedTimeMode = t
                    mode = 1
                    print('[MODE] Grasp')
                elif self.joystick.isPressed('TRIANGLE') and not mode == 2:
                    lastButtonPressedTimeMode = t
                    mode = 2
                    print('[MODE] Transport Auto')
                    paramRotationOffsetTime = t
                elif self.joystick.isPressed('SQUARE') and not mode == 3:
                    lastButtonPressedTimeMode = t
                    mode = 3
                    print('[MODE] Transport Manual')
                    paramRotationPhase = pi / 2
            # =======================================================
            # Flip direction of Z field
            # =======================================================
            if t - lastButtonPressedTimeR1 > BUTTON_RESPONSE_TIME:
                if self.joystick.isPressed('R1'):
                    lastButtonPressedTimeR1 = t
                    paramSgnMagZ = - paramSgnMagZ
                    print('The sign of fieldZ is {}'.format(paramSgnMagZ))
            # =======================================================
            # change magnitude of field with R2
            # =======================================================
            rawR2 = self.joystick.getStick(5) # -1 -> 1
            paramFieldScale = 0.5 * (- rawR2 + 1)
            # =======================================================
            # Process fieldXYZ in each mode
            # =======================================================
            if mode == 0:
                fieldX = 0
                fieldY = 0
                fieldZ = 0
            elif mode == 1:
                polar = self.joystick.getTiltLeft()
                azimuth = self.joystick.getAngleLeft()
                fieldX = self.params[1] * cosd(polar) * cosd(azimuth)
                fieldY = self.params[1] * cosd(polar) * sind(azimuth)
                fieldZ = self.params[1] * sind(polar)
            elif mode == 2:
                theta = - 2 * pi * self.params[2] * (t - paramRotationOffsetTime) + pi / 2
                fieldX = self.params[1] * cos(theta) * cosd(self.joystick.getAngleLeft())
                fieldY = self.params[1] * cos(theta) * sind(self.joystick.getAngleLeft())
                fieldZ = self.params[1] * sin(theta)
            elif mode == 3:
                if t - lastButtonPressedTimeMode > BUTTON_RESPONSE_TIME:
                    if self.joystick.isPressed('SQUARE'):
                        lastButtonPressedTimeMode = t
                        if self.joystick.isPressed('L1'):
                            paramRotationPhase = paramRotationPhase + pi/16
                        else:
                            paramRotationPhase = paramRotationPhase - pi/16
                fieldX = self.params[1] * cos(paramRotationPhase) * cosd(self.joystick.getAngleLeft())
                fieldY = self.params[1] * cos(paramRotationPhase) * sind(self.joystick.getAngleLeft())
                fieldZ = self.params[1] * sin(paramRotationPhase)

            self.field.setX(fieldX * paramFieldScale)
            self.field.setY(fieldY * paramFieldScale)
            self.field.setZ(fieldZ * paramFieldScale * paramSgnMagZ)
            if self.stopped:
                # self.vision.stopRecording()
                return

    def swimmerBenchmark(self):
        '''
        An example of testing the velocity of the swimmer with respect to frequency and magnitude.
        It demonstrates:
            - path following: Point0 -> Point1 -> Point0
            - do the same path following task for different frequencies. (Benchmarking *velocity* vs *frequency*)
            - draw lines and circles on the frame in real time

        '''
        # video writing feature
        self.vision.startRecording('benchmark.avi')
        startTime = time.time()
        state = 0 # indicates which goal point the robot is approaching. e.g. state0 -> approaching goalsX[0], goalsY[0]
        freq = [-15,-15,-17,-19,-21,-23,-25] # the first frequency is the freq that the robot is heading to the start point.
        freq = [i - 8 for i in freq] # the first frequency is the freq that the robot is heading to the start point.
        magnitude = 8
        benchmarkState = 0 # indicates which frequency the program is testing

        rect = [640,480] # size of the image. Format: width, height.
        pointsX = [0.2,0.8] # normalized position [0,1]
        pointsY = [0.2,0.8] # normalized position [0,1]
        goalsX = [int(rect[0]* i) for i in pointsX] # actual position (pixel)
        goalsY = [int(rect[1]* i) for i in pointsY] # actual position (pixel)
        tolerance = 20 # It is considered that the robot has reached the point once the distance is less than *tolerance*
        print('Moving to the home position. Frequency {} Hz'.format(freq[benchmarkState]))
        while True:
            # obtain positions
            x = self.vision.agent1.x
            y = self.vision.agent1.y
            goalX = goalsX[state] # must be int
            goalY = goalsY[state] # must be int

            # draw reference lines
            self.vision.clearDrawingRouting() # if we don't run this in a while loop, it freezes!!! (because *addDrawing* keeps adding new commands)
            self.vision.addDrawing('closedPath',[goalsX,goalsY])
            self.vision.addDrawing('circle',[goalX,goalY,5])
            self.vision.addDrawing('line',[x,y,goalX,goalY])

            # calculate distance and angle
            distance = sqrt((goalX - x)**2 + (goalY - y)**2)
            angle = degrees(atan2(-(goalY-y),goalX-x))   # computers take top left point as (0,0)


            # check if it has reached the goal point
            if distance <= tolerance:
                # if at the starting point, start a new round of benchmark test
                if state == 0:
                    benchmarkState += 1
                    if benchmarkState < len(freq):
                        print('Case {} - Benchmark Frequency {} Hz'.format(benchmarkState,freq[benchmarkState]))
                state += 1  # move to the next target point
                # if the path is finished, set the home position as the next goal point
                if state == len(pointsX):
                    state = 0
                # if the benchmark is finished, sdo not display the next point
                if benchmarkState < len(freq):
                    print('    >>> Step to point {} <<<'.format(state))

            # apply magnetic field
            t = time.time() - startTime # elapsed time (sec)
            theta = 2 * pi * freq[benchmarkState] * t
            fieldX = magnitude * cos(theta) * cosd(angle+self.params[0])
            fieldY = magnitude * cos(theta) * sind(angle+self.params[0])
            fieldZ = magnitude * sin(theta)
            self.field.setX(fieldX)
            self.field.setY(fieldY)
            self.field.setZ(fieldZ)
            if self.stopped or benchmarkState == len(freq):
                self.vision.stopRecording()
                return

    def examplePiecewiseFunction(self):
        """
        This function shows an example of a piecewise function.
        It first convert time into normalizedTime (range [0,1)).
        Values are selected based on *normT*.
        This makes it easier to change frequency without modifying the shape of the funciton.
        """
        #=============================
        # reference params
        # 0 'Frequency (Hz)'
        # 1 'Magnitude (mT)'
        # 2 'angle (deg)'
        # 3 'period1 (0-1)'
        # 4 'period2 (0-1)'
        #=============================
        startTime = time.time()

        while True:
            t = time.time() - startTime # elapsed time (sec)
            normT = normalizeTime(t,self.params[0]) # 0 <= normT < 1
            if normT < self.params[3]:
                magnitude = self.params[1] / oscX_sawself.params[3] * normT
                angle = 180
            elif normT < self.params[4]:
                magnitude = self.params[1]
                angle = (180 - self.params[2])/(self.params[3] - self.params[4]) * (normT - self.params[3]) + 180
            else:
                magnitude = self.params[1] / (self.params[4] - 1) * (normT - 1)
                angle = self.params[2]
            fieldX = magnitude * sind(angle)
            fieldY = 0
            fieldZ = magnitude * cosd(angle)
            self.field.setX(fieldX)
            self.field.setY(fieldY)
            self.field.setZ(fieldZ)
            if self.stopped:
                return

    def ellipse(self):
        #=============================
        # reference params
        # 0 'Frequency (Hz)'
        # 1 'azimuth (deg)'
        # 2 'B_horzF (mT)'
        # 3 'B_vert (mT)'
        # 4 'B_horzB (mT)'
        #=============================
        startTime = time.time()
        counter = 0
        record = ''
        while True:
            t = time.time() - startTime # elapsed time (sec)
            theta = 2 * pi * self.params[0] * t
            normT = normalizeTime(t,self.params[0]) # 0 <= normT < 1
            if normT < 0.5:
                B_horz = self.params[2] * cos(theta)
            else:
                B_horz = self.params[4] * cos(theta)
            fieldX = B_horz * cosd(self.params[1])
            fieldY = B_horz * sind(self.params[1])
            fieldZ = self.params[3] * sin(theta)
            self.field.setX(fieldX)
            self.field.setY(fieldY)
            self.field.setZ(fieldZ)
            # save to txt
            counter += 1
            if counter > 10:
                counter = 0
                record = record + '{:.5f}, {:.2f}, {:.2f}, {:.2f}, {}, {}\n'.format(t,self.field.x,self.field.y,self.field.z,self.vision.agent1.x,self.vision.agent1.y)
            if self.stopped:
                text_file = open("Output.txt", "w")
                text_file.write(record)
                text_file.close()
                return

    def oni_cutting(self):
        #=============================
        # reference params
        # 0 'Frequency (Hz)'
        # 1 'Magnitude (mT)'
        # 2 'angleBound1 (deg)'
        # 3 'angleBound2 (deg)'
        #=============================
        startTime = time.time()
        while True:
            t = time.time() - startTime # elapsed time (sec)
            angle = oscBetween(t,'sin',self.params[0],self.params[2],self.params[3])
            fieldX = self.params[1] * cosd(angle)
            fieldY = self.params[1] * sind(angle)
            self.field.setX(fieldX)
            self.field.setY(fieldY)
            self.field.setZ(0)
            if self.stopped:
                return

    def twistField(self):
        ''' credit to Omid '''
        #=============================
        # reference params
        # 0 'Frequency (Hz)'
        # 1 'Magniude (mT)'
        # 2 'AzimuthalAngle (deg)'
        # 3 'PolarAngle (deg)'
        # 4 'SpanAngle (deg)'
        #=============================
        startTime = time.time()
        record = 'Time(s), FieldX(mT), FiledY(mT), FieldZ(mT), X(pixel), Y(pixel) \n' # output to a txt file
        counter = 0
        while True:
            t = time.time() - startTime # elapsed time (sec)
            fieldX = self.params[1]* ( cosd(self.params[2])*cosd(self.params[3])*cosd(90-self.params[4]*0.5)*cos(2*pi*self.params[0]*t) - sind(self.params[2])*cosd(90-self.params[4]*0.5)*sin(2*pi*self.params[0]*t) + cosd(self.params[2])*sind(self.params[3])*cosd(self.params[4]*0.5));
            fieldY = self.params[1]* ( sind(self.params[2])*cosd(self.params[3])*cosd(90-self.params[4]*0.5)*cos(2*pi*self.params[0]*t) + cosd(self.params[2])*cosd(90-self.params[4]*0.5)*sin(2*pi*self.params[0]*t) + sind(self.params[2])*sind(self.params[3])*cosd(self.params[4]*0.5));
            fieldZ = self.params[1]* (-sind(self.params[3])*cosd(90-self.params[4]*0.5)*cos(2*pi*self.params[0]*t) + cosd(self.params[3])*cosd(self.params[4]*0.5));
            self.field.setX(fieldX)
            self.field.setY(fieldY)
            self.field.setZ(fieldZ)
            # save to txt
            counter += 1
            if counter > 300:
                counter = 0
                record = record + '{:.5f}, {:.2f}, {:.2f}, {:.2f}, {}, {}\n'.format(t,self.field.x,self.field.y,self.field.z,self.vision.agent1.x,self.vision.agent1.y)
            if self.stopped:
                text_file = open("Output.txt", "w")
                text_file.write(record)
                text_file.close()
                return

    def osc_saw(self):
        #=============================
        # reference params
        # 0 'Frequency (Hz)'
        # 1 'Lowerbound (mT)'
        # 2 'Upperbound (mT)'
        # 3 'Azimuthal Angle (deg)'
        # 4 'Polar Angle (deg)'
        #=============================
        startTime = time.time()
        while True:
            t = time.time() - startTime # elapsed time (sec)
            magnitude = oscBetween(t,'saw',self.params[0],self.params[1],self.params[2])
            fieldZ = magnitude * sind(self.params[4])
            fieldX = magnitude * cosd(self.params[4]) * cosd(self.params[3])
            fieldY = magnitude * cosd(self.params[4]) * sind(self.params[3])
            self.field.setX(fieldX)
            self.field.setY(fieldY)
            self.field.setZ(fieldZ)
            if self.stopped:
                return

    def osc_triangle(self):
        #=============================
        # reference params(200,255)
        # 0 'Frequency (Hz)'
        # 1 'Lowerbound (mT)'
        # 2 'Upperbound (mT)'
        # 3 'Azimuthal Angle (deg)'
        # 4 'Polar Angle (deg)'
        #=============================
        startTime = time.time()
        while True:
            t = time.time() - startTime # elapsed time (sec)
            magnitude = oscBetween(t,'triangle',self.params[0],self.params[1],self.params[2])
            fieldZ = magnitude * sind(self.params[4])
            fieldX = magnitude * cosd(self.params[4]) * cosd(self.params[3])
            fieldY = magnitude * cosd(self.params[4]) * sind(self.params[3])
            self.field.setX(fieldX)
            self.field.setY(fieldY)
            self.field.setZ(fieldZ)
            if self.stopped:
                return

    def osc_square(self):
        #=============================
        # reference params
        # 0 'Frequency (Hz)'
        # 1 'Lowerbound (mT)'
        # 2 'Upperbound (mT)'
        # 3 'Azimuthal Angle (deg)'
        # 4 'Polar Angle (deg)'
        #=============================
        startTime = time.time()
        while True:
            t = time.time() - startTime # elapsed time (sec)
            magnitude = oscBetween(t,'square',self.params[0],self.params[1],self.params[2])
            fieldZ = magnitude * sind(self.params[4])
            fieldX = magnitude * cosd(self.params[4]) * cosd(self.params[3])
            fieldY = magnitude * cosd(self.params[4]) * sind(self.params[3])
            self.field.setX(fieldX)
            self.field.setY(fieldY)
            self.field.setZ(fieldZ)
            if self.stopped:
                return

    def osc_sin(self):
        #=============================
        # reference params
        # 0 'Frequency (Hz)'
        # 1 'Lowerbound (mT)'
        # 2 'Upperbound (mT)'
        # 3 'Azimuthal Angle (deg)'
        # 4 'Polar Angle (deg)'
        #=============================
        startTime = time.time()
        while True:
            t = time.time() - startTime # elapsed time (sec)
            magnitude = oscBetween(t,'sin',self.params[0],self.params[1],self.params[2])
            fieldZ = magnitude * sind(self.params[4])
            fieldX = magnitude * cosd(self.params[4]) * cosd(self.params[3])
            fieldY = magnitude * cosd(self.params[4]) * sind(self.params[3])
            self.field.setX(fieldX)
            self.field.setY(fieldY)
            self.field.setZ(fieldZ)
            if self.stopped:
                return

    def rotateFBControl(self):
        #=============================
        # reference params
        # 0 'Frequency (Hz)'
        # 1 'Magniude (mT)'
        #=============================
        startTime = time.time()
        separationArray = []
        i = 0
        while True:
            t = time.time() - startTime # elapsed time (sec)

            #get coordinates of two agents
            x0 = self.vision.agent[0].x
            y0 = self.vision.agent[0].y
            x1 = self.vision.agent[1].x
            y1 = self.vision.agent[1].y

            # set up gradient field to achieve pulling
            realCOM = [int((x0+x1)/2),int((y0+y1)/2)]
            desiredCOM = self.vision.getDesiredCOM()
            vector = [desiredCOM[0]-realCOM[0],desiredCOM[1]-realCOM[1]]
            gradientX = 0.05*vector[0]
            if gradientX > 2.5:
                gradientX = 2.5
            elif gradientX < -2.5:
                gradientX = -2.5

            gradientY = -0.05*vector[1]
            if gradientY > 2.5:
                gradientY = 2.5
            elif gradientY < -2.5:
                gradientY = -2.5

            realSeparation = sqrt(pow(x1-x0,2)+pow(y1-y0,2))
            desiredSeparation = self.params[2]
            self.vision.setSeparation(desiredSeparation)

            PCoefficient = self.params[3]
            sepDifference = realSeparation - desiredSeparation
            if desiredSeparation >= 120:
                frequency = -0.1*desiredSeparation + 34 - (sepDifference)*2
            elif 90 <= desiredSeparation <120:
                frequency = -0.3*desiredSeparation + 58 - (sepDifference)*2
            else:
                frequency = -0.3*desiredSeparation + 58 - (sepDifference)*1
            #frequency = self.params[4]*desiredSeparation + 19 - 120*self.params[4] - (sepDifference)*PCoefficient
            #frequency = (7/4000)*(desiredSeparation)^2-(5/8)*desiredSeparation+344/5 - (sepDifference)*PCoefficient

            # use uniform field when agents are far
            pairHeadingAngle = atan2(-(y1-y0),x1-x0)
            if frequency < 0:
                frequency = 0
                self.field.setFrequency(round(frequency,3))
                fieldX = 8*cos(pairHeadingAngle)
                fieldY = 8*sin(pairHeadingAngle)
            elif frequency > 50:
                frequency = 50
                self.field.setFrequency(round(frequency,3))
                self.field.setMagnitude(self.params[1])
                theta = 2 * pi * frequency * t
                fieldX = self.params[1] * cos(theta)
                fieldY = self.params[1] * sin(theta)
            else:
                self.field.setFrequency(round(frequency,3))
                self.field.setMagnitude(self.params[1])
                theta = 2 * pi * frequency * t
                fieldX = self.params[1] * cos(theta)
                fieldY = self.params[1] * sin(theta)
            #self.vision.setOrientation(None)
            print(frequency)

            # if self.prex0 != None and self.prey0 != None and self.prex1 != None and self.prey1 != None:
            #     if abs(self.prex0-x0)>0 or abs(self.prey0-y0)>0 or abs(self.prex1-x1)>0 or abs(self.prey1-y1)>0:
            #         sys.stdout = open('position_data_fb1.txt', 'a')
            #         print(x0,',',y0,',',x1,',',y1,',',round(realSeparation,3),',',round(degrees(pairHeadingAngle),3),',',round(t,3),',',round(desiredSeparation/10,1),',',round(frequency,3))

            self.storeLastFrame(x0,y0,x1,y1)

            # update time
            if self.counter == 1000:
                self.vision.setTime(round(t,3))
                self.counter = 0
            # if realSeparation > 20 and i > 1:
            #     #print(separationArray[self.counter])
            #     if separationArray[i-1]-separationArray[i-2] > 0:
                    #sys.stdout = open('position_data2.txt', 'a')
                    #print(x0,y0,x1,y1,round(realSeparation,3),round(degrees(pairHeadingAngle),3),round(t,3))
            self.counter += 1

            # if realSeparation >= 250:
            #     psi_local = 0
            # elif 20 < realSeparation:
            #     psi_local = pi/2
            # uniformX = 8*cos(pairHeadingAngle+psi_local)
            # uniformY = 8*sin(pairHeadingAngle+psi_local)

            self.field.setXGradient(fieldX,gradientX)
            self.field.setYGradient(fieldY,gradientY)
            self.field.setZ(0)

            if self.stopped:
                self.vision.setTime(0)
                self.counter = 0
                return

    # def rotateXY(self):
    #     startTime = time.time()
    #     while True:
    #         t = time.time() - startTime # elapsed time (sec)
    #
    #         #get coordinates of two agents
    #         x0 = self.vision.agent[0].x
    #         y0 = self.vision.agent[0].y
    #         x1 = self.vision.agent[1].x
    #         y1 = self.vision.agent[1].y
    #
    #         # set up gradient field to achieve pulling
    #         realCOM = [int((x0+x1)/2),int((y0+y1)/2)]
    #         desiredCOM = self.vision.getDesiredCOM()
    #         vector = [desiredCOM[0]-realCOM[0],desiredCOM[1]-realCOM[1]]
    #         gradientX = 0.05*vector[0]
    #         if gradientX > 2.5:
    #             gradientX = 2.5
    #         elif gradientX < -2.5:
    #             gradientX = -2.5
    #
    #         gradientY = -0.05*vector[1]
    #         if gradientY > 2.5:
    #             gradientY = 2.5
    #         elif gradientY < -2.5:
    #             gradientY = -2.5
    #
    #         realSeparation = sqrt(pow(x1-x0,2)+pow(y1-y0,2))
    #
    #         # use uniform field when agents are far
    #         pairHeadingAngle = atan2(-(y1-y0),x1-x0)
    #
    #
    #         # if self.prex0 != None and self.prey0 != None and self.prex1 != None and self.prey1 != None:
    #         #     if abs(self.prex0-x0)>0 or abs(self.prey0-y0)>0 or abs(self.prex1-x1)>0 or abs(self.prey1-y1)>0:
    #         #         sys.stdout = open('position_data_fb6_20hz.txt', 'a')
    #         #         print(x0,',',y0,',',x1,',',y1,',',round(realSeparation,3),',',round(degrees(pairHeadingAngle),3),',',round(t,3))
    #
    #         self.storeLastFrame(x0,y0,x1,y1)
    #
    #         # update time
    #         if self.counter == 1000:
    #             self.vision.setTime(round(t,3))
    #             self.counter = 0
    #         self.counter += 1
    #
    #         self.field.setFrequency(self.params[0])
    #         self.field.setMagnitude(self.params[1])
    #         theta = 2 * pi * self.params[0] * t
    #         fieldX = self.params[1] * cos(theta)
    #         fieldY = self.params[1] * sin(theta)
    #         self.field.setXGradient(fieldX,gradientX)
    #         self.field.setYGradient(fieldY,gradientY)
    #         self.field.setZ(0)
    #
    #         if self.stopped:
    #             return

    def rotateXY(self):
        #=============================
        # reference params
        # 0 'Frequency (Hz)'
        # 1 'Magniude (mT)'
        #=============================
        startTime = time.time()
        while True:
            t = time.time() - startTime # elapsed time (sec)
            self.field.setFrequency(self.params[0])
            self.field.setMagnitude(self.params[1])
            theta = 2 * pi * self.params[0] * t
            fieldX = self.params[1] * cos(theta)
            fieldY = self.params[1] * sin(theta)
            # set up gradient field to achieve pulling
            realCOM = [int((self.vision.agent[0].x+self.vision.agent[1].x)/2),int((self.vision.agent[0].y+self.vision.agent[1].y)/2)]
            desiredCOM = self.vision.getDesiredCOM()
            vector = [desiredCOM[0]-realCOM[0],desiredCOM[1]-realCOM[1]]
            gradientX = 0.05*vector[0]
            if gradientX > 2.5:
                gradientX = 2.5
            elif gradientX < -2.5:
                gradientX = -2.5

            gradientY = -0.05*vector[1]
            if gradientY > 2.5:
                gradientY = 2.5
            elif gradientY < -2.5:
                gradientY = -2.5


            self.field.setXGradient(fieldX,gradientX)
            self.field.setYGradient(fieldY,gradientY)
            self.field.setZ(0)
            self.field.setMagnitude(8)
            if self.stopped:
                return

    def rotateYZ(self):
        #=============================
        # reference params
        # 0 'Frequency (Hz)'
        # 1 'Magniude (mT)'
        #=============================
        startTime = time.time()
        while True:
            t = time.time() - startTime # elapsed time (sec)
            self.field.setFrequency(self.params[0])
            self.field.setMagnitude(self.params[1])
            theta = 2 * pi * self.params[0] * t
            fieldY = self.params[1] * cos(theta)
            fieldZ = self.params[1] * sin(theta)
            self.field.setX(0)
            self.field.setY(fieldY)
            self.field.setZ(fieldZ)
            if self.stopped:
                return

    def rotateXZ(self):
        #=============================
        # reference params
        # 0 'Frequency (Hz)'
        # 1 'Magniude (mT)'
        #=============================
        startTime = time.time()
        while True:
            t = time.time() - startTime # elapsed time (sec)
            self.field.setFrequency(self.params[0])
            self.field.setMagnitude(self.params[1])
            theta = 2 * pi * self.params[0] * t
            fieldX = self.params[1] * cos(theta)
            fieldZ = self.params[1] * sin(theta)
            self.field.setX(fieldX)
            self.field.setY(0)
            self.field.setZ(fieldZ)
            if self.stopped:
                return

    def bangbang(self):
        startTime = time.time()
        while True:
            t = time.time() - startTime
            self.field.setFrequency(self.params[0])
            self.field.setMagnitude(self.params[1])
            theta = 2 * pi * self.params[0] * t
            s = sign(sin(theta))
            fieldX = self.params[1] * cos(radians(54.72))
            fieldY = self.params[1] * sin(radians(s*54.72))
            self.field.setX(fieldX)
            self.field.setY(fieldY)
            self.field.setZ(0)
            if self.stopped:
                return

    def binaryControllerXY(self):
        startTime = time.time()
        while True:
            t = time.time() - startTime
            self.field.setFrequency(0)

            #rotating field
            #theta = 2 * pi * self.params[0] * t
            #uniformX = self.params[1] * cos(theta)
            #uniformY = self.params[1] * sin(theta)

            # set up uniform field to achieve binary control
            pairHeadingAngle = atan2(-(self.vision.agent[1].y-self.vision.agent[0].y),self.vision.agent[1].x-self.vision.agent[0].x)
            pixelOverDistance = 1
            realSeparation = sqrt(pow(self.vision.agent[1].x-self.vision.agent[0].x,2)+pow(self.vision.agent[1].y-self.vision.agent[0].y,2))
            desiredSeparation = self.params[0]
            self.vision.setSeparation(desiredSeparation)

            # set up for PControl to control pair heading angle when the separation between two agents is small
            desiredOrientation = radians(self.params[1])
            self.vision.setOrientation(desiredOrientation)
            sepDifference = realSeparation - desiredSeparation
            psi_0 = radians(54.74)
            PCoefficient = self.params[2]

            oriDifference = pairHeadingAngle - desiredOrientation
            if oriDifference >= 0:
                trans_sign = -1
            elif oriDifference <= -0:
                trans_sign = +1

            psi_local = psi_0 - (sepDifference)*PCoefficient*1e-3
            if psi_local > pi/2:
                psi_local = pi/2
            elif psi_local < 0:
                psi_local = 0
            else:
                psi_local = trans_sign * psi_local

            if realSeparation >= desiredSeparation+20:
                psi_local = 0
            elif realSeparation <= desiredSeparation-20:
                psi_local = pi/2

            uniformX = 8*cos(pairHeadingAngle+psi_local)
            uniformY = 8*sin(pairHeadingAngle+psi_local)

            self.vision.setLocalAngle(psi_local)

            # set up gradient field to achieve pulling
            realCOM = [int((self.vision.agent[0].x+self.vision.agent[1].x)/2),int((self.vision.agent[0].y+self.vision.agent[1].y)/2)]
            desiredCOM = self.vision.getDesiredCOM()
            vector = [desiredCOM[0]-realCOM[0],desiredCOM[1]-realCOM[1]]
            gradientX = 0.05*vector[0]
            if gradientX > 2.5:
                gradientX = 2.5
            elif gradientX < -2.5:
                gradientX = -2.5

            gradientY = -0.05*vector[1]
            if gradientY > 2.5:
                gradientY = 2.5
            elif gradientY < -2.5:
                gradientY = -2.5


            self.field.setXGradient(uniformX,gradientX)
            self.field.setYGradient(uniformY,gradientY)
            self.field.setZ(0)
            self.field.setMagnitude(8)

            # self.counter += 1
            # if realSeparation > 20:
            #     if self.counter == 1000:
            #         self.counter = 0
            #         sys.stdout = open('position_data.txt', 'a')
            #         print(self.vision.agent[0].x,self.vision.agent[0].y,self.vision.agent[1].x,self.vision.agent[1].y,round(t,3))


            if self.stopped:
                self.field.setMagnitude(0)
                return

            # previous code
            #currentOrientation = atan2(-(self.vision.agent[1].y-self.vision.agent[0].y),self.vision.agent[1].x-self.vision.agent[0].x)
            # if self.vision.agent[1].y <= self.vision.agent[0].y:
            #     pairHeadingAngle = atan2(-(self.vision.agent[1].y-self.vision.agent[0].y),self.vision.agent[1].x-self.vision.agent[0].x)
            # else:
            #     pairHeadingAngle = atan2(-(self.vision.agent[1].y-self.vision.agent[0].y),self.vision.agent[1].x-self.vision.agent[0].x)+pi
            # oriDifference = pairHeadingAngle - desiredOrientation
            # if oriDifference >= 0:
            #     trans_sign = -1
            # else:
            #     trans_sign = 1

            # if realSeparation > 20: #avoid antiswap time gap
            #     #print(sepDifference)
            #     sepDifference = realSeparation - desiredSeparation
            #     psi_0 = radians(54.74)
            #     PCoefficient = self.params[2]
            #     #psi_local = psi_0 + sepDifference*PCoefficient*1e-3
            #     if realSeparation >= 130:
            #         psi_local = 0
            #         uniformX = 8*cos(pairHeadingAngle+psi_local)
            #         uniformY = 8*sin(pairHeadingAngle+psi_local)
            #     elif realSeparation <= 70:
            #         psi_local = pi/2
            #         uniformX = 8*cos(pairHeadingAngle+psi_local)
            #         uniformY = 8*sin(pairHeadingAngle+psi_local)
            #     else:
            #         psi_local = trans_sign*(psi_0 + (sepDifference)*PCoefficient*1e-3)
            #         uniformX = 8*cos(pairHeadingAngle+psi_local)
            #         uniformY = 8*sin(pairHeadingAngle+psi_local)
            # else:
            #     uniformX = 0
            #     uniformY = 0


    def rotateControl(self):
        #=============================
        # reference params
        # 0 'Frequency (Hz)'
        # 1 'Magniude (mT)'
        #=============================
        startTime = time.time()
        separationArray = []
        i = 0
        while True:
            t = time.time() - startTime # elapsed time (sec)

            #get coordinates of two agents
            x0 = self.vision.agent[0].x
            y0 = self.vision.agent[0].y
            x1 = self.vision.agent[1].x
            y1 = self.vision.agent[1].y

            # set up gradient field to achieve pulling
            realCOM = [int((x0+x1)/2),int((y0+y1)/2)]
            desiredCOM = self.vision.getDesiredCOM()
            vector = [desiredCOM[0]-realCOM[0],desiredCOM[1]-realCOM[1]]
            gradientX = 0.05*vector[0]
            if gradientX > 2.5:
                gradientX = 2.5
            elif gradientX < -2.5:
                gradientX = -2.5

            gradientY = -0.05*vector[1]
            if gradientY > 2.5:
                gradientY = 2.5
            elif gradientY < -2.5:
                gradientY = -2.5


            # control the pair heading angle and separation
            pairHeadingAngle = atan2(-(y1-y0),x1-x0)
            desiredOrientation = radians(self.params[3])
            self.vision.setOrientation(desiredOrientation)
            oriDifference = pairHeadingAngle - desiredOrientation

            realSeparation = sqrt(pow(x1-x0,2)+pow(y1-y0,2))
            desiredSeparation = self.params[2]
            self.vision.setSeparation(desiredSeparation)
            sepDifference = realSeparation - desiredSeparation

            if desiredSeparation <= 110:
                threshold = 4
            elif desiredSeparation > 110 and desiredSeparation <= 150:
                threshold = 3
            else:
                threshold = 1

            if oriDifference >= radians(threshold):
                self.state = 1
                if desiredSeparation >= 160:
                    frequency = 0.15*(desiredSeparation) - 48 + (sepDifference-5)*3
                elif 120 <= desiredSeparation < 160:
                    frequency = 0.15*desiredSeparation - 48 + (sepDifference-1)*3
                elif 90 <= desiredSeparation < 120:
                    frequency = 0.3*desiredSeparation - 66 + (sepDifference)*2
                else:
                    frequency = 0.3*desiredSeparation - 66 + (sepDifference)*1

                # use uniform field when agents are far
                if frequency > 0:
                    frequency = 0
                    self.field.setFrequency(round(frequency,3))
                    fieldX = 8*cos(pairHeadingAngle)
                    fieldY = 8*sin(pairHeadingAngle)
                elif frequency < -60:
                    frequency = -60

            elif oriDifference < radians(-threshold):
                self.state = 2
                if desiredSeparation >= 160:
                    frequency = -0.15*(desiredSeparation) + 48 - (sepDifference-5)*3
                elif 120 <= desiredSeparation < 160:
                    frequency = -0.15*desiredSeparation + 48 - (sepDifference-1)*3
                elif 90 <= desiredSeparation <120:
                    frequency = -0.3*desiredSeparation + 66 - (sepDifference)*2
                else:
                    frequency = -0.3*desiredSeparation + 66 - (sepDifference)*1

                if frequency < 0:
                    frequency = 0
                    self.field.setFrequency(round(frequency,3))
                    fieldX = 8*cos(pairHeadingAngle)
                    fieldY = 8*sin(pairHeadingAngle)
                elif frequency > 60:
                    frequency = 60

            else:
                if self.state == 2:
                    if desiredSeparation >= 160:
                        frequency = 0.15*(desiredSeparation) - 48 + (sepDifference-5)*3
                    elif 120 <= desiredSeparation < 160:
                        frequency = 0.15*desiredSeparation - 48 + (sepDifference-1)*3
                    elif 90 <= desiredSeparation <120:
                        frequency = 0.3*desiredSeparation - 66 + (sepDifference)*2
                    else:
                        frequency = 0.3*desiredSeparation - 66 + (sepDifference)*1

                    # use uniform field when agents are far
                    if frequency > 0:
                        frequency = 0
                        self.field.setFrequency(round(frequency,3))
                        fieldX = 8*cos(pairHeadingAngle)
                        fieldY = 8*sin(pairHeadingAngle)
                    elif frequency < -60:
                        frequency = -60
                else:
                    if desiredSeparation >= 160:
                        frequency = -0.15*(desiredSeparation) + 48 - (sepDifference-5)*3
                    elif 120 <= desiredSeparation < 160:
                        frequency = -0.15*desiredSeparation + 48 - (sepDifference-1)*3
                    elif 90 <= desiredSeparation <120:
                        frequency = -0.3*desiredSeparation + 66 - (sepDifference)*2
                    else:
                        frequency = -0.3*desiredSeparation + 66 - (sepDifference)*1

                    if frequency < 0:
                        frequency = 0
                        self.field.setFrequency(round(frequency,3))
                        fieldX = 8*cos(pairHeadingAngle)
                        fieldY = 8*sin(pairHeadingAngle)
                    elif frequency > 60:
                        frequency = 60

            self.field.setFrequency(round(frequency,3))
            self.field.setMagnitude(self.params[1])
            if frequency != 0:
                theta = 2 * pi * frequency * t
                fieldX = self.params[1] * cos(theta)
                fieldY = self.params[1] * sin(theta)
                #print(frequency)


            # if self.prex0 != None and self.prey0 != None and self.prex1 != None and self.prey1 != None:
            #     if abs(self.prex0-x0)>0 or abs(self.prey0-y0)>0 or abs(self.prex1-x1)>0 or abs(self.prey1-y1)>0:
            #         sys.stdout = open('fb2_data_Jan27.txt', 'a')
            #         print(x0,',',y0,',',x1,',',y1,',',round(realSeparation,3),',',round(desiredSeparation,1),',',round(degrees(pairHeadingAngle),3),',',round(self.params[3],1),',',round(t,3),',',round(frequency,3))


            self.storeLastFrame(x0,y0,x1,y1)

            # update time
            if self.counter == 1000:
                self.vision.setTime(round(t,3))
                self.counter = 0
            self.counter += 1


            self.field.setXGradient(fieldX,gradientX)
            self.field.setYGradient(fieldY,gradientY)
            self.field.setZ(0)

            if self.stopped:
                self.vision.setTime(0)
                self.counter = 0
                self.state = None
                return

    def storeLastFrame(self,x0,y0,x1,y1):
        self.prex0 = x0
        self.prey0 = y0
        self.prex1 = x1
        self.prey1 = y1

    def procedure(self):
        startTime = time.time()
        while True:
            t = time.time() - startTime # elapsed time (sec)

            #get coordinates of two agents
            x0 = self.vision.agent[0].x
            y0 = self.vision.agent[0].y
            x1 = self.vision.agent[1].x
            y1 = self.vision.agent[1].y

            if t <= 20:
                desiredCOM = self.vision.getDesiredCOM()
                desiredSeparation = 120
                desiredOrientation = 0
            elif t > 20 and t <= 40:
                desiredSeparation = 120
                desiredOrientation = radians(150)
            elif t > 40 and t <= 60:
                desiredSeparation = 140
                desiredOrientation = radians(90)
            elif t > 60 and t <= 80:
                desiredSeparation = 120
                desiredOrientation = 0
            elif t > 80 and t <= 90:
                desiredSeparation = 150
                desiredOrientation = 0
            elif t > 90 and t <= 130:
                desiredSeparation = 150+40*sin(pi*(round(t,1)-90)/10)
                desiredOrientation = 0
            elif t > 130 and t <= 150:
                desiredSeparation = 150
                desiredOrientation = 0
            elif t > 150 and t <= 160:
                desiredSeparation = 120
                desiredOrientation = radians(90)
            elif t > 160 and t <= 200:
                desiredSeparation = 120
                desiredOrientation = radians(90)+radians(60)*sin(pi*(t-160)/20)
            elif t > 200 and t <= 210:
                desiredSeparation = 120
                desiredOrientation = radians(90)
            elif t > 210 and t <= 220:
                desiredSeparation = 120
                desiredOrientation = radians(90)-radians((t-210)*9)
            elif t > 220 and t <= 225:
                desiredSeparation = 120
                desiredOrientation = 0
            elif t > 225 and t <= 240:
                desiredSeparation = 180
                desiredOrientation = 0
            elif t > 240 and t <= 250:
                desiredSeparation = 90
                desiredOrientation = 0
            elif t > 250 and t <= 260:
                desiredSeparation = 120
                desiredOrientation = 0
            elif t > 260 and t <= 270:
                self.vision.clearMouse()
                desiredCOM = [430,300]
                self.vision.setDesiredCOM(desiredCOM[0],desiredCOM[1])
                desiredSeparation = 120
                desiredOrientation = 0
            elif t > 270:
                desiredCOM = [300,240]
                self.vision.setDesiredCOM(desiredCOM[0],desiredCOM[1])
                desiredSeparation = 120
                desiredOrientation = 0


            # set up gradient field to achieve pulling
            realCOM = [int((x0+x1)/2),int((y0+y1)/2)]
            vector = [desiredCOM[0]-realCOM[0],desiredCOM[1]-realCOM[1]]
            gradientX = 0.05*vector[0]
            if gradientX > 2.5:
                gradientX = 2.5
            elif gradientX < -2.5:
                gradientX = -2.5

            gradientY = -0.05*vector[1]
            if gradientY > 2.5:
                gradientY = 2.5
            elif gradientY < -2.5:
                gradientY = -2.5


            # control the pair heading angle and separation
            pairHeadingAngle = atan2(-(y1-y0),x1-x0)
            self.vision.setOrientation(desiredOrientation)
            oriDifference = pairHeadingAngle - desiredOrientation

            realSeparation = sqrt(pow(x1-x0,2)+pow(y1-y0,2))
            self.vision.setSeparation(desiredSeparation)
            sepDifference = realSeparation - desiredSeparation

            if desiredSeparation <= 110:
                threshold = 4
            elif desiredSeparation > 110 and desiredSeparation <= 150:
                threshold = 3
            else:
                threshold = 1

            if oriDifference >= radians(threshold):
                self.state = 1
                if desiredSeparation >= 160:
                    frequency = 0.15*(desiredSeparation) - 48 + (sepDifference-5)*3
                elif 120 <= desiredSeparation < 160:
                    frequency = 0.15*desiredSeparation - 48 + (sepDifference-1)*3
                elif 90 <= desiredSeparation < 120:
                    frequency = 0.3*desiredSeparation - 66 + (sepDifference)*2
                else:
                    frequency = 0.3*desiredSeparation - 66 + (sepDifference)*1

                # use uniform field when agents are far
                if frequency > 0:
                    frequency = 0
                    self.field.setFrequency(round(frequency,3))
                    fieldX = 8*cos(pairHeadingAngle)
                    fieldY = 8*sin(pairHeadingAngle)
                elif frequency < -60:
                    frequency = -60

            elif oriDifference < radians(-threshold):
                self.state = 2
                if desiredSeparation >= 160:
                    frequency = -0.15*(desiredSeparation) + 48 - (sepDifference-5)*3
                elif 120 <= desiredSeparation < 160:
                    frequency = -0.15*desiredSeparation + 48 - (sepDifference-1)*3
                elif 90 <= desiredSeparation <120:
                    frequency = -0.3*desiredSeparation + 66 - (sepDifference)*2
                else:
                    frequency = -0.3*desiredSeparation + 66 - (sepDifference)*1

                if frequency < 0:
                    frequency = 0
                    self.field.setFrequency(round(frequency,3))
                    fieldX = 8*cos(pairHeadingAngle)
                    fieldY = 8*sin(pairHeadingAngle)
                elif frequency > 60:
                    frequency = 60

            else:
                if self.state == 2:
                    if desiredSeparation >= 160:
                        frequency = 0.15*(desiredSeparation) - 48 + (sepDifference-5)*3
                    elif 120 <= desiredSeparation < 160:
                        frequency = 0.15*desiredSeparation - 48 + (sepDifference-1)*3
                    elif 90 <= desiredSeparation <120:
                        frequency = 0.3*desiredSeparation - 66 + (sepDifference)*2
                    else:
                        frequency = 0.3*desiredSeparation - 66 + (sepDifference)*1

                    # use uniform field when agents are far
                    if frequency > 0:
                        frequency = 0
                        self.field.setFrequency(round(frequency,3))
                        fieldX = 8*cos(pairHeadingAngle)
                        fieldY = 8*sin(pairHeadingAngle)
                    elif frequency < -60:
                        frequency = -60
                else:
                    if desiredSeparation >= 160:
                        frequency = -0.15*(desiredSeparation) + 48 - (sepDifference-5)*3
                    elif 120 <= desiredSeparation < 160:
                        frequency = -0.15*desiredSeparation + 48 - (sepDifference-1)*3
                    elif 90 <= desiredSeparation <120:
                        frequency = -0.3*desiredSeparation + 66 - (sepDifference)*2
                    else:
                        frequency = -0.3*desiredSeparation + 66 - (sepDifference)*1

                    if frequency < 0:
                        frequency = 0
                        self.field.setFrequency(round(frequency,3))
                        fieldX = 8*cos(pairHeadingAngle)
                        fieldY = 8*sin(pairHeadingAngle)
                    elif frequency > 60:
                        frequency = 60

            self.field.setFrequency(round(frequency,3))
            self.field.setMagnitude(8)
            if frequency != 0:
                theta = 2 * pi * frequency * t
                fieldX = 8 * cos(theta)
                fieldY = 8 * sin(theta)
                #print(frequency)


            if self.prex0 != None and self.prey0 != None and self.prex1 != None and self.prey1 != None:
                if abs(self.prex0-x0)>0 or abs(self.prey0-y0)>0 or abs(self.prex1-x1)>0 or abs(self.prey1-y1)>0:
                    sys.stdout = open('final2_data_Feb12.txt', 'a')
                    print(x0,',',y0,',',x1,',',y1,',',desiredCOM[0],',',desiredCOM[1],',',round(realSeparation,3),',',round(desiredSeparation,1),',',round(degrees(pairHeadingAngle),3),',',round(degrees(desiredOrientation),1),',',round(t,3),',',round(frequency,3))


            self.storeLastFrame(x0,y0,x1,y1)

            # update time
            if self.counter == 1000:
                self.vision.setTime(round(t,3))
                self.counter = 0
            self.counter += 1


            self.field.setXGradient(fieldX,gradientX)
            self.field.setYGradient(fieldY,gradientY)
            self.field.setZ(0)

            if self.stopped:
                self.vision.setTime(0)
                self.vision.setDesiredCOM(None,None)
                self.counter = 0
                self.state = None
                return
