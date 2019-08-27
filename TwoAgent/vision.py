"""
=============================================================================
vision.py
----------------------------------------------------------------------------
Version
1.1.0 2018/08/04 Added snapshot feature.
1.0.0 2018/06/16 Added video writing feature.
0.0.1 2018/02/05 Initial commit
----------------------------------------------------------------------------
[GitHub] : https://github.com/atelier-ritz
=============================================================================
"""

import cv2, sys, re, time
from pydc1394 import Camera
import filterlib
import drawing
import objectDetection
from objectDetection import Agent
from math import cos,sin,sqrt,pow,atan2,degrees
from numpy import sign
#
# #=============================================================================================
# # Mouse callback Functions
# #=============================================================================================
mouseX = 0
mouseY = 0

def showClickedCoordinate(event,x,y,flags,param):
    # global mouseX,mouseY
    if event == cv2.EVENT_LBUTTONDOWN:
        global mouseX
        global mouseY
        mouseX,mouseY = x,y
        print('Clicked position  x: {} y: {}'.format(x,y))
        return mouseX,mouseY

class Vision(object):
    def __init__(self,field,index,type,guid=0000000000000000,buffersize=10):
        self._field = field
        self._id = index
        self._type = type
        self._guid = guid
        self._isUpdating = True
        self._isFilterBypassed = True
        self._isObjectDetectionEnabled = False
        self._isSnapshotEnabled = False
        self._detectionAlgorithm = ''
        self.filterRouting = [] # data structure: {"filterName", "args"}, defined in the GUI text editor
        self.frameCounter = 0

        # instances of Agent class. You can define an array if you have multiple agents.
        # Pass them to *processObjectDetection()*
        self.agent = [Agent(),Agent()]

        # drawings
        self._isFieldDrawingEnabled = True   # draw field indicators
        self._isAgentDrawingEnabled = True  # draw separation vector and COM for agents
        self.drawingRouting = [] # data structure: {"drawingName", "args"}, defined in Subthread

        # video writing
        self._isVideoWritingEnabled = False
        self.videoWriter =  None

        # desired separation and desired orientation
        self.separation = None
        self.orientation = None
        self.psi_local = None
        self.time = 0
        self.desiredX = None
        self.desiredY = None

        if self.isFireWire():
            self.cam = Camera(guid=self._guid)
            print("====================================================")
            print("CameraId:", self._id)
            print("Model:", self.cam.model)
            print("GUID:", self.cam.guid)
            print("Mode:", self.cam.mode)
            print("Framerate: ", self.cam.rate)
            print("====================================================")
            self.cam.start_capture(bufsize=buffersize)
            self.cam.start_video()
        else:
            self.cap = cv2.VideoCapture('template1.mp4')
            if not self.cap.isOpened():
                print('Camera is not detected. End program.')
                self.cap.release()
                sys.exit()


        cv2.namedWindow(self.windowName(),16) # cv2.GUI_NORMAL = 16
        cv2.moveWindow(self.windowName(), 100,-320+340*self._id);
        cv2.setMouseCallback(self.windowName(),showClickedCoordinate)


    def updateFrame(self):
        if self.isFireWire():
            if self.isUpdating():
                frameOriginal = self.cam.dequeue()
                if not self.isFilterBypassed() and not self.filterRouting == []:
                    frameFiltered = self.processFilters(frameOriginal.copy())
                else:
                    frameFiltered = frameOriginal
                if self.isObjectDetectionEnabled():
                    frameProcessed = self.processObjectDetection(frameFiltered,frameOriginal)
                else:
                    frameProcessed = frameFiltered
                if self.isFieldDrawingEnabled():
                    self.fieldDrawing(self._field.x,self._field.y,self._field.freq,self._field.mag,self.time,self.separation,self.orientation)
                if self.isAgentDrawingEnabled():
                    self.agentDrawing(self.agent[0].x,self.agent[0].y,self.agent[1].x,self.agent[1].y)
                if self.isDrawingEnabled():
                    frameProcessed = self.processDrawings(frameProcessed)
                if self.isSnapshotEnabled():
                    cv2.imwrite('snapshot.png',filterlib.color(frameProcessed))
                    self.setStateSnapshotEnabled(False)
                if self.isVideoWritingEnabled():
                    self.videoWriter.write(filterlib.color(frameProcessed))

                cv2.imshow(self.windowName(),frameProcessed)
                frameOriginal.enqueue()
                self.clearDrawingRouting()
        else:
            if self.isUpdating():
                _,frameOriginal = self.cap.read()
                if not self.isFilterBypassed() and not self.filterRouting == []:
                    frameFiltered = self.processFilters(frameOriginal.copy())
                else:
                    frameFiltered = frameOriginal
                if self.isObjectDetectionEnabled():
                    frameProcessed = self.processObjectDetection(frameFiltered,frameOriginal)
                else:
                    frameProcessed = frameFiltered
                if self.isFieldDrawingEnabled():
                    self.fieldDrawing(self._field.x,self._field.y,self._field.freq,self._field.mag,self.time,self.separation,self.orientation)
                if self.isAgentDrawingEnabled():
                    self.agentDrawing(self.agent[0].x,self.agent[0].y,self.agent[1].x,self.agent[1].y)
                if self.isDrawingEnabled():
                    frameProcessed = self.processDrawings(frameProcessed)
                if self.isSnapshotEnabled():
                    cv2.imwrite('snapshot.png',filterlib.color(frameProcessed))
                    self.setStateSnapshotEnabled(False)
                if self.isVideoWritingEnabled():
                    self.videoWriter.write(filterlib.color(frameProcessed))

                self.frameCounter += 1
                #If the last frame is reached, reset the capture and the frame_counter
                if self.frameCounter == self.cap.get(cv2.CAP_PROP_FRAME_COUNT):
                    self.frameCounter = 0 #Or whatever as long as it is the same as next line
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                #print(self.frameCounter)

                cv2.imshow(self.windowName(),frameProcessed)
                self.clearDrawingRouting()



    def closeCamera(self):
        if not self.videoWriter == None:
            self.videoWriter.release()
            self.videoWriter = None
        if self.isFireWire():
            self.cam.stop_video()
        else:
            self.cap.release
        cv2.destroyWindow(self.windowName())

    #==============================================================================================
    # obtain instance attributes
    #==============================================================================================
    def windowName(self):
        return 'CamID:{} (Click to print coordinate)'.format(self._id)

    def isFireWire(self):
        return self._type.lower() == 'firewire'

    def isUpdating(self):
        return self._isUpdating

    def isFilterBypassed(self):
        return self._isFilterBypassed

    def isObjectDetectionEnabled(self):
        return self._isObjectDetectionEnabled

    def isDrawingEnabled(self):
        return not self.drawingRouting == []

    def isSnapshotEnabled(self):
        return self._isSnapshotEnabled

    def isVideoWritingEnabled(self):
        return self._isVideoWritingEnabled

    def isFieldDrawingEnabled(self):
        return self._isFieldDrawingEnabled

    def isAgentDrawingEnabled(self):
        return self._isAgentDrawingEnabled

    #==============================================================================================
    # set instance attributes
    #==============================================================================================
    def setStateUpdate(self,state):
        self._isUpdating = state

    def setStateFiltersBypassed(self,state):
        self._isFilterBypassed = state

    def setStateObjectDetection(self,state,algorithm):
        self._isObjectDetectionEnabled = state
        self._detectionAlgorithm = algorithm

    def setVideoWritingEnabled(self,state):
        self._isVideoWritingEnabled = state

    def setStateSnapshotEnabled(self,state):
        self._isSnapshotEnabled = state

    def setSeparation(self,val):
        self.separation = val

    def setOrientation(self,val):
        self.orientation = val

    def setLocalAngle(self,val):
        self.psi_local = val

    def setTime(self,val):
        self.time = val

    def setDesiredCOM(self,x,y):
        self.desiredX = x
        self.desiredY = y

    def clearMouse(self):
        global mouseX
        global mouseY
        mouseX = mouseY = 0

    #==============================================================================================
    # Video recording
    #==============================================================================================
    def createVideoWriter(self,fileName):
        self.videoWriter = cv2.VideoWriter(fileName,fourcc=cv2.VideoWriter_fourcc(*'XVID'),fps=30.0,frameSize=(640,480),isColor=True)

    def startRecording(self,fileName):
        self.createVideoWriter(fileName)
        self.setVideoWritingEnabled(True)
        print('Start recording' + fileName)

    def stopRecording(self):
        self.setStateSnapshotEnabled(False)
        self.videoWriter.release()
        print('Stop recording.')

    #==============================================================================================
    # <Filters>
    # Define the filters in filterlib.py
    #==============================================================================================
    def createFilterRouting(self,text):
        self.filterRouting = []
        for line in text:
            line = line.split('//')[0]  # strip after //
            line = line.strip()         # strip spaces at both ends
            match = re.match(r"(?P<function>[a-z0-9_]+)\((?P<args>.*)\)", line)
            if match:
                name = match.group('function')
                args = match.group('args')
                args = re.sub(r'\s+', '', args) # strip spaces in args
                self.filterRouting.append({'filterName': name, 'args': args})

    def processFilters(self,image):
        for item in self.filterRouting:
            image = getattr(filterlib,item['filterName'],filterlib.filterNotDefined)(image,item['args'])
        # You can add custom filters here if you don't want to use the editor
        return image

    #==============================================================================================
    # <object detection>
    # Object detection algorithm is executed after all the filters
    # It is assumed that "imageFiltered" is used for detection purpose only;
    # the boundary of the detected object will be drawn on "imageOriginal".
    # information of detected objects can be stored in an instance of "Agent" class.
    #==============================================================================================
    def processObjectDetection(self,imageFiltered,imageOriginal):
        # convert to rgb so that coloured lines can be drawn on top
        imageOriginal = filterlib.color(imageOriginal)

        # object detection algorithm starts here
        # In this function, information about the agent will be updated, and the original image with
        # the detected objects highlighted will be returned
        algorithm = getattr(objectDetection,self._detectionAlgorithm,objectDetection.algorithmNotDefined)
        imageProcessed = algorithm(imageFiltered,imageOriginal,self.agent) # pass instances of Agent class if you want to update its info
        return imageProcessed

    #==============================================================================================
    # <subthread drawing>
    # Used to draw lines etc. on a plot
    # For showing the path that the robot wants to follow
    #==============================================================================================
    def clearAgentDrawing(self,state):
        if not state:
            self.agent[0].x = self.agent[0].y = self.agent[1].x = self.agent[1].y = 0
            global mouseX
            global mouseY
            mouseX = mouseY = 0

    def clearDrawingRouting(self):
        self.drawingRouting = []

    def addDrawing(self,name,args=None):
        self.drawingRouting.append({'drawingName': name, 'args': args})

    def processDrawings(self,image):
        # convert to rgb so that coloured lines can be drawn on top
        image = filterlib.color(image)
        for item in self.drawingRouting:
            image = getattr(drawing,item['drawingName'],drawing.drawingNotDefined)(image,item['args'])
        return image

    def fieldDrawing(self,x,y,freq,mag,time,separation,orientation):
        #field_mag = sqrt(pow(x,2)+pow(y,2)+pow(z,2))
        magXY = sqrt(pow(x,2)+pow(y,2))
        angleXY = atan2(y,x)
        self.addDrawing('circle',(80,400,40,1))
        self.addDrawing('circle',(80,400,2,2))
        # self.addDrawing('circle',(200,400,50,1))
        # self.addDrawing('circle',(200,400,2,2))
        # if self.psi_local != None:
        #     self.addDrawing('arrow',(200,400,int(200+50*cos(self.psi_local)),int(400-50*sin(self.psi_local))))
        if x==0 and y==0:
           self.addDrawing('arrow',(80,400,80,400,1))
        else:
           self.addDrawing('arrow',(80,400,80+int(round(40*cos(angleXY))),400-int(round(40*sin(angleXY))),1))

        # put text showing the field frequency and magnitude
        self.addDrawing('text',('Frequency:{}Hz'.format(freq),1))
        self.addDrawing('text',('Magnitude:{}mT'.format(mag),2))
        self.addDrawing('text',('Time:{}s'.format(time),3))
        if separation != None:
            self.addDrawing('text',('Target r = {}R'.format(round(separation/10,1)),4))
        else:
            self.addDrawing('text',('Target r = None',4))
        if orientation != None:
            self.addDrawing('text',('Target angle = {}'.format(round(degrees(orientation),2)),5))
        else:
            self.addDrawing('text',('Target angle = None',5))

        # draw the direction of rotating field
        if sign(freq) > 0:
            self.addDrawing('arrow',(52,372,51,373,2))
        elif sign(freq) < 0:
            self.addDrawing('arrow',(108,372,109,373,2))

    def agentDrawing(self,x1,y1,x2,y2):
        # put text showing the number of agents
        if x1 != 0 and y1 != 0 and x2 != 0 and y2 != 0:
            self.addDrawing('number',('1',x1,y1))
            self.addDrawing('number',('2',x2,y2))

            # draw the desired circles
            if self.separation != None:
                # pixelOverDistance = 1/25
                pixelOverDistance = 1
                self.addDrawing('circle',(x1,y1,int(pixelOverDistance*(self.separation/2)),3))
                self.addDrawing('circle',(x2,y2,int(pixelOverDistance*(self.separation/2)),4))

            # draw the desired pair heading angle
            if self.orientation != None:
                self.addDrawing('arrow',(int((x1+x2)/2-30*cos(self.orientation)),int((y1+y2)/2+30*sin(self.orientation)),int((x1+x2)/2+30*cos(self.orientation)),int((y1+y2)/2-30*sin(self.orientation)),1))

            # draw the center of mass
            self.addDrawing('circle',(int((x1+x2)/2),int((y1+y2)/2),2,5))


            # draw the arrow between two agents
            self.addDrawing('line',(x1,y1,x2,y2))

            # draw the pulling vector
            if mouseX != 0 and mouseY != 0:
                self.addDrawing('circle',(mouseX,mouseY,2,6))
                self.addDrawing('arrow',(int((x1+x2)/2),int((y1+y2)/2),mouseX,mouseY,1))
            elif self.desiredX != None and self.desiredY != None:
                self.addDrawing('circle',(self.desiredX,self.desiredY,5,7))
                self.addDrawing('arrow',(int((x1+x2)/2),int((y1+y2)/2),self.desiredX,self.desiredY,1))

        # self.addDrawing('circle',(200,400,50))
        # self.addDrawing('circle',(200,400,2))
        # self.addDrawing('ellipse',(200,400,50,20))


    def getDesiredCOM(self):
        if mouseX != 0 and mouseY != 0:
            desX = mouseX
            desY = mouseY
        else:
            desX = int((self.agent[0].x+self.agent[1].x)/2)
            desY = int((self.agent[0].y+self.agent[1].y)/2)
        return desX,desY
