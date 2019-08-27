# Used for drawing plots on the frames for general purposes
# but not related to object detection

import cv2

#=============================================================================================
# Call this function if selected drawingName is not defined
#=============================================================================================
def drawingNotDefined(img,args):
    print('Drawing not defined in drawing.py')
    return img

#=============================================================================================
# Pass a list as args
#=============================================================================================
def circle(img,args):
    x, y, r,index = args
    if index in (4,7):
        cv2.circle(img, (x,y), r, (0,0,255), 2)
    elif index == 3:
        cv2.circle(img, (x,y), r, (255,0,0), 2)
    elif index in (1,2,5,6) :
        cv2.circle(img, (x,y), r, (0,0,0), 2)
    return img

def arrow(img,args):
    x1,y1,x2,y2,index = args
    if index == 1:
        cv2.arrowedLine(img, (x1,y1), (x2,y2), (0,0,0), 3,tipLength=0.2)
    else:
        cv2.arrowedLine(img, (x1,y1), (x2,y2), (0,0,0), 3,tipLength=10)
    return img

def ellipse(img,args):
    x,y,r1,r2 = args
    cv2.ellipse(img,(x,y),(r1,r2),0,0,360,(0,255,0),2)
    return img

def line(img,args):
    x1,y1,x2,y2 = args
    cv2.line(img, (x1,y1), (x2,y2), (0,255,0), 2)
    return img

def text(img,args):
    text,index = args
    if index in (1,2,3):
        cv2.putText(img,text,(40+220*(index-1),40),0,0.6,(0,0,0),2)
    elif index in (4,5):
        cv2.putText(img,text,(40+220*(index-4),60),0,0.6,(0,0,0),2)
    return img

def number(img,args):
    text,x,y = args
    cv2.putText(img,text,(x+20,y-20),0,1,(0,0,0),2)
    return img

def pathUT(img,args):
    pathID, offsetX, offsetY, scale, _ = args
    if pathID == 0:
        points = [(10,10),(10,400),(150,400),(150,10)]
        points = [(int(point[0]+offsetX),int(point[1]+offsetY)) for point in points]
        for i in range(len(points)-1):
            cv2.line(img, points[i], points[i+1], (255,0,0), 2)
    elif pathID == 1:
        points = [(180,10),(250,10),(250,400),(250,10),(320,10)]
        points = [(int(point[0]+offsetX),int(point[1]+offsetY)) for point in points]
        for i in range(len(points)-1):
            cv2.line(img, points[i], points[i+1], (255,0,0), 2)
    return img

def closedPath(img,args):
    pointsX = args[0] # [x1,x2,....]
    pointsY = args[1] # [y1,y2,....]
    for i in range(len(pointsX)-1):
        cv2.line(img, (pointsX[i],pointsY[i]), (pointsX[i+1],pointsY[i+1]), (0,255,0), 2)
    return img
