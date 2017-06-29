import time, sys
import serial
import cv2
#import imutils
import numpy as np
from libraryCH.device.lcd import ILI9341

distMax = 63
distMin = 19
# if waitNextTimer=0 , delay for two-gestures will be disabled.
waitNextTimer = 1.0

ratio = (240/distMax) * 1.0
bgEasyG = True

Serial = serial.Serial("/dev/ttyS0", 9600, timeout= 0.5 )

lcd = ILI9341(LCD_size_w=240, LCD_size_h=320, LCD_Rotate=270)
#zeros = np.zeros( (320,270), dtype = "uint8")
#imgHold = cv2.imread("hand.jpg")
#imgBG = cv2.merge([zeros, zeros, zeros])

def ByteToHex(byteStr ):
    rtnValue = ''.join( [ "%02X " % ord( x ) for x in byteStr ] ).strip()
    return rtnValue

#def resizeIMG(image, rSize):
#    return imutils.resize(image, width = rSize) 

def displayHold(vData):
    global distMax, distMin, ratio
    #ratio = (240/distMax) * 1.0
    resizeR = int( vData * ratio)
    if(resizeR>240): resizeR=240

    lcd.drawRectangle(resizeR)

def singleG(vData):
    if(vData==1):
        lcd.displayImgfile("right.jpg")
        #print("Right")
    elif(vData==4):
        lcd.displayImgfile("left.jpg")
        #print("Left")
    elif(vData==2):
        lcd.displayImgfile("down.jpg")
        #print("Down")
    elif(vData==8):
        lcd.displayImgfile("up.jpg")
        #print("Up")
    elif(vData==48):
        lcd.displayImgfile("question.jpg")
        #print("??")

def distanceG(vData):
    global distMax, distMin
    print(vData)

    lastGesture = 0
    while ((vData<=distMax and vData>=distMin) or vData==192):
        if(vData<=distMax and vData>=distMin):
            displayHold(vData)

        if(Serial.in_waiting>0):
            line = Serial.read()
            vData = int(ByteToHex(line),16)
            print(vData)

        if(vData==48 and lastGesture==48):
            break

        lastGesture = vData

def twiceG(vData1, vData2):
    if(vData1==1):
        img1 = cv2.imread("sright.jpg")
    elif(vData1==4):
        img1 = cv2.imread("sleft.jpg")
    elif(vData1==2):
        img1 = cv2.imread("sdown.jpg")
    elif(vData1==8):
        img1 = cv2.imread("sup.jpg")
    elif(vData1==48):
        img1 = cv2.imread("squestion.jpg")

    if(vData2==1):
        img2 = cv2.imread("sright.jpg")
    elif(vData2==4):
        img2 = cv2.imread("sleft.jpg")
    elif(vData2==2):
        img2 = cv2.imread("sdown.jpg")
    elif(vData2==8):
        img2 = cv2.imread("sup.jpg")
    elif(vData2==48):
        img2 = cv2.imread("squestion.jpg")

    lcd.displayImg(np.hstack( (img1, img2)) )

lcd.displayImgfile("easyg.jpg")
lastAccess = time.time()

while True:
    #for line in Serial.read():
    if (Serial.inWaiting()>0): 
        line = Serial.read()
        vData = int(ByteToHex(line),16)
        print(vData)

        if(vData==192):
            distanceG(vData)

        else:
            time.sleep(waitNextTimer)  
            if (Serial.inWaiting()>0):
                line = Serial.read()
                vData2 = int(ByteToHex(line),16)
                twiceG(vData, vData2)

            else:
                singleG(vData)

        lastAccess = time.time()
        bgEasyG = False

    else:
        if( bgEasyG==False and (time.time()-lastAccess > 10) ):
            lcd.displayImgfile("easyg.jpg")
            bgEasyG = True
