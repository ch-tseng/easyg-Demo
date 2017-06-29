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

def id2jpg(vData):
    if(vData==1):
        return "right.jpg"
    elif(vData==4):
        return "left.jpg"
    elif(vData==2):
        return "down.jpg"
    elif(vData==8):
        return "up.jpg"
    elif(vData==48):
        return "question.jpg"

def singleG(vData):
    fileJPG = id2jpg(vData)
    lcd.displayImgfile(fileJPG)

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
    jpg1 = cv2.imread("s" + id2jpg(vData1))
    jpg2 =  cv2.imread("s" + id2jpg(vData2))

    lcd.displayImg(np.hstack( (jpg1, jpg2)) )

def threeG(vData1, vData2, vData3):
    jpg1 =  cv2.imread("ss" + id2jpg(vData1))
    jpg2 =  cv2.imread("ss" + id2jpg(vData2))
    jpg3 =  cv2.imread("ss" + id2jpg(vData3))

    lcd.displayImg(np.hstack( (jpg1, jpg2, jpg3) ))


lcd.displayImgfile("easyg.jpg")
lastAccess = time.time()

while True:
    #for line in Serial.read():
    if (Serial.inWaiting()>0): 
        line = Serial.read()
        vData1 = int(ByteToHex(line),16)
        print(vData1)

        if(vData1==192):
            distanceG(vData1)

        else:
            time.sleep(waitNextTimer)  

            if (Serial.inWaiting()>0):
                time.sleep(waitNextTimer)
                line = Serial.read()
                vData2 = int(ByteToHex(line),16)

                if (Serial.inWaiting()>0):
                    line = Serial.read()
                    vData3 = int(ByteToHex(line),16)
                    threeG(vData1, vData2, vData3)

                else:
                    twiceG(vData1, vData2)

            else:
                singleG(vData1)

        lastAccess = time.time()
        bgEasyG = False

    else:
        if( bgEasyG==False and (time.time()-lastAccess > 10) ):
            lcd.displayImgfile("easyg.jpg")
            bgEasyG = True
