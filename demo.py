import time, sys
import serial
#import cv2
#import imutils
#import numpy as np
from libraryCH.device.lcd import ILI9341

distMax = 63
distMin = 19
ratio = (240/distMax) * 1.0

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

    #lcd.displayClear()
    print ("vData:{}, Ratio:{}, Resize:{}".format(vData,ratio,resizeR))
    #imgResize = imutils.resize(imgHold, width = resizeR)
    #lcd.displayImg( imgResize )
    lcd.drawRectangle(resizeR)
    #time.sleep(0.06)
    #Serial.flushInput()

lcd.displayImgfile("easyg.jpg")

while True:
    for line in Serial.read():
        vData = int(ByteToHex(line),16)
        print(vData)

        if(vData==1):
            lcd.displayImgfile("right.jpg")
            print("Right")

        if(vData==4):
            lcd.displayImgfile("left.jpg")
            print("Left")

        if(vData==2):
            lcd.displayImgfile("down.jpg")
            print("Down")

        if(vData==8):
            lcd.displayImgfile("up.jpg")
            print("Up")

        if(vData==48):
            lcd.displayImgfile("question.jpg")
            print("??")

        if(vData==192):
            #lcd.displayImgfile("hand.jpg")
            vData = int(ByteToHex(line),16)
            print(vData)

            lastGesture = 0
            while ((vData<=distMax and vData>=distMin) or vData==192):
                if(vData<=distMax and vData>=distMin):
                    displayHold(vData)

                if(Serial.in_waiting>0):
                    line = Serial.read()
                    #print("Serial read: {} -->HEX: {}".format(line, ByteToHex(line)) )
                    vData = int(ByteToHex(line),16)
                    print("last:{}, this:{}".format(lastGesture, vData))
                #else:
                #    lcd.displayImgfile("black.jpg")
                #    vData = 0
               
                if(vData==48 and lastGesture==48):
                    lcd.displayImgfile("black.jpg")
                    break

                lastGesture = vData

