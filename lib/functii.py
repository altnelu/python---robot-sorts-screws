import cv2
from imutils import perspective
from imutils import contours
from PIL import ImageTk, Image
from scipy.spatial import distance as dist

import imutils
import numpy as np
import math
import serial
import time
import math
from functools import wraps, reduce

width, height = 600, 600
cap = cv2.VideoCapture(0)
serialPort=serial
serialOK=False
nr_linie=1
scara=100
xx=183
yy=96
zz=0
nr_linie=1
corectie=1.06
corecties=1.2


try:
    serialPort = serial.Serial(port = "COM5", baudrate=115200,
                           bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)

  #while ser.read():
    if serialPort.read():
        serialOK=True
        #meniu_lbl2.configure(text="Robot OK")
  #ser.close()

except serialPort.serialutil.SerialException:
    #meniu_lbl2.configure(text="Robot Deconectat")
    serialOK=False


    

#cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
# serial
def _checksum( command):
        return reduce(lambda x, y: x ^ y, map(ord, command))
    
def sterbuffer():
    time.sleep(.5)
    while(serialPort.in_waiting > 0):
        serialString = serialPort.readline()
        print("RESTUL {0}".format(serialString))
    #serialPort.flush()
    #serialPort.flushInput()
    #serialPort.flushOutput()    
def f_robot_ok():
        print(serialOK)
        if(serialOK==False):
                return False
        #0 ok   #1 in miscare
        serialString = "" 
        if(serialPort.in_waiting > 0):
                time.sleep(.1)
                serialString = serialPort.readline()
                print(serialString.decode('Ascii'))
                if serialString.decode('Ascii')=='echo:GATA\r\n' or serialString.decode('Ascii')=='wait\n':
                    return True
        return False

def verificcamera():
    if cap.isOpened():
        #camera ok
        #meniu_lbl.configure(text="Camera OK")
        return True
    else:
        #meniu_lbl.configure(text="Lipsa camera")
        return False
def captura():
    _, frame = cap.read()
    return frame
def imgcvtowin(edged,win):
    edged = cv2.cvtColor(edged, cv2.COLOR_BGR2RGB)
    img2 = Image.fromarray(edged)
    imgtk2 = ImageTk.PhotoImage(image=img2)
    win.imgtk = imgtk2
    win.configure(image=imgtk2) 
    
def midpoint(ptA, ptB):
	return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)
    

def get_angle(p1, p2):
    return math.atan2(p1[1] - p2[1], p1[0] - p2[0]) * 180/math.pi

def axe(box):
    (tl, tr, br, bl) = box
    (tltrX, tltrY) = midpoint(tl, tr)
    (blbrX, blbrY) = midpoint(bl, br)
    (tlblX, tlblY) = midpoint(tl, bl)
    (trbrX, trbrY) = midpoint(tr, br)
    unghi=get_angle(tl, br)
    dA = dist.euclidean((tltrX, tltrY), (blbrX, blbrY))
    dB = dist.euclidean((tlblX, tlblY), (trbrX, trbrY))           
    return(min(dA,dB),max(dA,dB),unghi)
def boxa(c):
    boxnr= cv2.minAreaRect(c)
    boxnr = cv2.cv.BoxPoints(boxnr) if imutils.is_cv2() else cv2.boxPoints(boxnr)
    boxnr = np.array(boxnr, dtype="int")
    boxnr = perspective.order_points(boxnr)
    return boxnr

def sortSecond(val): 
    return val[1]
def caut_obiecte(cv2image):
    #gray = cv2.cvtColor(cv2image, cv2.COLOR_BGR2GRAY)
    gray = cv2.cvtColor(cv2image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (7, 7), 0)
    #edged = cv2.Canny(gray, 50, 200)
    edged = cv2.Canny(gray, 50, 200)
    #ingroasa
    edged = cv2.dilate(edged, None, iterations=1)
    edged = cv2.erode(edged, None, iterations=1)
    cv2.imshow("Imagean", edged)
    cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    #cnts = cv2.findContours(edged.copy(), cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    return cnts, edged

def caut_obiecte2(cv2image):
    #gray = cv2.cvtColor(cv2image, cv2.COLOR_BGR2GRAY)
    gray = cv2.cvtColor(cv2image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (7, 7), 0)
    #edged = cv2.Canny(gray, 50, 200)
    edged = cv2.Canny(gray, 50, 200)
    #ingroasa
    edged = cv2.dilate(edged, None, iterations=1)
    edged = cv2.erode(edged, None, iterations=1)
    cnts = cv2.findContours(edged.copy(), cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    #cnts = cv2.findContours(edged.copy(), cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    
    return cnts, edged

def cauta(frame,scara,xx,yy,zz,corectie,corecties):
    dx,dy,unghi,tip,dA,dB = 0,0,0,0,0,0
    cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
    cnts, edged=caut_obiecte(cv2image)
    if len(cnts)==0:
        #mesaje.configure(text="0 obiecte")
        print("0 obiecte")
        return 0,0,0,0,0,0
    (cnts, _) = contours.sort_contours(cnts)
    obiecte =[]
    i=0
    #filtrez si sortez 
    for c in cnts:
        x=0
        y=0
        boxnr=boxa(c)
        aria=cv2.contourArea(boxnr)
        cv2.drawContours(frame, [boxnr.astype("int")], -1, (255, 0, 0), 2)
#        if aria>1930 and aria<1945 or aria>8400:
        if aria>100 and aria<8500:
            M = cv2.moments(boxnr)
            if M["m00"]==0:
                continue
            x = int(M["m10"] / M["m00"])
            y = int(M["m01"] / M["m00"])
            obiecte.append((i,aria,boxnr,x,y))       
            i=i+1
    obiecte.sort(key=sortSecond)
    if len(obiecte)<2:
        #mesaje.configure(text="Nu sunt piese")
        return 0,0,0,0,0,0
    # caut originile DE FACUT
    box =[(box, aria,x,y) for i,aria,box,x,y in obiecte if i==0]
    (box,aria,x0,y0)=box[0]
    print("baza")
    print (aria,x0,y0)
    if y0<240 or x0>70 or aria<100 or aria >20000:
        return 0,0,0,0,0,0   
    cv2.drawContours(frame, [box.astype("int")], -1, (255, 255, 0), 2)
    cv2.circle(frame, (int(x0), int(y0)), 5, (0, 0, 255), -1)
    (dA,dB,unghi)=axe(box)
    pixelsPerMetric = max(dA,dB) / scara
    cv2.putText(frame,'x,y:{0!r} , {1!r}'.format(round(max(dA,dB)/pixelsPerMetric,1),round(min(dA,dB)/pixelsPerMetric,1))
                +'' ,(int(x0 - 20), int(y0)-40), cv2.FONT_HERSHEY_SIMPLEX,0.465, (255, 255, 255), 1)
    cv2.putText(frame,'X,Y:{0!r} , {1!r}'.format(round(xx,1),round(yy,1))
                +'' ,(int(x0 - 20), int(y0)-20), cv2.FONT_HERSHEY_SIMPLEX,0.465, (255, 255, 255), 1)


    #cv2.imshow("Image", orig)
    #prima piesa
    nr_coloana=0
    #0 piulita, 1 surub
    tip=0

    if len(obiecte)>0:
        box =obiecte[0]
#    for box in obiecte:
        (nr,aria,box,x,y)=box
        #print (aria,x,y)
        cv2.drawContours(frame, [box.astype("int")], -1, (255, 255, 0), 2)
        cv2.circle(frame, (int(x), int(y)), 5, (0, 0, 255), -1)
        (dA,dB,unghi)=axe(box)
        #Calculez COORDONATE
        dx=xx-((y-y0)/pixelsPerMetric)
        dy=yy-((x-x0)/pixelsPerMetric)
        cv2.putText(frame,'X={0!r}'.format(int(dx))+ ' Y= {0!r} '.format(int(dy))+ " aria={0!r} ".format(aria)+  " u={0!r}".format(int(unghi)) ,(int(x + 10), int(y)+5), cv2.FONT_HERSHEY_SIMPLEX,0.465, (0, 0, 255), 1)
        dA=dA/pixelsPerMetric
        dB=dB/pixelsPerMetric
        if dA<dB:
                temp=dA
                dA=dB
                dB=temp
        print (dA/dB)
        if  dA/dB>1.4 :
                tip=1
                raport=15
                xmic=x-raport
                ymic=y-raport
                w=x+raport
                h=y+raport        
                if xmic<0:xmic=0
                if ymic<0:ymic=0
                if h<=ymic: return 0,0,0,0,0,0
                if w<=xmic: return 0,0,0,0,0,0
                
                im2=cv2image[ymic:h, xmic:w]
                im2=cv2.rectangle(im2, (0,0), (raport*2-1,raport*2-1), (127,127,127), 1)
                cv2.imshow("ImageMica", im2)
                #cnts2, edged2=caut_obiecte(im2)
                cnts2, edged2=caut_obiecte2(im2)
                maxim=len(cnts2)
                print(maxim)
                metric=""
                #if maxim>2:
                for b in cnts2:
                        #box2=boxa(cnts2[1])
                        
                        box2=boxa(b)
                        (dA2,dB2,unghi2)=axe(box2)
                        cv2.drawContours(im2, [box2.astype("int")], -1, (255, 255, 0), 2)
                        cv2.imshow("ImageMica2", im2)
                        if dA2/dB2>0.7 or dA2*dB2==0:
                            continue
                        #print("-----",dA2,dB2,dA2/dB2)
                        #cv2.imshow("ImageMica", im2)
                        (dA2,dB2,unghi2)=axe(box2)
                        dB2=round(min(dA2,dB2)/pixelsPerMetric,1)
                        print(dB2)
                        if dB2>2 and dB2<7:
                                dB=dB2
                                break
                        
                        
    return dx,dy,abs(unghi),tip,dA,dB
def trimit_gcode(txt_gcode):
    global nr_linie
    prefix = "N" + str(nr_linie) + " " + txt_gcode
    command = prefix + "*" + str(_checksum(prefix))
    print(command)
    #serialPort.write(btxt_gcode)
    command="{0}\r\n".format(command)
    #res = ''.join(format(i, 'b') for i in bytearray(txt_gcode, encoding ='utf-8'))
    #print(txt_gcode)
    if(serialOK==False):
            return
    serialPort.write(command.encode('ascii'))
    while(1):
        if(serialPort.in_waiting > 0):
            serialString = serialPort.readline()
            #print(serialString.decode('Ascii'))
            if "OK" in serialString.decode('Ascii').upper()  :
                nr_linie=nr_linie+1
                return
            else:
                if 'Error:checksum mismatch' in serialString.decode('Ascii'):
                    trimit_gcode(txt_gcode)
            #if serialString.decode('Ascii')=='wait\r\n' or serialString.decode('Ascii')=='wait\n' :
            #    trimit_gcode(txt_gcode)
                #return
   
def decizii(dx,dy,unghi,tip,dA,dB):
        if dx<100 or dx>280 or dy<-150 or dy >75 :
                return
        trimit_gcode("G1 Z50 F90000")
        trimit_gcode("M280 P0 S0")
        trimit_gcode("G1 X"+'{0!r}'.format(dx)+" Y"'{0!r}'.format(dy))
        if abs(unghi)>45 and abs(unghi)>135 and (dA/dB>1.399) :
                #rotesc
                trimit_gcode("G1 X"+'{0!r}'.format(dx-20)+" Y"'{0!r}'.format(dy-max(dA,dB)/3))
                trimit_gcode("M280 P0 S80")
                trimit_gcode("G4 P100")
                trimit_gcode("G1 Z{0!r}".format(zz))
                trimit_gcode("G1 X"+'{0!r}'.format(dx+20)+" Y"'{0!r}'.format(dy-max(dA,dB)/3))
                trimit_gcode("G1 Z50")
                trimit_gcode("M280 P0 S0")
                trimit_gcode("G4 P100")
                trimit_gcode("G1 X150 Y-150")
                trimit_gcode("G4 P100")
                return
        trimit_gcode("G1 Z{0!r}".format(zz))
        trimit_gcode("G4 P100")
        metric=""
        if dA/dB<1.4:
                print("piulita")
                print("{0} corectie saiba ={1}".format(((dA+dB)/2),7.95/((dA+dB)/2)))
                if ((dA+dB)/2)*corectie<7.5:
                        metric="3mm"
                        trimit_gcode("M280 P0 S90")
                        trimit_gcode("G4 P100")
                        trimit_gcode("G1 Z50")
                        trimit_gcode("G1 X150 Y-150")
                        trimit_gcode("G1 X146 Y-160")
                if ((dA+dB)/2)*corectie>=7.5 and ((dA+dB)/2)*corectie<8.4 :
                        metric="4mm"
                        trimit_gcode("M280 P0 S80")
                        trimit_gcode("G4 P100")
                        trimit_gcode("G1 Z50")
                        trimit_gcode("G1 X150 Y-150")
                        trimit_gcode("G1 X81 Y-160")
                if ((dA+dB)/2)*corectie>=8.4*corectie :
                        metric="5mm"
                        trimit_gcode("M280 P0 S70")
                        trimit_gcode("G4 P100")
                        trimit_gcode("G1 Z50")
                        trimit_gcode("G1 X150 Y-150")
                        trimit_gcode("G1 X16 Y-160")  
                print("piulita "+metric)               
        else:
                print("{0} corectie surub ={1}".format(dB, 4/dB))
                grosime=dB
                grosime=round(grosime*corecties,1)
                print("surub")
                #if grosime<2 or grosime>7:
                #    return
                if grosime<3.5:
                    metric="M3"
                if grosime>=3.5 and grosime<=4.5:
                    metric="M4"
                if grosime>4.5 :
                    metric="M5"   


                if metric=="M3":
                        trimit_gcode("M280 P0 S120")
                        trimit_gcode("G4 P100")
                        trimit_gcode("G1 Z50")
                        trimit_gcode("G1 X150 Y-150")
                        trimit_gcode("G1 X178,75 Y-160")
                if metric=="M4":
                        trimit_gcode("M280 P0 S110")
                        trimit_gcode("G4 P100")
                        trimit_gcode("G1 Z50")
                        trimit_gcode("G1 X150 Y-150")
                        trimit_gcode("G1 X113 Y-160")
                if metric=="M5" :
                        trimit_gcode("M280 P0 S105")
                        trimit_gcode("G4 P100")
                        trimit_gcode("G1 Z50")
                        trimit_gcode("G1 X150 Y-150")
                        trimit_gcode("G1 X48 Y-160")             
                print("surub "+metric)
        trimit_gcode("G4 P100")
        trimit_gcode("M280 P0 S0")
        trimit_gcode("G4 P100")
        trimit_gcode("M118 E1 GATA")
        return

def acasa():
    print("HOME")
    trimit_gcode("M84 Z")
    trimit_gcode("G28")
    trimit_gcode("G1 Y-80 Z190")
    trimit_gcode("M84 Z")
    trimit_gcode("G28")
    trimit_gcode("G1 X100 Y-100 F10000")
    trimit_gcode("G1 Z50")
    trimit_gcode("M118 E1 GATA")
    #mesaje echo trimit_gcode("M111 S1")
    #trimit_gcode("G1 Z30")
    
