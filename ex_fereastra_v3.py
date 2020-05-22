#!/usr/bin/env python3

# Display UTC.
# started with https://docs.python.org/3.4/library/tkinter.html#module-tkinter
from lib.functii import *

try:
    import Tkinter as tk
except:
    import tkinter as tk
    from tkinter import filedialog as fd
import time
import math

root = tk.Tk()
frame=None
chk_state = tk.IntVar()
chk_state.set(0) #uncheck


dx,dy,unghi,tip,dA,dB = 0,0,0,0,0,0



def current_iso8601():
    """Get current date and time in ISO8601"""
    # https://en.wikipedia.org/wiki/ISO_8601
    # https://xkcd.com/1179/
    #return time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    now = time.time()
    mlsec = repr(now).split('.')[1][:3]
    
    #return time.strftime("%H:%M:%S.{}".format(mlsec))
    return time.strftime("%H:%M:%S")

 
 

class Application(tk.Frame):
    def __init__(self, master=None):
        self.nr_pas=0
        tk.Frame.__init__(self, master)
        self.grid(column=0, row=0)
        self.createWidgets()

    def createWidgets(self):
        self.navigare=tk.Frame(self,bg="white")
        self.navigare.grid(column=0, row=0)
        self.video1=tk.Frame(self,bg="white")
        self.video1.grid(column=2, row=0)

        self.lbl1 = tk.Label(self.navigare, text="Setari",font=("Arial Bold",10))
        self.lbl1.grid(column=0, row=0)

        self.lbl_robot = tk.Label(self.navigare, text="Robot",font=("Arial Bold",10))
        self.lbl_robot.grid(column=0, row=1)

        self.B_robot = tk.Button(self.navigare, text="Acasa", command=lambda: self.pasul(0))
        self.B_robot.grid(column=0, row=2)

        # nu merge self.B1 = tk.Button(self.navigare, text="B1", command=(self.pasul(2)))
        self.B1 = tk.Button(self.navigare, text="Imagine", command=lambda: self.pasul(1))
        self.B1.grid(column=0, row=3)

        self.B2 = tk.Button(self.navigare, text="Captura", command=lambda:  self.pasul(2))
        self.B2.grid(column=0, row=4)

        self.Check_live = tk.Checkbutton(self.navigare,text='Video', var=chk_state, command=lambda:  self.pasul(2))
        self.Check_live.grid(column=0, row=5)

        self.l_video1 = tk.Label(self.video1)
        self.l_video1.grid(column=0, row=0)
        

        self.now = tk.StringVar()
        self.time = tk.Label(self, font=('Helvetica', 24))
        self.time.grid(column=0, row=1)
        self.time["textvariable"] = self.now

        self.QUIT = tk.Button(self, text="QUIT", fg="red",
                                            command=root.destroy)
        self.QUIT.grid(column=0, row=2)

        # initial time display
        self.onUpdate()

    def pasul(self, nr):
        self.nr_pas=nr

    def onUpdate(self):
        # update displayed time
        self.now.set(current_iso8601())
        self.lbl1.configure(text=self.nr_pas)
        fpasi(self)
        if not (frame is None):
            cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
            orig = cv2image.copy()
            imgcvtowin(orig,self.l_video1)
        
        # schedule timer to call myself after 1 second
        self.after(1, self.onUpdate)

def fpasi(self):
    global frame
    global dx,dy,unghi,tip,dA,dB
    #print(self.nr_pas)
    #0 start
    #1 verific robot
    #2 verific camera
    #3 captez imagime
    #98 o captura
    #99 bucla
    if self.nr_pas==0:
        print(serialOK)
        if serialOK:
            self.lbl_robot.configure(text="Robot OK")
            while f_robot_ok():
                z=1
                #print("Robot in miscare")    
            sterbuffer()
            acasa()
        else:
            self.lbl_robot.configure(text="No robot")
            #serial neconectat
        self.nr_pas=99

    if self.nr_pas==1:
        #filename = fd.askopenfilename(initialdir = "/",title = "Select file",filetypes = (("jpeg files","*.jpg"),("all files","*.*"))) #
        filename = fd.askopenfilename(title = "Select file",filetypes = (("jpeg files","*.jpg"),("all files","*.*"))) #
        if  filename :
            frame = cv2.imread(filename)
            self.nr_pas=4
        else:
            self.nr_pas=0
    if self.nr_pas==2:
        if verificcamera():
            #print("camera ok")
            self.nr_pas=3
        else:
            self.nr_pas=98
    if self.nr_pas==3:
        #print("fac captura")
        frame=captura()
        self.nr_pas=4
        return
    if self.nr_pas==4:
        dx,dy,unghi,tip,dA,dB = cauta(frame,scara,xx,yy,zz,corectie,corecties)
        #caut piesa
        cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
        orig = cv2image.copy()
        imgcvtowin(orig,self.l_video1)
        self.nr_pas=5
        return
    if self.nr_pas==5:
        #decizii
        print(dx,dy,unghi,tip,dA,dB)
        decizii(dx,dy,unghi,tip,dA,dB)
        self.nr_pas=50
        return

    if self.nr_pas==50:
        #print("afisez")
        #frame=captura()
        cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
        orig = cv2image.copy()
        imgcvtowin(orig,self.l_video1)        
        self.nr_pas=98

    if self.nr_pas==98:
        if chk_state.get():
            self.nr_pas=2
        else:
            self.nr_pas=99
            
        


app = Application(master=root)
root.mainloop()
cv2.destroyAllWindows()
#serialPort.close
