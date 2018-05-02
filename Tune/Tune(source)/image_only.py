import serial
import matplotlib
matplotlib.use("tkagg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.animation as animation
import tkinter as tk
from PIL import Image, ImageTk
import time
import os
import struct
from ctypes import c_uint8

def open_port(Port_no,self,com_status):
    global ser
    comPort = "COM" + Port_no
    baudRate = 115200
    try:
        ser.append(serial.Serial(comPort,115200,timeout=5))
        ser.append(serial.Serial("COM8",115200,timeout=5))
        com_status.config(text = "Connected")
    except serial.SerialException as e:
        if "OSError" in str(e.args):
            com_status.config(text = "TImeOut")
        else:
            com_status.config(text = "Bluetooth is off")
    except:
        com_status.config(text = "Error")
    self.newWindow = tk.Toplevel(self)
    self.app = video_page(self.newWindow)

def close_port(self,com_status):
    if self.newWindow:
        self.newWindow.destroy()
    global ser
    for bt in ser:
        bt.close()
    ser = []
    com_status.config(text = "Disconnect")

def start(mode,self):
    global status
    for bt in ser:
        if bt.inWaiting() != 0:
            bt.reset_input_buffer()
            time.sleep(.5)
            bt.reset_input_buffer()
    status = True
    global width,imageSize,height
    ser[0].write(b'c')
    num = ser[0].readline().decode("ascii")
    width = int(num[0:-1])
    self.width_label.config(text = "Width: " + str(width))
    num = ser[0].readline().decode("ascii")
    imageSize = int(num[0:-1])
    height = int(imageSize * 8 / width)
    self.height_label.config(text = "Height: " + str(height))
    video_animate()

def stop():
    global status
    ser[0].write(b's')
    status = False
    app.after_cancel(id)

def save_status(save_mode):
    global save
    save = not save

    if save:
        save_mode.config(text = "On")
    else:
        save_mode.config(text = "Off")

def video_animate():
    global image,img,pixels,frameNo,id
    if status:
        pixels = ser[0].read(int(imageSize/2)) + ser[1].read(int(imageSize/2))
    image = Image.frombytes('1',(width,height),pixels)
    img = ImageTk.PhotoImage(image)
    if save:
        image.save(newpath + r'\frame'+str(frameNo)+'.bmp')
        frameNo+=1
    fig.itemconfig(fig.image,image = img)
    app.update_idletasks()
    id = app.after(300,video_animate)

def change_setting(output,self):
    global contrast,brightness
    if output == "c+":
        ser[0].write(b"e")
        contrast+=1
    elif output == "c-":
        ser[0].write(b"r")
        contrast-=1
    elif output == "b+":
        ser[0].write(b"q")
        brightness+=1
    elif output == "b-":
        ser[0].write(b"w")
        brightness-=1
    self.c_label.config(text = contrast)
    self.b_label.config(text = brightness)

class GUI(tk.Tk):
    def __init__(self,*args,**kwargs):
        tk.Tk.__init__(self,*args,**kwargs)

        container = tk.Frame(self)
        container.pack(side ="top",fill = "both", expand = True)

        frame = main_page(container,self)
        frame.grid(row = 0 ,column = 0 , sticky = "nsew")
        frame.tkraise()

class main_page(tk.Frame):
    def __init__(self,parent,controller):
        tk.Frame.__init__(self,parent)

        status_frame = tk.Frame(self)
        com_status = tk.Label(status_frame,text = "Disconnect",font =("Helvetica",15))

        main_frame = tk.Frame(self)
        com_label = tk.Label(main_frame,text = "Com Port:",font =("Helvetica",15),padx = 10)
        com_entry = tk.Entry(main_frame,width = 7)
        connect = tk.Button(main_frame,text ="open",font =("Helvetica",10),padx = 5, command = lambda : open_port(com_entry.get(),self,com_status))
        disconnect = tk.Button(main_frame,text ="close",font =("Helvetica",10),padx = 5,command = lambda : close_port(self,com_status))

        com_label.pack(side = tk.LEFT)
        com_entry.pack(side = tk.LEFT)
        connect.pack(side = tk.LEFT, padx = 5)
        disconnect.pack(side = tk.LEFT, padx = 5)
        com_status.pack()

        main_frame.pack(fill= tk.X,pady = [15,5])
        status_frame.pack(fill= tk.X,pady=10)

class video_page(tk.Frame):
    def __init__(self,parent):
        global fig,img
        self.parent = parent
        self.frame = tk.Frame(parent)
        self.fig_frame = tk.Frame(parent)
        self.value_frame = tk.Frame(parent)
        self.setting_frame = tk.Frame(parent)
        self.save_frame = tk.Frame(parent)

        self.width_label = tk.Label(self.value_frame,text = "Width: ",font =("Helvetica",9))
        self.height_label = tk.Label(self.value_frame,text = "Height: ",font =("Helvetica",9))
        self.c_label = tk.Label(self.setting_frame,text= "0",font =("Helvetica",9))
        self.b_label = tk.Label(self.setting_frame,text= "0",font =("Helvetica",9))

        connect = tk.Button(self.frame,text ="Start", command = lambda:start("video",self),font =("Helvetica",11))
        stop_button = tk.Button(self.frame,text ="Stop", command = stop,font =("Helvetica",11))
        c_plus = tk.Button(self.setting_frame,text ="contrast +", command = lambda: change_setting("c+",self),font =("Helvetica",9))
        c_minus = tk.Button(self.setting_frame,text ="contrast -", command = lambda: change_setting("c-",self),font =("Helvetica",9))
        b_plus = tk.Button(self.setting_frame,text ="brightness +", command = lambda: change_setting("b+",self),font =("Helvetica",9))
        b_minus = tk.Button(self.setting_frame,text ="brightness -", command = lambda: change_setting("b-",self),font =("Helvetica",9))
        fig = tk.Canvas(self.fig_frame, width=width, height=height)
        img = ImageTk.PhotoImage(image)
        fig.image = fig.create_image(100,80,image = img)

        save_mode = tk.Label(self.save_frame,text = "Off", font =("Helvetica",10))
        save_image = tk.Button(self.save_frame,text ="save", command = lambda: save_status(save_mode), font =("Helvetica",10))

        connect.pack(side = tk.LEFT)
        stop_button.pack( side = tk.LEFT,padx = 15)
        self.frame.pack(pady = [20,10])
        fig.pack(padx= [70,0])
        self.fig_frame.pack()
        self.width_label.pack(padx = [0,10],side = tk.LEFT)
        self.height_label.pack(side = tk.LEFT)
        self.value_frame.pack()
        c_plus.pack(side = tk.LEFT,padx = [10,0])
        self.c_label.pack(side = tk.LEFT,padx = 10)
        c_minus.pack(side = tk.LEFT)
        b_plus.pack(side = tk.LEFT,padx = 10)
        self.b_label.pack(side = tk.LEFT)
        b_minus.pack(side = tk.LEFT,padx = 10)
        self.setting_frame.pack(pady=10)
        save_image.pack(side = tk.LEFT)
        save_mode.pack(side = tk.LEFT,padx =[5,0])
        self.save_frame.pack()

#start of the program
global img,fig,ani,id
ser = []
status = False
save = False
width = 240
height = 180
imageSize = width*height /8
contrast = 0
brightness = 0
frameNo = 0

image_path = r'.\image\sample'
folder_no = 1
newpath= image_path +  str(folder_no)
while os.path.exists(newpath) and len(os.listdir(newpath)):
    folder_no+=1
    newpath = image_path + str(folder_no)
if not os.path.exists(newpath):
    os.makedirs(newpath)

pixels = b'\x00' * int(imageSize)
image = Image.frombytes('1',(width,height),pixels)

app = GUI()
app.mainloop()


'''
ser =[]
ser.append(serial.Serial("COM7",115200,timeout=5))
ser.append(serial.Serial("COM8",115200,timeout=5))
ser[0].write(b'c')
ser[0].inWaiting()
ser[1].flushInput()
ser[0].flushInput()
ser[1].inWaiting()
ser[0].close()
ser[1].close()
pixel = ser[0].read(4800) + ser[1].read(4800)

ser = serial.Serial("COM8",115200,timeout=5)
ser.write(b'e')
ser.read_all()
pixels = ser.read(size = int(imageSize))
image = Image.frombytes('1',(width,height),pixels)
ser.inWaiting()
ser.flushInput()
ser.close()
'''
