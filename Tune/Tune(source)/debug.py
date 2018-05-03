import serial
import matplotlib
matplotlib.use("tkagg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.animation as animation
import tkinter as tk
import turtle
import struct
from ctypes import c_uint8
import time
import sys
import os


def find_data_file(filename):
    if getattr(sys, 'frozen', False):
        # The application is frozen
        datadir = os.path.dirname(sys.executable)
    else:
        # The application is not frozen
        # Change this bit to match where you store your data files:
        datadir = ""

    return os.path.join(datadir, filename)

def open_port(Port_no,self):
    global ser
    comPort = "COM" + Port_no
    baudRate = 115200
    try:
        ser = serial.Serial(comPort,baudRate,timeout = 3)
        self.com_status.config(text = "Connected")
    except serial.SerialException as e:
        if "OSError" in str(e.args):
            self.com_status.config(text = "TImeOut")
        else:
            self.com_status.config(text = "Bluetooth is off")
    except:
        self.com_status.config(text = "Error")

def close_port(self):
    ser.close()
    self.com_status.config(text = "Disconnect")

def begin(self):
    global start,state,x_value,L_color, R_color,Ltarget,Rtarget
    if ser.inWaiting != 0:
        ser.reset_input_buffer()
        time.sleep(.5)
        ser.reset_input_buffer()
    ser.write(b's')
    Lvalue.clear()
    Rvalue.clear()
    x.clear()
    x_value = 0
    Ltarget = 0
    Ltarget_line.clear()
    Rtarget = 0
    Rtarget_line.clear()
    start = True
    state = len(status) - 1
    t.penup()
    t.clear()

def receive():
    global id,status_update,turtle_update,id

    if status_update:
        try:
            for label in status:
                label.config(bg = "SystemButtonFace")
                status[state].config(bg = "red")
        except:
            print(state)
        status_update = False

    if turtle_update:
        print(str(X_Coor) + ',' + str(Y_Coor)+'\n')
        t.setpos(X_Coor - 160,Y_Coor - 120)
        t.pendown()
        t.dot(size,"blue")
        t.penup()
        turtle_update = False

def stop():
    ser.write(b'S')
    global start
    app.after_cancel(id)
    start = False

def PID_animate(i):
    if not start or ser.in_waiting == 0:
        return
    global id,state,x_value,L_color, R_color,Ltarget,Rtarget
    while ser.in_waiting != 0:
        data = ser.readline().decode("ascii").strip('\n')
        if data[0] == 'T':
            a,b = data.split(",")
            Ltarget = int(a[2:])
            Rtarget = -int(b)
        elif data[0] == 'E':
            a,b = data.split(",")
            Ltarget_line.append(Ltarget)
            Rtarget_line.append(Rtarget)
            x.append(x_value)
            x_value+=1
            Lvalue.append(int(a[2:]))
            Rvalue.append(int(b))
            L_ax.clear()
            L_ax.plot(x,Lvalue,'r',x,Ltarget_line,'b-')
            R_ax.clear()
            R_ax.plot(x,Rvalue,'r',x,Rtarget_line,'b-')
        elif  data[0] == 'S':
            state = int(data[2:])
            global status_update
            status_update = True
        elif data[0] == 'P':
            global X_Coor,Y_Coor,turtle_update
            a,b = data.split(",")
            X_Coor = int(a[2:])
            Y_Coor  = int(b)
            turtle_update = True
    receive()

def sendData(text):
    buffer = text.split(',')
    for data in buffer:
        if data == '':
            continue
        if data[0] == '+':
            status.append(tk.Label(status_frame,text = data[1:],font =("Helvetica",15)))
            status[-1].pack(padx =[5,0])
            names.append(data[1:])
            continue
        if data[0] == '-':
            status[int(data[1])].destroy()
            del names[int(data[1])]
            continue
        try:
            data = float(data)
            data_array = bytearray(struct.pack(">f", data))
            for b in data_array:
                ser.write(c_uint8(b))
        except ValueError:
            data = str(data)
            ser.write(data.encode(encoding= 'ascii'))
        time.sleep(0.05)

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
        global status,status_frame
        middle_frame = tk.Frame(self)
        left_frame = tk.Frame(middle_frame)
        status_frame = tk.Frame(left_frame)
        box_frame = tk.Frame(left_frame)
        status = []
        if len(names) > 0:
            for name in names:
                status.append(tk.Label(status_frame,text = name,font =("Helvetica",15)))
        else:
            status.append(tk.Label(status_frame,text = "diff > 0, turn left",font =("Helvetica",15)))
            status.append(tk.Label(status_frame,text = "diff < 0, turn right",font =("Helvetica",15)))
            status.append(tk.Label(status_frame,text = "target lost",font =("Helvetica",15)))
            status.append( tk.Label(status_frame,text = "target not find",font =("Helvetica",15),bg = "red"))

        Turtle_canvas = tk.Canvas(master = status_frame, width = 320, height = 240)
        canvas_frame = tk.Frame(middle_frame)
        canvas = FigureCanvasTkAgg(graph_fig,canvas_frame)
        canvas.show()
        global ani
        ani = animation.FuncAnimation(graph_fig, PID_animate, interval= 20)

        start_button = tk.Button(box_frame,text = "Start",font =("Helvetica",10),command = lambda : begin(self))
        stop_button = tk.Button(box_frame,text = "Stop",font =("Helvetica",10),command =  stop)

        init_frame = tk.Frame(self)
        com_entry = tk.Entry(init_frame,width = 7)
        self.com_status = tk.Label(init_frame,text = "Disconnect",font =("Helvetica",15))
        com_label = tk.Label(init_frame,text = "Com Port:",font =("Helvetica",15),padx = 10)
        connect = tk.Button(init_frame,text ="open",font =("Helvetica",10),padx = 5, command = lambda : open_port(com_entry.get(),self))
        disconnect = tk.Button(init_frame,text ="close",font =("Helvetica",10),padx = 5,command = lambda : close_port(self))
        input_box = tk.Text(box_frame,width = 25,height = 5,wrap = "word")
        send_data = tk.Button(box_frame,text = "Send",command = lambda: sendData(input_box.get("1.0",tk.END)))

        init_frame.pack(fill= tk.X,pady = [15,5])
        middle_frame.pack(fill = tk.X)
        left_frame.pack(side = tk.LEFT)
        status_frame.pack(side = tk.TOP)
        box_frame.pack(side = tk.BOTTOM)
        canvas_frame.pack(side = tk.RIGHT)

        com_label.pack(side = tk.LEFT)
        com_entry.pack(side = tk.LEFT)
        connect.pack(side = tk.LEFT, padx = 5)
        disconnect.pack(side = tk.LEFT, padx = 5)
        Turtle_canvas.pack(side = tk.TOP)
        self.com_status.pack(side = tk.LEFT, padx = 5)
        for label in status:
            label.pack(padx =[5,0],pady = [0,5])
        input_box.pack(pady = 10)
        start_button.pack(padx = [0,10], side = tk.LEFT)
        stop_button.pack(padx = 10, side = tk.LEFT)
        send_data.pack(padx = 10, side = tk.LEFT)
        canvas.get_tk_widget().pack()
        global t
        t = turtle.RawTurtle(Turtle_canvas)
        t.speed("fastest")
global ser,id,state,ani

file = open(find_data_file("log.txt"),'r')
names = []
buf = file.readline()
while buf != '':
    names.append(buf.strip('\n'))
    buf = file.readline()
file.close()

status_update = False
turtle_update = False
start = False
state = 3
Lvalue = []
Rvalue = []
x = []
x_value = 0
Ltarget = 0
Ltarget_line = [Ltarget]
Rtarget = 0
Rtarget_line = [Rtarget]
L_color = 'r'
R_color = 'r'
X_Coor = 0
Y_Coor = 0
size = 10
graph_fig = Figure(figsize = (6,5),dpi = 100)
L_ax = graph_fig.add_subplot(212)
L_ax.set_title("Left")
R_ax = graph_fig.add_subplot(211)
R_ax.set_title("Right")
app = GUI()
app.mainloop()
file = open(find_data_file("log.txt"),"w")
for name in names:
    file.write(name +'\n')
file.close()
try:
    if ser.isOpen():
        ser.close()
except:
    print("no ")
