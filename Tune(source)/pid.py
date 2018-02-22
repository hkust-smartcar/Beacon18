import serial
import matplotlib
matplotlib.use("tkagg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.animation as animation
import tkinter as tk
import sys

def animate(i):
    global x_value,value1,x,y
    if ser.in_waiting == 0 :
        return
    data = ser.readline().decode("ascii").strip('\n')
    if data != '':
        value1.append(int(data))
        x.append(x_value)
        y.append(0)
        x_value+=1
    ax.clear()
    ax.plot(x,value1,'b-')

def sendPID(m_kp,m_kd,m_ki):
    global kp,kd,ki
    kp = int(m_kp)
    ser.write(b'p' + c_uint8(kp))
    kd = int(m_kd)
    ser.write(b'd' + c_uint8(kd))
    ki = int(m_ki)
    ser.write(b'i' + c_uint8(ki))

def open_port(Port_no):
    global ser
    comPort = "COM" + Port_no
    baudRate = 115200
    ser = serial.Serial(comPort,baudRate,timeout=5)

def start():
    ser.write(b"s")
    if ser.readline() != b's\n':
        sys.exit
    kp = int.from_bytes(ser.read(), byteorder='little', signed=False)
    kd = int.from_bytes(ser.read(), byteorder='little', signed=False)
    ki = int.from_bytes(ser.read(), byteorder='little', signed=False)

class GUI(tk.Tk):
    def __init__(self,*args,**kwargs):
        tk.Tk.__init__(self,*args,**kwargs)

        container = tk.Frame(self)
        container.pack(side ="top", expand = True)
        container.grid_rowconfigure(0,weight =1)
        container.grid_columnconfigure(0,weight =1)

        self.frames = {}
        frame = Graph(container,self)
        self.frames[Graph] = frame
        frame.pack(fill = tk.X)
        #frame.grid(row = 0 ,column = 0, sticky = "nsew")
        self.show_frame(Graph)

    def show_frame(self,cont):
        frame = self.frames[cont]
        frame.tkraise()

class Graph(tk.Frame):
    def __init__(self,parent,controller):
        tk.Frame.__init__(self,parent)

        connect_frame = tk.Frame(self)
        connect_frame.pack(fill= tk.X)
        com_label = tk.Label(connect_frame,text = "Com Port:",font =("Helvetica",15))
        com_entry = tk.Entry(connect_frame,width = 5)
        connect = tk.Button(connect_frame,text ="open", command = lambda : open_port(com_entry.get()))
        disconnect = tk.Button(connect_frame,text ="close", command = lambda : ser.close())

        canvas_frame = tk.Frame(self)
        canvas_frame.pack(fill= tk.X)
        canvas = FigureCanvasTkAgg(fig,canvas_frame)
        canvas.show()

        action_frame = tk.Frame(self)
        action_frame.pack(fill= tk.X)
        start_button = tk.Button(action_frame,text = "Start", command = lambda : start,width = 10)
        stop_button = tk.Button(action_frame,text = "Stop", command = lambda : ser.write(b"s"),width = 10)

        value_frame = tk.Frame(self)
        value_frame.pack(fill= tk.X)
        kp_label = tk.Label(value_frame,text = "Kp")
        kp_entry = tk.Entry(value_frame,width = 10)
        kp_entry.insert(0,kp)

        kd_label = tk.Label(value_frame,text = "Kd")
        kd_entry = tk.Entry(value_frame,width = 10)
        kd_entry.insert(0,kd)

        ki_label = tk.Label(value_frame,text = "Ki")
        ki_entry = tk.Entry(value_frame,width = 10)
        ki_entry.insert(0,ki)

        ok = tk.Button(value_frame,text="Set",width = 15,command = lambda: sendPID(kp_entry.get(),kd_entry.get(),ki_entry.get()))

        com_label.pack(side = tk.LEFT)
        com_entry.pack(side = tk.LEFT)
        connect.pack(side = tk.LEFT, padx = 5)
        disconnect.pack(side = tk.LEFT, padx = 5)
        canvas.get_tk_widget().pack()
        start_button.pack(padx= 10,side = tk.LEFT)
        stop_button.pack(padx= 5,side = tk.LEFT)
        kp_label.pack(padx = 5,side =tk.LEFT)
        kp_entry.pack(padx = 5,side =tk.LEFT)
        kd_label.pack(padx = 5,side =tk.LEFT)
        kd_entry.pack(padx = 5,side =tk.LEFT)
        ki_label.pack(padx = 5,side =tk.LEFT)
        ki_entry.pack(padx = 5,side =tk.LEFT)
        ok.pack(pady = 10)

global ser,kp,kd,ki

value1 =[0]
x = [0]
y = [0]
x_value = 0

fig = Figure(figsize = (5,3),dpi = 100)
ax = fig.add_subplot(111)

app = GUI()
ani = animation.FuncAnimation(fig, animate, interval=60)
app.mainloop()

ser.close()
