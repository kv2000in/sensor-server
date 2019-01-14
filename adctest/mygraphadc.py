# -*- coding: utf-8 -*-

import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
from collections import deque
import serial
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import numpy as np

SPI_PORT   = 0
SPI_DEVICE = 1
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

win = pg.GraphicsWindow()
win.setWindowTitle('pyqtgraph example: Scrolling Plots')

nsamples=600 #tamanho das matrizes para os dados
tx_aq = 0 #velocidade da aquisição
intervalo_sp = 0.5 #intervalo para secao de poincare

# 1) Simplest approach -- update data in the array such that plot appears to scroll
#    In these examples, the array size is fixed.
p1 = win.addPlot()
p1.setRange(yRange=[0,35])

p2 = win.addPlot()
p2.setRange(yRange=[-100,100])

p3 = win.addPlot()
p3.setRange(yRange=[-100,100])
p3.setRange(xRange=[-0,35])

#p3.plot(np.random.normal(size=100), pen=(200,200,200), symbolBrush=(255,0,0), symbolPen='w')
'''
p3.setDownsampling(mode='peak')
p3.setClipToView(True)
p3.setRange(xRange=[-100, 0])
p3.setLimits(xMax=0)
'''

data1= np.zeros((nsamples,2),float) #ARMAZENAR POSICAO
vec_0=deque()
vec_1=deque()
vec_2=deque()
ptr1 = 0

data2= np.zeros((nsamples,2),float) #ARMAZENAR VELOCIDADE
diff=np.zeros((2,2),float)
diff_v=deque()

data3= np.zeros((nsamples,2),float)
data3_sp=np.zeros((1,2),float)

ptr3=0

curve1 = p1.plot(data1)
curve2 = p2.plot(data2)
curve3 = p3.plot(data3)

#Coeficientes da calibração do IR
c1=-7.246
c2=44.17
c3=-95.88
c4=85.28

tlast=time.clock()
tlast_sp=time.clock()
#print tlast

def getdata():
    global vec_0, vec_1, vec_2, tlast
    timenow=time.clock()

    if timenow-tlast>=tx_aq:
        #name=input("HUGO")

        tlast=timenow

        t0=float(time.clock())
        str_0 =mcp.read_adc(0)
        t1=float(time.clock()) 
        str_1 =mcp.read_adc(0)
        t2=float(time.clock())
        str_2 =mcp.read_adc(0)

        d0x=float(str_0)
        #d0= c1*d0x**3+c2*d0x**2+c3*d0x+c4
        vec_0=(t0, d0x)

        d1x=float(str_1)
        #d1= c1*d1x**3+c2*d1x**2+c3*d1x+c4
        vec_1=(t1, d1x)

        d2x=float(str_2)
        #d2= c1*d2x**3+c2*d2x**2+c3*d2x+c4
        vec_2=(t2, d2x)

        functions()

def diferenciar():
    global data2


    diff=(data1[-1,1]-data1[-3,1])/(data1[-1,0]-data1[-3,0])

    data2[:-1] = data2[1:]
    data2[-1,1] = diff
    data2[-1,0] = data1[-2,0]


def organizar():
    global data1, data3

    data1[:-1] = data1[1:]
    vec_x1=np.array(vec_1)
    data1[-1]=vec_x1

def EF(): #ESPACO DE FASE
    global data3, ptr3

    data3[:-1] = data3[1:]
    data3[-1,0]=data1[-1,1]
    data3[-1,1]=data2[-1,1]

def SP():
    global timenow_sp, tlast_sp

    timenow_sp=time.clock()

    if timenow_sp-tlast_sp>=intervalo_sp:

        tlast_sp=timenow_sp

        data3_sp[0,0]=data3[-2,0]
        data3_sp[0,1]=data3[-2,1]
        p3.plot(data3_sp, pen=None, symbol='o', symbolPen=None, symbolSize=4, symbolBrush=('r'))
        #print data3_sp

def plotar():
    global ptr1
    curve1.setData(data1)    
    ptr1 += 1
    curve2.setData(data2)
    #curve2.setPos(ptr1, 0)

    #p3.plot(data3)

def functions():

    diferenciar()
    organizar()
    EF()
    SP()
    plotar()

def update1():
    global data1, curve1, ptr1

    getdata()


# update all plots
def update():
    update1()

timer = pg.QtCore.QTimer()
timer.timeout.connect(update)
timer.start(50)



## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
