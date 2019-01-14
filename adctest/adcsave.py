# Simple example of reading the MCP3008 analog input channels and printing
# them all out.
# Author: Tony DiCola
# License: Public Domain
import time
import numpy
# Import SPI library (for hardware SPI) and MCP3008 library.
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008


# Software SPI configuration:
#CLK  = 18
#MISO = 23
#MOSI = 24
#CS   = 25
#mcp = Adafruit_MCP3008.MCP3008(clk=CLK, cs=CS, miso=MISO, mosi=MOSI)

#Hardware SPI configuration:
SPI_PORT   = 0
SPI_DEVICE = 1
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE,20000))

my_list=[]
count=0
print('Reading MCP3008 values, press Ctrl-C to quit...')
#ADC takes about 3075 samples per second with this setting (while loop - count upto 2000-append an array-save into a file in the end)

try:
	starttime = time.time()*1000
	while ((time.time()*1000-starttime)<1000):
	#while count<6500:
		my_list.append(mcp.read_adc(7))
		count=count+1
	a=numpy.asarray(my_list)
        numpy.savetxt("foo.csv",a,delimiter=",")
except KeyboardInterrupt:
	a=numpy.asarray(my_list)
	numpy.savetxt("foo.csv",a,delimiter=",")


