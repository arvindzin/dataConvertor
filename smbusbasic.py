# I2C test program for PCA9555
# Developed by RG for PCM9562 Advantech board
import smbus
import time

class PCA9555 :
			#open Linux device SMBus(2)
			i2c = smbus.SMBus(2)
			#construct a new object with I2C address of the PCA9555
			def __init__(self,address):
					self.address =address
					
			# write a 16 bit value to register pair
			# write low byte of value to register reg
			# write high byte of value to register reg+1
			def writeRegisterPair(self,reg,value):
					low = value & 0xff
					high = (value >>  8) & 0xff
					self.i2c.write_byte_data(self.address,reg,low)
					self.i2c.write_byte_data(self.address,reg+1,high)
			
			# read a 16 bit value to register pair
			def readRegisterPair(self,reg):
					low = self.i2c.read_byte_data(self.address,reg)
					high = self.i2c.read_byte_data(self.address,reg+1)
					return low | (high << 8)
			
			# set IO ports to input if the corresponding direction bit is 1.
			# otherwise set it to output
			def setInputDirection(self,direction):
					self.writeRegisterPair(6,direction)
			
			# set the IO port outputs
			def setOutput(self, value):
					self.writeRegisterPair(2,value)
			# read the IO port inputs
			def getInput(self):
					return self.readRegisterPair(0)

# create a new project to test PCA9555
i2c_address=0x20
ioExpander = PCA9555(
					 i2c_address
					 )

# set as input pins lower byte and higher byte as output.
allin =0xffff
ioExpander.pCA(allin)
read=ioExpander.getInput()
print read 


					