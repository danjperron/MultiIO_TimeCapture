import PicModule
import time


r = PicModule.PicMbus(127,Device="com7")

r.module.serial.flushInput()

CAPTURE_MODE =  r.IOCONFIG_TIME_CAPTURE_LOW


CapturePinToIO= (0,3,8,9)


def checkConfig(Pin):
  try:
     configvalue = r.readConfig(CapturePinToIO[Pin])
     if  not( configvalue == CAPTURE_MODE):
        r.config(CapturePinToIO[pin],CAPTURE_MODE)
        time.sleep(0.5)
  except IOError:
        pass
  except ValueError:
        pass
	 
	  


def showCapture(value):

   while(True):
    try:
      capture = r.readSensor(CapturePinToIO[value])
      break
    except IOError:
      time.sleep(0.1)
      r.module.serial.flushInput()
      continue
    except ValueError:
      continue
	  
   print(" |C{}={:<10} n={:<5} ".format(value,capture[0]*65536+capture[1],capture[2]),end='')


def m():
   while(True):
     try:
      ckh = r.module.read_register(241,0,3)
      ckl = r.module.read_register(240,0,3)
      break;
     except IOError:
      time.sleep(0.1)
      r.module.serial.flushInput()
      continue
     except ValueError:
      continue
   print("Clk:{:<10}   ".format(ckh*65536+ckl),end='')
   for loop in range(4):
     showCapture(loop)
   print(" ")
   

# check config
for loop in range(4):
  checkConfig(loop)

   
while(True):
 
   try:
   
     m()

   except KeyboardInterrupt:
     break   
 
  
	 
     
 
   
   