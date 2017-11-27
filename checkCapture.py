import PicModule
import time
import sys



r = PicModule.PicMbus(127,Device="COM6:")

r.module.serial.flushInput()


# conversion Des IO au CCPx
CapturePinToIO= (0,3,8,9)

#variable des captures

CaptureTimer = [ 0,0,0,0 ]   
CaptureCount = [ 0,0,0,0 ]


# Vérification de la configuration des IO 0,3,8 et 9
# 
# soit 
#  IOCONFIG_TIME_CAPTURE_LOW
# ou
#  IOCONFIG_TIME_CAPTURE_HIGH
#


def checkConfig(Pin,CaptureMode):
   configvalue = r.readConfig(CapturePinToIO[Pin])
   if  not( configvalue == CaptureMode):
     r.config(CapturePinToIO[Pin],CaptureMode)
     time.sleep(0.5)

# Affiche time capture du specifique IO
def showCapture(value):
   capture = r.readSensor(CapturePinToIO[value])
   CaptureTimer[value] = capture[0]*65536 + capture[1]
   CaptureCount[value] = capture[2]
   print(" | C{}={:<10} n={:<5} ".format(value,capture[0]*65536+capture[1],capture[2]),end='')


# Affichage tout les timers de capture et Timer1 clock
def ShowAllCapture():
   #read free running timer
   T1 = r.readFreeRunningTimer()
   print("Clk:{:<10}   ".format(T1[0]*65536+T1[1]),end='')
   for loop in range(4):
      showCapture(loop)
   print(" ")

# Affichage final de chaque délai
def AfficheLapseTemps( value):
   print('Capteur{} IO{} = '.format(value,CapturePinToIO[value]),end='')
   if CaptureCount[value] == 0:
      print('Non déclanché')
   else:
      Delta = CaptureTimer[value] - CaptureTimer[0]
#      print('Delta = {}'.format(Delta))
#      sys.stdout.flush()
      #conversion de compte à interval en micro-seconde
      Delta = float(Delta)/ float(ClockMhz)
    
      #Le format doit êter ajusté dépendant du clock
      clockFormat = '{:.0f} µs'
      if ClockMhz == 8 :
         clockFormat = '{:.3f} µs'
      if ClockMhz == 4 :
         clockFormat = '{:.2f} µs'
      if ClockMhz == 2 :
         clockFormat = '{:.1f} µs'
         
      print(clockFormat.format(Delta),end='')
      if CaptureCount[value] > 1:
          print(' Multiple déclenchement = {}'.format(CaptureCount[value]))
      else:
          print(' ')
             
# Vérification des configs et changer s'il le faut
for loop in range(4):
  checkConfig(loop, r.IOCONFIG_TIME_CAPTURE_LOW)

# Ajustement du clock pour Timer1  soit (1,2,4 ou 8 Mhz)

# set 1Mhz   #possibilité (1,2,4ou 8)Mhz
r.setFreeRunningTimerClock(4)  
ClockMhz = r.readFreeRunningTimerClock()

#reset counter
r.resetFreeRunningTimer()
for loop in range(4):
   r.resetCounter(CapturePinToIO[loop])

  
   
while(True):
   try:
      ShowAllCapture()
      # verifie si capture du IO9 est déclanchée
      if CaptureCount[3] > 0 :
        break      
   except KeyboardInterrupt:
      quit()
      
#ok analyse des données
# minimum il faut que IO0 soit déclenché

if CaptureCount[0] == 0:
   print('Erreur! Aucun déclenchement de départ')
   quit()

print('Fréquence d''horloge : {}Mhz'.format(ClockMhz))
for loop in range(1,4):
   AfficheLapseTemps(loop)
   
   
