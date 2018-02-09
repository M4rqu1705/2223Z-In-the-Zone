import matplotlib.pyplot as plt
import numpy as np

#Replace array.append with return

distance = 10.0
distanceDivisor = 14.0
speed = 50.0
speedDivisor = 5.0
array = []
 
for x in np.arange(0.0,distance*1.0, 0.1):
  if 0<=x and x<distance/14.0:
    array.append(((speed/5.0)/(distance/14.0)**2.0)*(x**2.0))
  elif distance/14.0<=x and x<distance*3.0/14.0:
    array.append(((speed*3.0/5.0)/(distance/7.0))*(x-distance/14.0)+speed/5.0)
  elif distance*3.0/14.0<=x and x<distance*2.0/7.0:
    array.append(((speed*4/5.0-speed)/(distance*3.0/14.0-distance*2.0/7.0)**2.0)*(x-distance*2.0/7.0)**2.0+speed)
  elif distance*2.0/7.0<=x and x<distance*5.0/7.0:
    array.append(speed)
  elif distance*5.0/7.0<=x and x<distance*11.0/14.0:
    array.append(((speed*4/5.0-speed)/(distance*11.0/14.0-distance*5.0/7.0)**2.0)*(x-distance*5.0/7.0)**2.0+speed)
  elif distance*11.0/14.0<=x and x<distance*13.0/14.0:
    array.append(-1*((speed*3.0/5.0)/(distance/7.0))*(x-distance*13.0/14.0)+speed/5.0)
  elif distance*13.0/14.0<=x and x<distance:
    array.append(((speed/5.0)/(distance*13.0/14.0-distance)**2.0)*(x-distance)**2.0)


plt.plot(array)
plt.xlabel("Time")
plt.ylabel("Output") 
plt.show()
