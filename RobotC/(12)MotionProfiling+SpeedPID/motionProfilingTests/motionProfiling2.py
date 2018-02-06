import matplotlib.pyplot as plt


#Replace array.append with return

distance = 100
speed = 100
distanceMultiplier = [(1/3.0), (2/3.0)]
speedMultiplier = [(1/5.0), (4/5.0)]
offset = speed/2.0

array = []
 
for x in range(0,distance, 1):
    if(0<=x and x<distance*distanceMultiplier[0]):
        array.append( ((speed-offset)/(distance*distanceMultiplier[0]))*x + offset )
    elif(distance*distanceMultiplier[0]<=x and x<distance*distanceMultiplier[1]):
        array.append( speed )
    elif(distance*distanceMultiplier[1]<= x and x<=distance):
        array.append( ((-speed)/(distance-distance*distanceMultiplier[1])) * (x - distance*distanceMultiplier[1])+speed )


plt.plot(array)
plt.xlabel("Distance")
plt.ylabel("Output") 
plt.show()
