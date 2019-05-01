import cv2
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt




cap = cv2.VideoCapture("../../Downloads/remember to brake the car.mp4")

Red = []
counter = 0
if (cap.isOpened() == True):
    print "Woooop"
    while (True):
        ret, frame = cap.read()
        if counter == 0:
            #static pixels
            frame[400,530] = 0
            frame[83,557] = 0
            frame[200,300] = 0
            frame[450,100]= 0
            frame[10,10] = 0

            #dynamic pixels
            frame[150,200] = 0
            frame[243,432] = 0
            frame[182,553] = 0
            frame[105,474] = 0
            frame[131,324] = 0
            first_frame = frame
        if (cv2.waitKey(25) & ret == False):
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        r,g,b = cv2.split(frame)
        Red.append(r[10,10])
        #print np.shape(gray)

        #cv2.imshow("frame",gray)
        counter += 1
    cap.release()
    cv2.destroyAllWindows()
else:
    print "Error opening the video!"

print(np.shape(Red))

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.scatter(25,25,25,label='Red')

ax.set_xlim(0,255)
ax.set_ylim(0,255)
ax.set_zlim(0,255)
ax.set_xlabel('Blue')
ax.set_ylabel('Green')
ax.set_zlabel('Red')

#plt.scatter(25,25,25, label='Red')
#plt.xlim(0,255)
#plt.ylim(0,255)
plt.show()

#cv2.imshow("pew", first_frame)
#cv2.waitKey()

