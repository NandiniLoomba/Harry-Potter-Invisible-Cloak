import cv2
import numpy as np
import time

#This function is basically starting the webcam
#The parameter which it takes depends upon your system like for default camera use 0
#as im using external webcam so i provided 1
cap=cv2.VideoCapture(1)

#We used time library which comes along python
#The main reason for using it is to provide some time to camera to settle down
time.sleep(3)


#To save the ouput video in a file names as Invisible_Cloak.mp4
# FourCC is a 4-byte code used to specify the video codec.
# A video codec is software or hardware that compresses and decompresses digital video.

fourcc=cv2.VideoWriter_fourcc(*'mp4v')
width=int(cap.get(3))
height=int(cap.get(4))
#Laast parameter True is for colorful image
output=cv2.VideoWriter("Invisible_Cloak.mp4",fourcc,20,(width,height),True)


#Capturing Background view

background=0
for i in range(30):
    succ,background=cap.read()



#Now comes the real part where we can experience being invisible

while(cap.isOpened()):
    
    #Reading video frame by frame 
    success,frame=cap.read()
    if success==False:
        break
 
    #Converting frame to HSV (Hue(color), Saturation, Value(Brightness))
    #BGR is the default color space of webcams
    #Question is why do we prefer HSV color space over BGR
    #Ans: Try imagining you have an image of a single-color plane with a shadow on it. 
    # In RGB colorspace, the shadow part will most likely have very different characteristics 
    # than the part without shadows. In HSV colorspace, the hue component of both patches is more 
    # likely to be similar: the shadow will primarily influence the value, or maybe satuation 
    # component, while the hue, indicating the primary "color" (without it's brightness and 
    # diluted-ness by white/black) should not change so much.So in short we can say that it is 
    # often more natural to think about a color in terms of hue and saturation 

    #Also color range in 0-360 but in openCv we cannot store value above 8 bits ie. 256
    # so we divide them by half now the color range is : 0-180

    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    
    #In hsv color space, red color comes in two different ranges

    lower_red = np.array([0,120,70])
    upper_red = np.array([10,255,255])

    #masking is like : pixels whose value lies in lower and upper range are visible in white, rest all are in black 
    mask1=cv2.inRange(hsv,lower_red,upper_red)

    lower_red=np.array([170,120,70])
    upper_red=np.array([180,255,255])

    mask2=cv2.inRange(hsv,lower_red,upper_red)
    mask1=mask1+mask2 #bitwise or
    

    #open Morphology reduces noices
    #Morphology dilate makes the image more smoother
    mask1=cv2.morphologyEx(mask1,cv2.MORPH_OPEN,np.ones((3,3),np.uint8),iterations=2)
    mask1=cv2.morphologyEx(mask1,cv2.MORPH_DILATE,np.ones((3,3),np.uint8),iterations=1)
    
    #everything except the cloak
    mask2=cv2.bitwise_not(mask1)

    stream1=cv2.bitwise_and(frame,frame,mask=mask2)
    stream2=cv2.bitwise_and(background,background,mask=mask1)
    out=cv2.addWeighted(stream1,1,stream2,1,0)
    
    output.write(out)
    cv2.imshow("video",out)
    
    #window should shut down if n is pressed
    if cv2.waitKey(1) & 0xFF==ord('n'):
        break


cap.release()
output.release()
cv2.destroyAllWindows()
