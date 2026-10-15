"""Detector accepts RGB pixels only; it has no simulator or target-position access."""
from dataclasses import dataclass
import math
import cv2
import numpy as np

@dataclass(frozen=True)
class Detection:
    u:float
    v:float
    area:float
    bbox:tuple

def detect_red(rgb):
    if rgb.ndim!=3 or rgb.shape[2]!=3 or rgb.dtype!=np.uint8:raise ValueError('Expected uint8 HxWx3 RGB image.')
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    mask=cv2.inRange(hsv,(0,110,25),(10,255,255))|cv2.inRange(hsv,(170,110,25),(179,255,255))
    mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
    contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    result=[]
    for c in contours:
        area=cv2.contourArea(c);perimeter=cv2.arcLength(c,True)
        if area<14 or perimeter==0 or 4*math.pi*area/(perimeter**2)<.5:continue
        moments=cv2.moments(c)
        result.append(Detection(moments['m10']/moments['m00'],moments['m01']/moments['m00'],float(area),tuple(map(int,cv2.boundingRect(c)))))
    return sorted(result,key=lambda d:-d.area)

def bearing(detection,joints,width=320,height=240,fovy=54):
    """Convert a pixel ray plus measured gimbal angles to world bearing angles."""
    f=height/(2*math.tan(math.radians(fovy)/2));x=(detection.u-(width-1)/2)/f;z=((height-1)/2-detection.v)/f
    pan,tilt=joints;c,s=math.cos(tilt),math.sin(tilt)
    y=c-s*z;z=s+c*z
    c,s=math.cos(pan),math.sin(pan);x,y=c*x-s*y,s*x+c*y
    return np.array([math.atan2(-x,y),math.atan2(z,math.hypot(x,y))])
