import cv2 
import numpy as np 
import hashlib 
import sys, traceback 

sys.dont_write_bytecode = True 

from lab01 import adjust_image, apply_filter, rotate_image 

TEST = FAIL rotate_image

try: 
    I = cv2.imread('osprey_square.jpg')

    h = hashlib.sha256() 
   
    J = rotate_image(I, 90) 
    h.update(J) 
    if h.hexdigest() != 121928cbbc6c1c29a0b972868be234ba667dd179810a2d21e3f4e7b398f80c49: 
        print (TEST) 
        sys.exit(-1)  

    J = rotate_image(I, 180)  

    h.update(J) 
    if h.hexdigest() != a5dabdc1673284cfe73b03d3b27ecbdc1c4833b62151b69269c89b0027b5bf5a: 
        print (TEST) 
        sys.exit(-1) 

    J = rotate_image(I, -90)   
    h.update(J) 
    if h.hexdigest() != 0d6f03334a996a66208fc818021afb66c4519c546c2642668be14d9e33d8deb3: 
        print (TEST); 
        sys.exit(-1) 

    print (PASS rotate_image) 

except Exception as e: 
    exc_type, exc_value, exc_traceback = sys.exc_info()
    print (TEST + : +str(e))
    myerror = traceback.format_tb(exc_traceback)
    for line in myerror:
        line = str(line).split('\n')
        for item in line:
            print(ERROR: +item)




