import cv2 
import numpy as np 
import hashlib 
import sys, traceback 

sys.dont_write_bytecode = True 

from lab01 import adjust_image, apply_filter, rotate_image 

try: 
    I = cv2.imread('usna_small.jpg')
    h = hashlib.sha256() 

    J = adjust_image(I, -300, 1) 
    h.update(J) 
    if h.hexdigest() != '6d7f1c959fff039f5d49a859abe4146ca408ce8cc6ea9b2332fa4447cd70e59b': 
        print ("FAIL adjust_image") 
        sys.exit(-1)  

    J = adjust_image(I, -50, 1.25) 
    h.update(J) 
    if h.hexdigest() != '780856fc06829e43876494475147d0399dc6c3fdd4aeb00ebd9b1e5307c8afcb': 
        print ("FAIL adjust_image") 
        sys.exit(-1) 

    J = adjust_image(I, 50, 0.75)
    h.update(J) 
    if h.hexdigest() != '3ac9855aaad3a133f87e75c4feec962505a6e0d0e91ef7f933539fdd3ec2bbab': 
        print ("FAIL adjust_image"); 
        sys.exit(-1) 

    print ("PASS adjust_image") 

except Exception as e: 
    exc_type, exc_value, exc_traceback = sys.exc_info()
    print ("FAIL adjust_image:" +str(e))
    myerror = traceback.format_tb(exc_traceback)
    for line in myerror:
        line = str(line).split('\n')
        for item in line:
            print("ERROR:" +item)




