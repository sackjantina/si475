import cv2 
import numpy as np 
import hashlib 
import sys, traceback 

sys.dont_write_bytecode = True 

from lab01 import adjust_image, apply_filter, rotate_image 

TEST = "FAIL apply_filter"

try: 
    I = cv2.imread('usna_small.jpg')
    h = hashlib.sha256() 
   
    f = np.full((4,4), 1.0 / 16.0)

    J = apply_filter(I, f) 
    h.update(J) 
    if h.hexdigest() != '875957890bd234fc99137288ae972ddba3344b42c8b2caf1b6c2c9b00a991828': 
        print (TEST+' : 1') 
        sys.exit(-1)  

    f = (1.0 / 273.0) * np.array([[1,4,7,4,1],[4,16,26,16,4],[7,26,41,26,7],[4,16,26,16,4],[1,4,7,4,1]])
    J = apply_filter(I, f)  
    h.update(J) 
    if h.hexdigest() != 'ee68650602ba648c03d95df62e3afd6bd609e047ea1d4ff6b65d53e004017c6b': 
        print (TEST+' : 2') 
        sys.exit(-1) 

    f = np.array([[-1, -1, -1],[-1, 8, -1], [-1, -1, -1]])
    J = apply_filter(I, f)  

    h.update(J) 
    if h.hexdigest() != '670678900ab082aa7fc4dd9e84f13071c0831c4d77d63c5349cd6168413f1d76': 
        print (h.hexdigest())
        print (TEST+' : 3') 
        sys.exit(-1) 

    print ('PASS apply_filter') 

except Exception as e: 
    exc_type, exc_value, exc_traceback = sys.exc_info()
    print (TEST +' : '+str(e))
    myerror = traceback.format_tb(exc_traceback)
    for line in myerror:
        line = str(line).split('\n')
        for item in line:
            print('ERROR: '+item)




