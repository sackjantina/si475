import cv2

datadir = '/data/ford/'

for i in range(6):
    img = cv2.imread(datadir+'Cam'+str(i)+'/1326032045346941.tiff')
    img = cv2.resize(img, (img.shape[0]//4, img.shape[1]//4))
    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    cv2.imshow('Cam'+str(i), img)

cv2.waitKey()
