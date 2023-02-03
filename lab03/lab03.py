import cv2
import numpy as np
import glob

def fix_img(img):
    img = cv2.resize(img, (img.shape[0]//4, img.shape[1]//4))
    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    return img

datadir = '/data/ford/'

cam1 = glob.glob(datadir+'Cam1/*')
cam1 = sorted(cam1)
cam2 = glob.glob(datadir+'Cam2/*')
cam2 = sorted(cam2)

print("Number of image pairs:", len(cam1))

sift = cv2.SIFT_create()
matcher = cv2.BFMatcher_create(crossCheck=True)

start = 1000
for i in range(start, len(cam1)):
    # print(cam1[i], cam2[i])
    img1 = cv2.imread(cam1[i])
    img1 = fix_img(img1)
    img2 = cv2.imread(cam2[i])
    img2 = fix_img(img2)
    gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

    keypoints1, descriptors1 = sift.detectAndCompute(gray1, None)
    keypoints2, descriptors2 = sift.detectAndCompute(gray2, None)
    matches = matcher.match(descriptors2, descriptors1)

    matches = sorted(matches, key=lambda x: x.distance)[:20]

    outImg = np.zeros(img1.shape, img1.dtype)
    outImg = cv2.drawMatches(img2, keypoints2, img1, keypoints1, matches, outImg)

    kp1 = [(np.float32(kp.pt[0]), np.float32(kp.pt[1])) for kp in keypoints1]
    kp2 = [(np.float32(kp.pt[0]), np.float32(kp.pt[1])) for kp in keypoints2]
    srcPoints = np.array([kp2[match.queryIdx] for match in matches])
    dstPoints = np.array([kp1[match.trainIdx] for match in matches])
    # print(kp1[0])
    # print(srcPoints[0])

    H, _ = cv2.findHomography(srcPoints, dstPoints, cv2.RANSAC)
    # print(H)

    warpedImg = cv2.warpPerspective(img1, H, (2*img1.shape[1], 2*img1.shape[0]), flags=cv2.WARP_INVERSE_MAP)

    # copy img2 into warped version of image 1
    height = img2.shape[0]
    for c in range(len(img2[0])):
        warpedImg[:height,c] = img2[:,c]

    cv2.imshow('Cam1', img1)
    cv2.imshow('Cam2', img2)
    cv2.imshow('Out', outImg)
    cv2.imshow('Warped Image', warpedImg)
    cv2.waitKey(5)

