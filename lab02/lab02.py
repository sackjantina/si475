# SI475 Lab02
# Jack Santina m235580

import cv2
import numpy as np

def dog(I, sigma1, sigma2):
    k_size = (5,5)
    return cv2.GaussianBlur(I, k_size, sigma1) - cv2.GaussianBlur(I, k_size, sigma2)

def hough(I):
    # extract edges using Canny
    edge_img = cv2.Canny(I, 150, 190)
    cv2.imshow('Edge Image', edge_img)

    # initalize parameter space: rho's and theta's
    w, h = edge_img.shape
    maxR = 2*max(w,h)
    rs = list(range(2*maxR))
    thetas = np.arange(0, 360, 1)

    # create accumulator array and initialize to zero
    accumulator = np.zeros((len(rs), len(thetas)), dtype=I.dtype)
    # print(accumulator.shape)

    # for each edge pixel (all non-zero or 255 pixels in edge image)
    rows, cols = np.nonzero(edge_img)
    for x, y in zip(rows, cols):
        # for each theta
        for t in range(len(thetas)):
            # calculate rho
            rho = x*np.cos(np.deg2rad(thetas[t])) + y*np.sin(np.deg2rad(thetas[t])) + maxR
            rho = int(rho)

            # increment accumulator at rho, theta
            # print(rho, thetas[t])
            accumulator[rho, t] += 1


    # find maximum values in accumulator
    num_lines = 10
    lines = []
    for i in range(num_lines):
        index = np.unravel_index(accumulator.argmax(), accumulator.shape)
        lines.append(index)
        # print(index, accumulator[index[0], index[1]])
        accumulator[index[0], index[1]] = 0

    acc_img = np.zeros(I.shape)

    # draw lines on img
    for line in lines:
        rho, t = line[0], line[1]
        theta = thetas[t]
        # print(rho, theta)
        a = np.cos(np.deg2rad(thetas[t]))
        b = np.sin(np.deg2rad(thetas[t]))
        x0 = ((rho-maxR) * a) - (1000 * -1 * b)
        x1 = ((rho-maxR) * a) + (1000 * -1 * b)
        y0 = ((rho-maxR) * b) - (1000 * a)
        y1 = ((rho-maxR) * b) + (1000 * a)
        x0 = int(x0)
        y0 = int(y0)
        x1 = int(x1)
        y1 = int(y1)
        # print(f"({rho}, {thetas[t]}), {accumulator[rho, t]}")
        # print(f"({x0},{y0}) - ({x1},{y1})")

        cv2.line(I, (y0, x0), (y1, x1), (255,255,255), 1)

    # print("IMG Dimensions:", I.shape, maxR)
    return I, accumulator

# img = cv2.imread('card.jpeg')
# cv2.imshow('usna', img)
# cv2.waitKey()

# dog = dog(img, 1, 0.5)
# cv2.imshow('dogged', dog)
# cv2.waitKey()

# edges = cv2.Canny(img, 50, 100)
# cv2.imshow('filtered', edges)

# hough, accumulator = hough(img)
# cv2.imshow('accumulator', accumulator)
# cv2.imshow('hough', hough)
# cv2.waitKey()
