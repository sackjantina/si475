#Spring 2022 SI475
# Jack Santina 235580
import cv2
import numpy as np

# This function applys the brightness and contrast 
# settings to the provided image.  
# Input: 
#   I: an image as a Numpy N-dimensional array 
#   brightness: how much to change the brightness by
#       integer in the range [-255, 255] 
#   contrast: how much to change the contrast by 
#       floating point (a value of 1.0 will not change
#       the contrast)
# 
# Returns: 
#   a new image showing the results of applying the 
#       the supplied brightness and contrast settings 

def adjust_image(I, brightness, contrast): 
    result = np.zeros(I.shape, np.intc) # make new blank image
    for r, row in enumerate(I):
        for c, col in enumerate(row):
            for i, channel in enumerate(col):
                result[r][c][i] = contrast*channel + brightness

    result = np.clip(result, 0, 255)
    result = result.astype(I.dtype)
    return result


# This function applys a filter to an image 
# Input: 
#   I: an image as a Numpy N-dimensional array 
#   h: the filter (kernel) as a 2-D Numpy array 
#
# Returns: 
#   a new image showing the results of applying the 
#       the filter to the image I 
def apply_filter(I, h): 
    result = np.zeros(I.shape, np.intc)
    filter_shape = h.shape

    # COLLABORATED WITH NICK KANG ON THIS MATH
    vert_skip_l = (filter_shape[0]-1)//2
    vert_skip_r = (filter_shape[0]-1)-vert_skip_l
    horz_skip_l = (filter_shape[1]-1)//2
    horz_skip_r = (filter_shape[1]-1)-horz_skip_l

    for r in range(0+vert_skip_l, I.shape[0]-vert_skip_r):
        for c in range(0+horz_skip_l, I.shape[1]-horz_skip_r):
            for i in range(3):
                img_matrix = I[r-vert_skip_l:r+vert_skip_r+1, c-horz_skip_l:c+horz_skip_r+1, i] # get sub-image the same size as the kernel
                img_matrix = img_matrix.astype(np.uint)
                result[r][c][i] = np.sum(np.multiply(h, img_matrix))

    result = np.clip(result, 0, 255)
    result = result.astype(I.dtype)
    return result

# This function rotates an image by theta 
# Input: 
#   I: an image as a Numpy N-dimensional array 
#   theta (integer): number of degrees to rotate the image 
#
# Returns: 
#   a new image which is the rotated version of I 

def rotate_image(I, theta): 
    result = np.zeros(I.shape, I.dtype)
    center_r, center_c = I.shape[0]//2, I.shape[1]//2

    # Create first translation matrix
    translate_to_corner = np.identity(3, dtype=np.intc)
    translate_to_corner[0,2] = -1*center_c
    translate_to_corner[1,2] = -1*center_r

    # Create second translation matrix
    translate_to_center = np.identity(3, dtype=np.intc)
    translate_to_center[0,2] = center_c
    translate_to_center[1,2] = center_r

    # Create rotation matrix
    rotation_matrix = np.identity(3)
    theta = np.deg2rad(theta)
    rotation_matrix[0,0] = np.cos(theta)
    rotation_matrix[1,0] = np.sin(theta)
    rotation_matrix[0,1] = -1*np.sin(theta)
    rotation_matrix[1,1] = np.cos(theta)

    # pre-multiply matrices
    T = translate_to_center@rotation_matrix@translate_to_corner

    for r, row in enumerate(I):
        for c, col in enumerate(row):
            shifted_point = T@np.array([r, c, 1])
            shifted_point = shifted_point.astype(np.intc)
            
            if shifted_point[0] > 0 and shifted_point[0] < I.shape[0] and shifted_point[1] > 0 and shifted_point[1] < I.shape[1]:
                result[shifted_point[0]][shifted_point[1]] = I[r][c]

    return result

# smol = cv2.imread('usna_small.jpg')
# cv2.imshow('smol', smol)
# cv2.waitKey()

#res = adjust_image(smol, 50, 1)
#cv2.imshow('adjust', res)
#cv2.waitKey()

# blur_size = 7
# blur = np.ones((blur_size,blur_size))
# kernel = blur/np.sum(blur)
# kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
# kernel = np.array([[-1, -1, -1], [-1, 8, -1], [-1, -1, -1]])

# kernel = np.ones((3,20))
# kernel = kernel/np.sum(kernel)

# kernel = np.array([[-1, -1, -1],[-1, 8, -1], [-1, -1, -1]]) 
# filtered = apply_filter(smol, kernel)
# cv2.imshow('filtered', filtered)
# cv2.waitKey()

# osprey = cv2.imread('osprey_square.jpg')
# cv2.imshow('osprey', osprey)
# cv2.waitKey()
# rotated = rotate_image(osprey, -180)
# cv2.imshow('rotated', rotated)
# cv2.waitKey()

