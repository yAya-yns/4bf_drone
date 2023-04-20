import cv2 as cv
import numpy as np

camera_res = (640, 480)
post_crop = (19, 44, 617, 394)

img_shape = (post_crop[2] - post_crop[0], post_crop[3] - post_crop[1])

def order_points(pts):
    '''
    From: https://stackoverflow.com/questions/62295185/warping-a-license-plate-image-to-be-frontal-parallel
    '''
    # Step 1: Find centre of object
    center = np.mean(pts)

    # Step 2: Move coordinate system to centre of object
    shifted = pts - center

    # Step #3: Find angles subtended from centroid to each corner point
    theta = np.arctan2(shifted[:, 0], shifted[:, 1])

    # Step #4: Return vertices ordered by theta
    ind = np.argsort(theta)
    return pts[ind]

def view(img, text=''):
    cv.imshow(text, img)
    cv.waitKey()
    # exit()

def unwrap(img : cv.Mat):
    '''
    highly influenced by: https://stackoverflow.com/questions/62295185/warping-a-license-plate-image-to-be-frontal-parallel
    '''
    assert img.ndim == 2, img.ndim

    # print(img.shape)

    _, thresh = cv.threshold(img, 150, 255, cv.THRESH_BINARY)
    img_blur = cv.GaussianBlur(thresh, (5, 5), 1)
    # view(img_blur)

    dst = cv.Canny(img_blur, 50, 200, None, 3)

    # view(dst)
    contours, hierarchy = cv.findContours(dst, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    # print("Number of Contours found = " + str(len(contours)))

    maxArea = 0
    biggest = np.array([])
    index = None
    for i, cnt in enumerate(contours):  # Change - also provide index
        area = cv.contourArea(cnt)
        if area > 500:
            peri = cv.arcLength(cnt, True)
            approx = cv.approxPolyDP(cnt,0.02*peri, True)
            if area > maxArea and len(approx) == 4:
                biggest = approx
                maxArea = area
                index = i  # Also save index to contour
    warped = None  # Stores the warped license plate image
    height = img.shape[0]
    width = height * 74 // 27
    # cv.drawContours(background, contours, index, (255,255,255), 3)
    # view(background)
    if index is not None: # Draw the biggest contour on the image
        # cv.drawContours(background, contours, index, (255, 255, 255), 3)

        src = np.squeeze(biggest).astype(np.float32) # Source points
        
        
        # Destination points
        dst = np.float32([[0, 0], [0, height - 1], [width - 1, 0], [width - 1, height - 1]])

        # Order the points correctly
        biggest = order_points(src)
        dst = order_points(dst)
        # print(biggest)
        # print(dst)

        # Get the perspective transform
        M = cv.getPerspectiveTransform(src, dst)

        # Warp the image
        img_shape = (width, height)
        # print(img_shape)
        warped = cv.warpPerspective(img, M, img_shape, flags=cv.INTER_LINEAR)

    margins = 5
    # view(warped)
    return warped[margins: height - margins, margins : width - margins]

def segment(img):
    _, thresh = cv.threshold(img, 100, 255, cv.THRESH_BINARY)
    dst = cv.Canny(thresh, 140, 150, None, 3)

    # view(dst)
    contours, hierarchy = cv.findContours(dst, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    # print("Number of Contours found = " + str(len(contours)))
    # view(img)
    cnts = np.concatenate(contours)
    x, y, w, h = cv.boundingRect(cnts)
    img_cropped = thresh[y:y+h, x:x+w]

    # view(img_cropped)
    return [img_cropped[:, i * w//4 : (i+1) * w//4 + 1] for i in range(0, 4)]

def classify_number(img):
    padding = 20
    img = img[padding:img.shape[0] - padding, padding:img.shape[1] - padding]
    edges = cv.Canny(img, 50, 150)
    contours, hierarchy = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    # print("Number of Contours found = " + str(len(contours)))

    angles = []
    for cnt in contours:
        rect = cv.minAreaRect(cnt)
        if rect[1][1] * rect[1][0] < 500:
            continue
        angle = rect[-1] + (0 if rect[1][1] > rect[1][0] else 90)
        # print(angle)
        angles.append(angle)

    #     cv.drawContours(blank,[box],0,(0,255,255),2)
    # view(blank)
    print(len(angles))
    # view(img)
    if len(angles) != 4:
        mapping = {6 : 2, 2 : 3, 5 : 5}
        return mapping[len(angles)]
    
    # find horizontal lines
    h_line = np.count_nonzero(np.abs(np.abs(np.array(angles)) - 90) < 20)
    mapping = [6, 4, 1]
    return mapping[h_line]

def task5(img, rotate180 = False):
    if rotate180:
        img = cv.rotate(img, cv.ROTATE_180)
    unwrapped = unwrap(img)
    view(unwrapped)
    
    chars = segment(unwrapped)
    assert len(chars) == 4
    ret = [classify_number(chars[i]) for i in range(len(chars))]
    if rotate180:
        print(f'task 5 output: {ret[::-1]}')
    else:
        print(f'task 5 output: {ret}')
    return ret
    
if __name__ == '__main__':
    # img_path = r'/Users/yefan/Downloads/task5-obj/0022.png'
    # img_path = r'src/perception/task5_test_data/0009.png'
    # img_path = r'/Users/yefan/Downloads/task5-obj/0027.png'
    # img_path = r'/Users/yefan/Downloads/task5-obj/0015.png'
    # img_path = r'/Users/yefan/Downloads/task5-obj/0004.png'
    img_path = r'/Users/yefan/Downloads/qwer.jpg'
    # img_path = r'/Users/yefan/Downloads/test.jpg'
    img_obj = cv.imread(img_path, cv.IMREAD_GRAYSCALE)
    # assert img_obj != None
    # view(img_obj)
    try:
        task5(img_obj, rotate180=False)
    except:
        task5(img_obj, rotate180=True)
    # if False:
    #     view(chars[3])
    #     print(classify_number(chars[3]))
    # else:
    #     for i in range(0,4):
    #         # view(chars[i])
    #         print(classify_number(chars[i]))

    
    


