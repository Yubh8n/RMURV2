#!/usr/bin/env python
import cv2 as cv
import MarkerTracker
import numpy as np
import cv2.aruco
import glob


def Calibrate_cam():

    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((9 * 7, 3), np.float32)
    objp[:, :2] = np.mgrid[0:7, 0:9].T.reshape(-1, 2)

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    images = glob.glob('/home/chris/Webcam/*.jpg')

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (7, 9), None)

        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)

            cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners)

            # Draw and display the corners
            cv2.drawChessboardCorners(img, (7, 6), corners, ret)
            #cv2.imshow('img', img)
            #cv2.waitKey(0)
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    return mtx, dist

def Exercise_2():
    '''Use the locate marker method in MarkerTracker.py to locate fourth order markers (n = 4) in a fixed image.
    Visualize  the  location  of  the  detected  marker in the image.'''

    image = cv.imread("/home/chris/PycharmProjects/markertracker/MarkerLocator/documentation/pythonpic/input/hubsanwithmarker.jpg")
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    tracker = MarkerTracker.MarkerTracker(4, 21, 1)
    tracker.locate_marker(gray)

    magnitude = np.sqrt(tracker.frame_sum_squared)
    maximum_magnitude = np.max(magnitude)
    argument = np.arctan2(tracker.frame_imag, tracker.frame_real)
    cv.circle(image, (tracker.pose.x, tracker.pose.y), 100, (255,0,0), thickness=50)
    cv.namedWindow("test", cv.WINDOW_NORMAL)
    cv.imshow("test", image)
    #cv.imwrite("hubsan_magnitude_response_inverted_n4_kernel.png",
    #            (1-magnitude/maximum_magnitude) * 255)
    cv.waitKey(0)

def Exercise_3(mirror=False):
    cam = cv.VideoCapture(0)
    while True:
        ret_val, img = cam.read()

        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        tracker = MarkerTracker.MarkerTracker(4, 8, 1)
        tracker.locate_marker(gray)
        cv.circle(img, (tracker.pose.x, tracker.pose.y), 100, (255,0,0), thickness=50)
        cv.namedWindow("test", cv.WINDOW_NORMAL)
        #cv.imshow("test", img)
        if cv.waitKey(1) == 27:
            break  # esc to quit
    cv.destroyAllWindows()

def Exercise_4():
    cam = cv.VideoCapture(0)
    #marker = cv.imread("/home/chris/PycharmProjects/Lecture18/marker.png")
    dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
    im = cv2.aruco.drawMarker(dic, 20, 400)
    cv.imshow("marker", im)
    while True:

        ret_val, img = cam.read()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, dic, parameters=parameters)
        frame_markers = cv2.aruco.drawDetectedMarkers(img.copy(), corners, ids)
        
        cv.namedWindow("test", cv.WINDOW_NORMAL)
        cv.imshow("test", frame_markers)
        if cv.waitKey(1) == 27:
            break  # esc to quit
    cv.destroyAllWindows()

def Exercise_6(mxt, dist):
    cam = cv.VideoCapture(0)
    dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
    im = cv2.aruco.drawMarker(dic, 20, 400)
    cv.imshow("marker", im)
    while True:

        ret_val, img = cam.read()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, dic, parameters=parameters)
        frame_markers = cv2.aruco.drawDetectedMarkers(img.copy(), corners, ids)
        if ids != None:
            rvec, tvec = cv2.aruco.estimatePoseSingleMarkers(corners, 2, mxt, dist)
            imgWithAruco = cv2.aruco.drawDetectedMarkers(img, corners, ids, (0, 255, 0))
            imgWithAruco = cv2.aruco.drawAxis(imgWithAruco, mxt, dist, rvec, tvec, 2)
        else:
            imgWithAruco = img

        cv.namedWindow("test", cv.WINDOW_NORMAL)
        cv.imshow("test", imgWithAruco)
        if cv.waitKey(1) == 27:
            break  # esc to quit
    cv.destroyAllWindows()

def main():
    #show_webcam(mirror=True)
    mtx, dist = Calibrate_cam()
    Exercise_6(mtx, dist)


if __name__ == '__main__':
    main()
