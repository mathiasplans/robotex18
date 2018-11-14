#!/usr/bin/env python
from __future__ import print_function

import roslib
import rospy
import image
from sensor_msgs import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2
import sys
import yaml
import time

bridge = CvBridge()

def main(args):
    img_pub = rospy.Publisher()
    img_sub = rospy.Subscriber("camera/stream", Image, callback)


def saveCameraParams(filename, imageSize, cameraMatrix, distCoeffs, totalAvgErr):
    print("Camera matrix:", cameraMatrix)
    print("Dist coefs:", distCoeffs)

    calibration = {'camera_matrix': cameraMatrix.tolist(), 'distortion_coefficients': distCoeffs.tolist()}

    calibrationData = dict(
        image_width=imageSize[0],
        image_height=imageSize[1],
        camera_matrix=dict(
            rows=cameraMatrix.shape[0],
            cols=cameraMatrix.shape[1],
            dt='d',
            data=cameraMatrix.tolist(),
        ),
        distortion_coefficients=dict(
            rows=disCoeffs.shape[0],
            cols=disCoeffs.shape[1],
            dt='d',
            data=disCoeffs.tolist(),
        ),
        avg_reprojection_error=totalAvgErr,
    )

    with open(filename, 'w') as outfile:
        yaml.dump(calibrationData, outfile)


sqWidth = 10  # number of squares width
sqHeight = 8  # number of squares height
allCorners = []  # all Charuco Corners
allIds = []  # all Charuco Ids
decimator = 0

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
board = cv2.aruco.CharucoBoard_create(sqWidth, sqHeight, 0.0219222222, 0.0109611111, dictionary)

cap = cv2.VideoCapture(0)
nr_of_images = 0

def callback(data):
    global dictionary, allCorners, allIds, board, nr_of_images, sqHeight, sqWidth, decimator
    img = bridge.imgmsg_to_cv2(data, "bgr8")
    
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    markerCorners, markerIds, rejectedImgPoints = cv2.aruco.detectMarkers(img, dictionary)

    if len(markerCorners) > 20:
        nr_of_images += 1
        [ret, charucoCorners, charucoIds] = cv2.aruco.interpolateCornersCharuco(markerCorners, markerIds, img, board)
        if charucoCorners is not None and charucoIds is not None and len(charucoCorners) > 3 and decimator % 3 == 0:
            allCorners.append(charucoCorners)
            allIds.append(charucoIds)

        cv2.aruco.drawDetectedMarkers(img, markerCorners, markerIds)
        cv2.aruco.drawDetectedCornersCharuco(img, charucoCorners, charucoIds)
        time.sleep(0.5)

    #smallimg = cv2.resize(img, (1024, 768))
    # img = cv2.putText(img, str(nr_of_images), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, 0, 3)
    # cv2.imshow("frame", img)
    # if cv2.waitKey(1) != -1:
    #     break
    decimator += 1
    print(nr_of_images)
    if nr_of_images == 50:
        
        print("CALCULATING based on", nr_of_images, "images ...")
        imsize = img.shape
        print(imsize)

        try:
            [ret, cameraMatrix, disCoeffs, rvecs, tvecs] = cv2.aruco.calibrateCameraCharuco(allCorners, allIds, board, imsize,
                                                                                            None, None)
            print("Rep Error:", ret)
            print("SAVING ...")
            print("CameraMatrix:", cameraMatrix)
            print("DistCoef:", disCoeffs)

            saveCameraParams("calibration-live.yml", imsize, cameraMatrix, disCoeffs, ret)
            print("SAVED!")

        except ValueError as e:
            print(e)
        except NameError as e:
            print(e)
        except AttributeError as e:
            print(e)
        except:
            print("calibrateCameraCharuco fail:", sys.exc_info()[0])

        cv2.destroyAllWindows()

