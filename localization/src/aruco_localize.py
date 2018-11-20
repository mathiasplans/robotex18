import numpy
import yaml
import cv2

'''''''''''''''''''''''''''   Field drawing stuff '''''''''''''''''''''''''''


def convertRealCoordsToFieldImg(x, y, round_to_integer=True):
    pixelsPerMeter = (630-121)/4.6  # 630-121 pixels correspond to 4.6 meters

    x *= pixelsPerMeter
    y *= -pixelsPerMeter

    x += 376
    y += 262

    if round_to_integer:
        return int(x), int(y)
    else:
        return x, y


def getFieldWithPoint(x, y):
    field_background = cv2.imread("field.png")
    center = convertRealCoordsToFieldImg(x, y)
    cv2.circle(field_background, center, 3, (255, 0, 0), 3)
    return field_background


'''''''''''''''''''''''''''   Detection stuff '''''''''''''''''''''''''''


camera_matrix = None
distortion_coefficients = None
cap = cv2.VideoCapture(0)

MARKER_SIZE_PHONE = 0.041
MARKER_SIZE_CHARUCO = 0.0109611111
MARKER_SIZE_HUGE = 0.195
MARKER_SIZE_CALCULATOR = 0.0555
MARKER_SIZE_REAL_FIELD = 0.25
marker_ids_to_detect = [10, 11, 21, 22]

ARUCO_PARAMETERS = cv2.aruco.DetectorParameters_create()
ARUCO_PARAMETERS.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

ARUCO_DICT = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)

with open("../calibration/calibration-live.yml", 'r') as stream:
    try:
        calibration = yaml.load(stream)
        camera_matrix = numpy.array(calibration['camera_matrix']['data'])
        distortion_coefficients = numpy.array(calibration['distortion_coefficients']['data'])
    except yaml.YAMLError as exc:
        print(exc)

while True:
    # read the image in and detect the marker & position
    check, img = cap.read()
    grayscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(grayscale, ARUCO_DICT, parameters=ARUCO_PARAMETERS)
    rvec, tvec, _objPoints = cv2.aruco.estimatePoseSingleMarkers(corners, MARKER_SIZE_CALCULATOR, camera_matrix, distortion_coefficients)

    if rvec is not None:
        for i in range(len(rvec)):
            if ids[i] in marker_ids_to_detect:
                img = cv2.aruco.drawAxis(img, camera_matrix, distortion_coefficients, rvec[i], tvec[i], 0.1)

                if ids[i] == 22:
                    marker_position = (2.3, -0.2)

                    # See on see marker, mis on sinisest korvist paremal
                    dist = numpy.sqrt(tvec[i][0][0]**2 + tvec[i][0][2]**2)

                    robot_x = marker_position[0] - tvec[i][0][2]
                    robot_y = marker_position[1] + tvec[i][0][0]

                    field = getFieldWithPoint(robot_x, robot_y)

                    angle_const = 1
                    angle = rvec[i][0][2]*angle_const

                    rotM = numpy.zeros(shape=(3, 3))
                    cv2.Rodrigues(rvec[i - 1], rotM, jacobian=0)
                    ypr = cv2.RQDecomp3x3(rotM)
                    z_telg = numpy.arccos(ypr[4][2][2])

                    if rvec[i][0][0] < 0:
                          angle = angle * -1

                    if angle < 0:
                        angle = -z_telg
                    else:
                        angle = z_telg

                    cv2.putText(field, "rot: " + str(rvec[i][0][2]*angle_const*180/3.141592) + " deg", (30, 30), cv2.FONT_HERSHEY_COMPLEX, 1, 0)

                    pointx = marker_position[0] - dist * numpy.cos(angle)
                    pointy = marker_position[1] + dist * numpy.sin(angle)

                    center = convertRealCoordsToFieldImg(pointx, pointy)
                    cv2.circle(field, center, 3, (100, 255, 0), 3)

                    # print(rvec[i])
                    cv2.imshow("Field", field)

                const = 1  # 180/3.14159265358979323

                rotM = numpy.zeros(shape=(3, 3))
                cv2.Rodrigues(rvec[i - 1], rotM, jacobian=0)
                ypr = cv2.RQDecomp3x3(rotM)
                z_telg = numpy.arccos(ypr[4][2][2])*180/3.14
                teine = numpy.arccos(ypr[5][1][1])*180/3.14
                print(str(int(z_telg)) + " asd: " + str(int(teine)))
                print("\n")

                img = cv2.putText(img, str(rvec[i][0][0]*const), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, 0, 3)
                img = cv2.putText(img, str(rvec[i][0][1]*const), (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, 0, 3)
                img = cv2.putText(img, str(rvec[i][0][2]*const), (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, 0, 3)

    cv2.imshow("Output", img)
    if cv2.waitKey(1) != -1:
        break
cv2.destroyAllWindows()