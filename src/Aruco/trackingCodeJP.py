import numpy as np
import cv2
import cv2.aruco as aruco
import os


aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
camera_matrix = np.array([[997.14, 0, 620.27], [0, 1000.62, 401.64], [0, 0, 1]])
distortion_matrix = np.array([[0.07521726747524617, -0.1364085827047695, 0.01111997443259327, -0.001636969888224974]])
cap = cv2.VideoCapture(0)

def findArucoMarkers(img, markerSize=4, totalMarkers=50, draw=True):
    # """
    # :param img: iage in which to find the aruco markers
    # :param markerSize: the size of the markers
    # :param totalMarkers: total number of markers that composethe dictionary
    # :param draw: flag to draw bbox around markers detected
    # :return: bounding boes and id numbers of markers detected
    # """
    
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    key = getattr(aruco,'DICT_4X4_50') # For now let's force the marker key.
    # key = getattr(aruco,f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    arucoDict = aruco.Dictionary_get(key)
    arucoParams = aruco.DetectorParameters_create()
    coners, ids, rejected = aruco.detectMarkers(imgGray, arucoDict, parameters=arucoParams)
    #print(ids)
    
    if draw:
        aruco.drawDetectedMarkers(img, coners)
        
    return [coners, ids]

def augmentArucoBasic(bbox, id, img, drawId=True):    
    tl = int(bbox[0][0][0]), int(bbox[0][0][1])
    tr = bbox[0][1][0], bbox[0][1][1]
    br = bbox[0][2][0], bbox[0][2][1]
    bl = bbox[0][3][0], bbox[0][3][1]
    if drawId:
        cv2.putText(img, str(id), tl, cv2.FONT_HERSHEY_PLAIN, 2, (255,0,255), 2)
        
def draw_axis(frame, camera_matrix, distortion_matrix, board, verbose=True):
    corners, ids, rejected_points = cv2.aruco.detectMarkers(frame, aruco_dict)
    
    if corners is None or ids is None:
        return None
    if len(corners) != len(ids) or len(corners) == 0:
        return None
    
    print(corners)
    print(type(corners))

    try:
        ret, p_rvec,p_tvec = aruco.estimatePoseBoard(corners,ids, board, camera_matrix, distortion_matrix) 


        if p_rvec is None or p_tvec is None:
            return None
        if np.isnan(p_rvec).any() or np.isnan(p_tvec).any():
            return None
        cv2.aruco.drawAxis(frame,
                        camera_matrix,
                        distortion_matrix,
                        p_rvec,
                        p_tvec,
                        0.01)

    except cv2.error:
        return None

    if verbose:
        print('Translation : {0}'.format(p_tvec))
        print('Rotation    : {0}'.format(p_rvec))
        print('Distance from camera: {0} m'.format(np.linalg.norm(p_tvec)))

    return [p_rvec, p_tvec]


def track(matrix_coefficients, distortion_coefficients):
    while True:
        ret, frame = cap.read()
        # operations on the frame come here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Change grayscale
        # Use 5x5 dictionary to find markers
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters_create()  # Marker detection parameters
# lists of ids and the corners beloning to each id
        corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict,
                                                                parameters=parameters,
                                                                cameraMatrix=matrix_coefficients,
                                                                distCoeff=distortion_coefficients)


        # if np.all(ids is not None):  # If there are markers found by detector
        #     for i in range(0, len(ids)):  # Iterate in markers
        #         # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
        #         rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], 0.01, matrix_coefficients,
        #                                                                    distortion_coefficients)
        #         (rvec - tvec).any()  # get rid of that nasty numpy value array error
        rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners, 0.01, matrix_coefficients, distortion_coefficients)
        aruco.drawDetectedMarkers(frame, corners)  # Draw A square around the markers
        if np.all(ids is not None): 
            for i in range(0, len(ids)):  # Iterate in markers
                aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec[i], tvec[i], 0.01)  # Draw Axis
        # Display the resulting frame
        cv2.imshow('frame', frame)
        # Wait 3 milisecoonds for an interaction. Check the key and do the corresponding job.
        key = cv2.waitKey(3) & 0xFF
        if key == ord('q'):  # Quit
            break
    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
    

def main():
    cap = cv2.VideoCapture(0)

    while True:
        success, img = cap.read()
        arucoFound = findArucoMarkers(img) # gives bbox and id in arucoFound[0] and arucoFound[1]
        corners = arucoFound[0]
        rvecs,tvecs,_objPoints = aruco.estimatePoseSingleMarkers(corners, 4 , camera_matrix, distortion_matrix)
        # cv2.aruco.drawAxis(img, camera_matrix, distortion_matrix, rvecs, tvecs,  0.01)
        
        # print(arucoFound)
        # print(arucoFound[1])
        # print len(arucoFound[1])
        cv2.imshow("Image", img)
        cv2.waitKey(1) #gives delay of 1 milisecond

if __name__ == "__main__":
    # main()
    track(camera_matrix, distortion_matrix)