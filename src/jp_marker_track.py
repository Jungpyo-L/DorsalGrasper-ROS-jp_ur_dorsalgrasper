#!/usr/bin/env python

import os, sys
import rospy

import numpy as np
import cv2
import cv2.aruco as aruco
import os
import pickle

from datetime import datetime

from jp_ur_dorsalgrasper.msg import MarkerPacket
from jp_ur_dorsalgrasper.msg import cmdPacket

IDLE = 0
STREAMING = 1


NO_CMD = 0
START_CMD = 2
IDLE_CMD = 3
RECORD_CMD = 10

currState = IDLE
CMD_in = NO_CMD

aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

def callback(data):
    global CMD_in        
    CMD_in = data.cmdInput
    print CMD_in


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
    arucoParam = aruco.DetectorParameters_create()
    corners, ids, rejected = aruco.detectMarkers(imgGray, arucoDict, parameters=arucoParam)
    #print(ids)
    
    if draw:
        aruco.drawDetectedMarkers(img, corners)
        
    return [corners, ids]

def augmentArucoBasic(bbox, id, img, drawId=True):    
    tl = int(bbox[0][0][0]), int(bbox[0][0][1])
    tr = bbox[0][1][0], bbox[0][1][1]
    br = bbox[0][2][0], bbox[0][2][1]
    bl = bbox[0][3][0], bbox[0][3][1]
    if drawId:
        cv2.putText(img, str(id), tl, cv2.FONT_HERSHEY_PLAIN, 2, (255,0,255), 2)

def draw_axis(frame, camera_matrix, dist_coeff, board, verbose=True):
    corners, ids, rejected_points = cv2.aruco.detectMarkers(frame, aruco_dict)
    
    if corners is None or ids is None:
        return None
    if len(corners) != len(ids) or len(corners) == 0:
        return None

    try:
        ret, p_rvec,p_tvec = aruco.estimatePoseBoard(corners,ids, board, camera_matrix, dist_coeff) 

        if p_rvec is None or p_tvec is None:
            return None
        if np.isnan(p_rvec).any() or np.isnan(p_tvec).any():
            return None
        cv2.aruco.drawAxis(frame,
                        camera_matrix,
                        dist_coeff,
                        p_rvec,
                        p_tvec,
                        0.02)

    except cv2.error:
        return None

    if verbose:
        print('Translation : {0}'.format(p_tvec))
        print('Rotation    : {0}'.format(p_rvec))
        print('Distance from camera: {0} m'.format(np.linalg.norm(p_tvec)))

    return [p_rvec, p_tvec]    

# draw axis for single markers
def draw_axis_single(frame, camera_matrix, dist_coeff, i, verbose=True):
    corners, ids, rejected_points = cv2.aruco.detectMarkers(frame, aruco_dict)
    
    if corners is None or ids is None:
        return None
    if len(corners) != len(ids) or len(corners) == 0:
        return None
    if ids is not None:
        if i in ids == False:
            return None
        else:
            index = np.where(ids == 0)[0]


    try:
        rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[index], 0.01, camera_matrix, dist_coeff) 

        if rvec is None or tvec is None:
            return None
        if np.isnan(rvec).any() or np.isnan(tvec).any():
            return None
        cv2.aruco.drawAxis(frame,
                        camera_matrix,
                        dist_coeff,
                        rvec,
                        tvec,
                        0.01)

    except cv2.error:
        return None

    if verbose:
        print('Translation : {0}'.format(tvec))
        print('Rotation    : {0}'.format(rvec))
        print('Distance from camera: {0} m'.format(np.linalg.norm(tvec)))

    return [rvec, tvec]    

def main():
    global currState
    global CMD_in  
    
    size_of_marker =  0.010 # side lenght of the marker in meter    

    datadir = os.path.expanduser('~') + "/catkin_ws/src/jp_ur_experiment/src/Aruco/workdir/"
    
    # Load data (deserialize)
    with open(datadir + 'calibMtx.pickle', 'rb') as handle:
        loaded_data = pickle.load(handle)

    mtx = loaded_data['mtx']
    dist = loaded_data['dist']
    print mtx

    
    print "here"

    rospy.init_node('jp_marker_track')
    pub = rospy.Publisher('MarkerPacket', MarkerPacket, queue_size=10)
    rospy.Subscriber("cmdPacket",cmdPacket, callback)
    msg = MarkerPacket()

    #Set the boards and save them. 
    board = []
    board.append(aruco.GridBoard_create(1,1,0.01,0.005,aruco_dict,firstMarker=0)) #refpoint: wrist pivot point
    board.append(aruco.GridBoard_create(1,1,0.01,0.005,aruco_dict,firstMarker=1)) #object
    board.append(aruco.GridBoard_create(1,1,0.01,0.005,aruco_dict,firstMarker=2)) #first phalanx
    board.append(aruco.GridBoard_create(1,1,0.01,0.005,aruco_dict,firstMarker=3)) #second phalanx
    board.append(aruco.GridBoard_create(1,1,0.01,0.005,aruco_dict,firstMarker=4)) #third phalanx
    board.append(aruco.GridBoard_create(1,1,0.01,0.005,aruco_dict,firstMarker=5)) #forth phalanx


    while not rospy.is_shutdown():
        
            if currState == IDLE and CMD_in == START_CMD:
            # if True:
                CMD_in = NO_CMD
                currState = STREAMING     
                cap = cv2.VideoCapture(0)
                 # We need to set resolutions.
                # so, convert them from float to integer.
                frame_width = int(cap.get(3))
                frame_height = int(cap.get(4))
                
                size = (frame_width, frame_height)
                
                # Below VideoWriter object will create
                # a frame of above defined The output 
                # is stored in 'filename.avi' file.
                ResultSavingDirectory = os.path.expanduser('~') + '/JPExperiment/' + datetime.now().strftime("%y%m%d")
                if not os.path.exists(ResultSavingDirectory):
                    os.makedirs(ResultSavingDirectory)
                result = cv2.VideoWriter(ResultSavingDirectory + '/tmpFile.avi',cv2.VideoWriter_fourcc(*'MJPG'),30, size)

                recordFlag = False

                while not CMD_in == IDLE_CMD and not rospy.is_shutdown():
                    try:
                        if CMD_in == RECORD_CMD:
                            recordFlag = True
                            CMD_in = NO_CMD

                        success, frame = cap.read()

                        numMarkersGroup = 6

                        tvecs = np.zeros((numMarkersGroup,3))
                        rvecs = np.zeros((numMarkersGroup,3))

                        for i in range(0,numMarkersGroup):
                            output = draw_axis_single(frame, mtx, dist, i, False)
                            if output is not None:                                
                                rvecs[i,:] = output[0].reshape(3)
                                tvecs[i,:] = output[1].reshape(3)
                        
                        frame = cv2.undistort(src = frame, cameraMatrix = mtx, distCoeffs = dist)
                        
                        msg.data = np.concatenate((np.reshape(tvecs, (1,3*numMarkersGroup)), np.reshape(rvecs, (1,3*numMarkersGroup)) ), axis=1)[0,:]  
                        msg.header.stamp = rospy.Time.now()  
                        pub.publish(msg)   

                        cv2.imshow("Image", frame)
                        cv2.waitKey(1) #gives delay of 1 milisecond
                        #Checking for commit 

                        if recordFlag:
                            result.write(frame)
                        
                    except Exception as e:
                     print "SensorComError: " + str(e)
                     pass
            
                cap.release()
                result.release()
                cv2.destroyAllWindows()

                CMD_in = NO_CMD
                currState = IDLE


       

if __name__ == "__main__":
    main()
