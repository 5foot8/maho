#!/usr/bin/python

import argparse
import multiprocessing
import queue
import time
import sqlite3
import threading

import cv2
import subprocess
import numpy as np
import pylab

import os
import sys
import math

# import datetime

from maho.camera import IPCamera
from maho.adsb import Dump1090
from maho.util import AzimuthAltitudeDistance

from time import sleep


def restart_program():
     
    print("Restarting")
    time.sleep(0)
    python = sys.executable
    os.execl(sys.executable, 'python', __file__, *sys.argv[1:])
    
find_aircraft_icao = ("")
window_name = ("Maho")

def find_aircraft():
    find_aircraft_icao = input("Which aircraft ICAO are you looking for? ")
    

#conn = sqlite3.connect('planeDB.db')
#print ("Opened database successfully");


def camera_control(camera_host, camera_port, camera_user, camera_pass, q):
    """Control a maho.Camera based on inputs from a multiprocessing queue"

        On startup this function will place the stream_url in the queue if
        camera communication works. If it fails the exception for why will be
        placed in the queue before exiting """

    try:
        camera = IPCamera(camera_host, camera_port, camera_user, camera_pass)
        q.put(camera.get_rtsp_url())
    except RuntimeError as exc:
        q.put(exc)

    try:
        while True:
            camera.move_to(*q.get())
    except KeyboardInterrupt:
        pass
    
    
        
            


def track_closest_aircraft(latitude, longitude, elevation, host, port, q):
    """Forward adsb messages to a Queue

    Args:
        host (str): The dump1090 host
        port (int): The dump1090 port
        q (queue): Messages will be placed in this queue

        On startup this function will place True in the queue if dump1090
        starts properly. If it fails the exception will be placed in the
        queue before exit
    """
    
    try:
        d = Dump1090(host, port)
        q.put(True)
    except IOError as exc:
        q.put(exc)
        return

    target = None
    target_distance = None
    last_azi = None
    


    aad = AzimuthAltitudeDistance(latitude, longitude, elevation)
    

    try:
        for aircraft in d.updates():
            lat, lng = aircraft.position
            azimuth, altitude, distance = aad.calculate(
                lat,
                lng,
                aircraft.altitude
            )
            

            if aircraft.position[0] > latitude:
                northSouth = ('N')
            else:
                northSouth = ('S')
                
            if lat > latitude:
                northSouth_t = ('N')
            else:
                northSouth_t = ('S')                
            

            # if we don't have a target we do now
            # or target is old, then use this new aircraft
            # or new aircraft isn't the target, but it is closer, so we switch!

            # print (("<--{}-->{}km {} {}").format(aircraft,int((distance)/1000), aircraft.icao, time.time()))
                        
            # make sure aircraft swaps between cameras as it passes over
            # so the correct camera is pointing at the aircraft, if this is not
            # here the camera continues to follow untill an aircraft is closer.
            
            # if you are using only one camera you dont require this section
            
            my_lat = 53.43450
            
            '''if find_aircraft_icao = aircraft.icao:
                target = aircraft
                print (find_aircraft_icao)
            else:
                pass'''
            
            
            if latitude >= my_lat and target is None:
                pass
            elif (latitude >= my_lat and  target.icao == aircraft.icao and lat < my_lat):
                target = None
                #q.put((target, azimuth, altitude, distance))
            else:
                pass
            
            if latitude <= my_lat and target is None:
                pass
            elif (latitude <= my_lat and  target.icao == aircraft.icao and lat > my_lat):
                target = None
                #q.put((target, azimuth, altitude, distance))
            else:
                pass
            
            '''try
            while find_aircraft_icao == aircraft.icao:
                target = aircraft
                print (find_aircraft_icao)
            else:
                pass'''
            
            
            
            
            
            # Mondified code to split between the North and South facing cameras.
            
            #North facing camera:
            if (latitude >= my_lat) and (aircraft.altitude >= (30000 * 0.3048)) or (aircraft.position[0] < latitude):
            
                if (target is None or target.age > 10 or target.icao != aircraft.icao and distance < target_distance) \
                    and northSouth is ('N'):
                    #print (distance, target_distance)
                    #print (lat-latitude)
                    target = aircraft
                else:
                    pass
                
            #South facing camera:
            
            if (latitude <= my_lat) and (aircraft.altitude >= (30000 * 0.3048)) or (aircraft.position[0] > latitude):
            
                if (target is None or target.age > 10 or target.icao != aircraft.icao and distance < target_distance) \
                    and northSouth is ('S'):
                    target = aircraft
                else:
                    pass
                
            else:
                pass
            
            #print (find_active, " ", find_aircraft_icao)

            # if we aren't the target at this point then bail
            
            if target != aircraft:
                continue
            
            target = aircraft
            target_distance = distance
            q.put((target, azimuth, altitude, distance))
    except KeyboardInterrupt:
        pass


def go_maho(
    latitude,
    longitude,
    elevation,
    camera_host,
    camera_port,
    camera_user,
    camera_pass,
    adsb_host,
    adsb_port,
):


    
   # fork a process to communicate with dump1090
    targets = multiprocessing.Queue()
    tracker = multiprocessing.Process(
        target=track_closest_aircraft,
        args=(latitude, longitude, elevation, adsb_host, adsb_port, targets,)
    )
    tracker.start()

    # fist thing in the queue will be startup status
    # True if good
    # an Exception if bad
    status = targets.get()
    if isinstance(status, Exception):
        raise RuntimeError("Unable to connect to dump1090 on {}:{}: {}".format(
            adsb_host,
            adsb_port,
            status
        ))

    # run camera control in own process as moving the camera
    # can block for seconds
    camera_queue = multiprocessing.Queue()
    camera = multiprocessing.Process(
        target=camera_control,
        args=(camera_host, camera_port, camera_user, camera_pass, camera_queue,)
    )
    # myCamera = args.camera_host
    camera.start()

    # fist thing in the queue will be startup status
    # Stream URL if good
    # an Exception if bad
    stream_url = camera_queue.get()
    if isinstance(stream_url, Exception):
        raise RuntimeError("Unable to connect to camera on {}:{}: {}".format(
            camera_host,
            camera_port,
            stream_url
        ))

    cap = cv2.VideoCapture(stream_url)
    ret, frame = cap.read()

    # cv2.namedWindow("maho")

    # orb = cv2.ORB_create()

    # build a mask that's the center of the frame
    # we'll focus searching for aircraft in this region
    # search_mask = np.zeros((frame.shape[0], frame.shape[1], 1), dtype=np.uint8)
    cx = frame.shape[1] / 2
    cy = frame.shape[0] / 2

    size = 0.33

    search_rect = (
        (int(cy - (cy * size)), int(cx - (cx * size))),
        (int(cy + (cy * size)), int(cx + (cx * size)))
    )

    # openCV UI main loops
    start = None
    end = None
    fps = 0
    elapsed = 0

    target = None
    last_target = None
    last_azi = 0
    last_aztime = 0
    # distanceOld = 0
    # photoTaken = 0
    try:
        while True:
            start = time.time()

            
            # grab a frame from the camera
            ret, frame = cap.read()
            # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # check for new / updated target info
            try:
                target, azimuth, altitude, distance = targets.get(False)
                if last_target is None or target.icao != last_target.icao:
                    last_target = target

                    print("Now tracking {} / {} - Distance: {}m".format(
                        target.icao,
                        target.callsign,
                        int(distance)
                    ))

                print("{} | azi: {:.3f}, alt: {:.3f}, dist: {}m.".format(
                    target,
                    azimuth,
                    altitude,
                    int(distance)
                ))
                
                # a function to slow down large camera movements and also
                # keep the camera moving when large gaps between reading ASDB data
                # from the aeroplane.
 
                aztime = time.time()
                angle_move = (last_azi - azimuth)
                time_between = (aztime - last_aztime)
                angle_move_rate = (angle_move / time_between)
                
                
                if (abs(angle_move_rate)) > 15:
                    angle_move_rate = (angle_move_rate / 2)
                else:
                    pass
                
                if time_between == 0:
                    pass
                else:
                    #print (angle_move_rate," Angle move")
                    pass
                
                last_aztime = aztime
                last_azi = azimuth
                
                sleep(1)
                camera_queue.put(((azimuth + angle_move_rate), altitude))
                
                
            except queue.Empty:
                pass

            # annotate the frame
            if target:
                cv2.putText(
                    frame,
                    target.callsign or target.icao,
                    (0, 50),
                    cv2.FONT_HERSHEY_DUPLEX,
                    1.5,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA
                )
                
                txt = "{0:.3f}, {1:.3f} @ {2:.0f}m (dist: {3:.0f}m)".format(
                    target.position[0],
                    target.position[1],
                    target.altitude,
                    distance
                )
                cv2.putText(
                    frame,
                    txt,
                    (10, 75),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    .5,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA
                )

                cv2.rectangle(frame, search_rect[0][::-1], search_rect[1][::-1]
                    , (0, 0, 255), 2)

                # draw a line in the frame at the south point
                # the left edge is always 45 degrees less than camera angle
                # right edge is 45 degrees more than azimuth angle.

                if frame.any() is None:
                    bx = 1
                    by = 1
                else:
                    bx = int(frame.shape[1])
                    by = int(frame.shape[0])
                
                # fov = ((azimuth - 45),(azimuth + 45))  #field of view
                
                findSouth = ((45 - int(azimuth - 180)))
                findWest = ((45 - int(azimuth - 270)))
                findEast = ((45 - int(azimuth - 90)))
                
                if findSouth >= 180:
                    findNorth = ((findSouth - 180))
                else:
                    findNorth = ((findSouth + 180))
                    
                southLine = int((findSouth / 90) * bx)
                westLine = int((findWest / 90) * bx)
                eastLine = int((findEast / 90) * bx)
                northLine = int((findNorth / 90) * bx)
                
                cv2.line(frame, (southLine, (by-100)),
                (southLine, by), (0, 0, 255), 2)
                cv2.line(frame, (westLine, (by-100)),
                (westLine, by), (0, 255, 0), 2)
                cv2.line(frame, (eastLine, (by-100)),
                (eastLine, by), (0, 255, 255), 2)
                cv2.line(frame, (northLine, (by-100)),
                (northLine, by), (255, 255, 255), 2)

                cv2.putText(
                    frame,
                    "S",
                    ((southLine + 5), (by-80)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 0, 255),
                    1,
                    cv2.LINE_AA
                )

                cv2.putText(
                    frame,
                    "W",
                    ((westLine + 5), (by-80)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA
                )

                cv2.putText(
                    frame,
                    "E",
                    ((eastLine + 5), (by-80)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 255, 255),
                    1,
                    cv2.LINE_AA
                )

                cv2.putText(
                    frame,
                    "N",
                    ((northLine + 5), (by-80)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA
                )

                cv2.putText(
                    frame,
                    "fps: {0:02d} ({1:03d} ms)".format(fps, elapsed),
                    (10, 100),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    .5,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA
                )
                cv2.putText(
                    frame,
                    "Camera Position: Az: {:.0f}, Alt: {:.0f}".format(azimuth, altitude),
                    (10, 125),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    .5,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA
                )

            
            # different ways of displaying the final output.
            # uncomment to activate, not sure if you cannot have more than three active.
            
            my_lat = 53.43450 
            
            if latitude >= my_lat:
                window_name = ('Maho NORTH')
            else:
                window_name = ('Maho SOUTH')
            
            # display it
            # small = cv2.resize(frame, (0,0), fx=1, fy=1)
            # small = frame
            # small = cv2.resize(frame, (0,0), width, height)
            # cv2.imshow('maho', frame)
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            # cv2.setWindowProperty('Maho', cv2.WINDOW_FULLSCREEN)
            # cv2.setWindowProperty('Maho', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            # cv2.setWindowProperty('Maho', cv2.WINDOW_AUTOSIZE, cv2.WINDOW_FULLSCREEN)
            cv2.setWindowProperty(window_name, cv2.WINDOW_AUTOSIZE, cv2.WND_PROP_FULLSCREEN)
            # cv2.namedWindow('Maho', cv2.WND_PROP_FULLSCREEN)
            # cv2.resizeWindow('Maho', 1000, 700)
            cv2.imshow(window_name, frame)
            
            
            
            # handle input
            keypress = cv2.waitKey(1) & 0xFF

            end = time.time()
            elapsed = int((end - start) * 1000)
            fps = int(1000 / elapsed)

            if keypress == ord('q'):
                raise KeyboardInterrupt

            # Create a timestamp

            # timestr = time.strftime("%Y%m%d-%H%M%S")

            # Press 's' to save an image
            # Image saves with timestamp as filename
            # and a note is added to the script window to show
            # that the file has been created.

            def saveScreen():
                timestr = time.strftime("%Y%m%d-%H%M%S")
                cv2.imwrite('MC' + timestr + '-' + target.icao + '.png', frame)
                # cv2.imwrite('MC' + timestr + '.png', frame)
                print("Image saved MC" + timestr)

            if keypress == ord('s'):
                saveScreen()
                
            if keypress == ord('r'):
                restart_program()
            
            if keypress == ord('f'):
                find_aircraft()


    except KeyboardInterrupt:
        tracker.terminate()
        tracker.join()

        camera.terminate()
        camera.join()

        cap.release()
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(
        prog='maho',
        description='ADS-B assisted aircraft spotting',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument('--latitude', type=float, required=True,
    help='Latitude of the camera')
    parser.add_argument('--longitude', type=float, required=True,
    help='Longitude of the camera')
    parser.add_argument('--elevation', type=float, required=True,
    help='Elevation of the camera')

    parser.add_argument('--camera-host', type=str, required=True,
    help='Camera hostname/ip')
    parser.add_argument('--camera-port', type=int, default=80,
    help='Camera port')

    parser.add_argument('--camera-user', type=str, required=True,
    help='Camera username')
    parser.add_argument('--camera-pass', type=str, required=True,
    help='Camera password')

    parser.add_argument('--adsb-host', type=str, default='localhost',
    help='dump1090 hostname/ip')
    parser.add_argument('--adsb-port', type=int, default=30002,
    help='dump1090 TCP raw output port')

    args = parser.parse_args()

    try:
        go_maho(
            args.latitude,
            args.longitude,
            args.elevation,
            args.camera_host,
            args.camera_port,
            args.camera_user,
            args.camera_pass,
            args.adsb_host,
            args.adsb_port
        )

        # myCamera = (args.camera_host)

    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        parser.error(exc)
        raise SystemExit


if __name__ == "__main__":
    main()

