# maho: ADS-B assisted aircraft spotting

### *All the credit goes to cnelson, I have modified to make the software better fit my own personal requirements for the software. 
* Following aircraft above 30,000 ft.[where contrails are more likely to happen]
* Allow two cameras to work together on two sides of the house
* Take a picture on request from the live video stream
* Restart the interface if it appears to freeze

***I have also elimiated the very cleaver aircraft spotting cv2 code which was compromising the speed on my fairly slow PC, its a great concept but doesn't work so well if there is cloud cover, the code would treat every single feature in the clouds as target points - totalling 1,000's.

A proof of concept application for aircraft spotting using positional data from ADS-B
 and a PTZ IP camera.
 
 ![asa311](https://user-images.githubusercontent.com/604163/36133796-76b08af6-1035-11e8-912a-9106d85e6927.jpg)


This application receives aircraft position updates via ADS-B, calculates the azimuth
and altitude to the aircraft from the camera's position and instructs the camera to
point at that location.

***Very basic image analysis is then performed on the video stream from the camera
to highlight the aircraft.

# How to use

In order to use this application you need the following:

## PTZ IP Camera

* The camera must support the ONVIF protocol and ptz.AbsoluteMove.
* The camera must be positioned level and facing north.
* The camera should have a full view of the sky.
* You must know the latitude, longitude, and elevation of the camera.

## ADS-B receiver

This application depends on [dump1090](https://github.com/mutability/dump1090) to decode ADS-B.
Install it and run with `--net` to enable networking.

## This software
Install it with:

```bash
pip install https://github.com/cnelson/maho/archive/master.zip
```

Run it:

```bash
maho \
--latitude :camera-latitude: \
--longitude :camera-longitude: \
--elevation :camera-elevation-in-meters: \
--camera-host :camera-host-or-ip \
--camera-port :camera-onvif-port: \
--camera-user :camera-username: \
--camera-pass :camera-password: \
--adsb-host :host-running-dump1090: \
--adsb-port :dump1090-raw-tcp-port:
```

# Caveats

Original from Nelson:

* This application was written in an afternoon, it is not suitable for any production use.
* Some tests exist but code coverage is poor.
* Expect false positives from image analysis especially if the aircraft is near the horizon, or
the sky is very cloudy.

5Foot8's:
* The code is written for a very specific install, two cameras, one facing North, one South.
You should be able to comment out one of the cameras, if you wish to use with single camera.
* In the two camera setup, the simplest way I could find to inform the code which camera to use 
was to give a latitude that was fractionally larger for the North facing camera, and slightly
smaller for the South facing one, so be acurate when stating the co-ordinates, as each side of the
building will have fractionally different locations. Set your 'my_lat' at the mid-point of the two 
latitudes.
* Presently you have to run each camera independatly in two CLI windows, I have yet to learn how
to combine into a single instance.
* If running with single camera the code will need to be commented out which selects the
camera to use.
* My two cameras are different models but from the same company, unfortunatley the ONVIF commands
for each camera were slightly different.  I modified the code to identify which camera was 
requesting the data from IP input and select the appropriate commands for each.  If running single 
camera you will need to comment out one of the camera selecting modules.
* Proximity to local airport and wishing to see only 'aircraft-over-the-top' I've limited the
the minimum height to 30,000 ft. This can be adjusted in the code.
* I've run the code for many hours and attempted to put checks on all the occurances that can
cause it to stop running - there may be a couple still in there, it usualy is something to do 
with data not being available and returning None, which breaks the code.
* Sometimes for no apprent reason the camera will stop taking the ONVIF commands, I added a restart
command, just press 'r' on the keyboard, which will hopefully kickstart the feed again, if not
you will need to run the command again in a new CLI window.
* If you live in a place where you rarely see clouds and have a really quick PC, you should be able
with reference to CNelsons original MAHO be able to reinstate the image analysis.


