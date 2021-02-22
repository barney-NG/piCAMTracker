# next version
* web interface: with touchmove scroll through the last snapshots
* snapshot via camera.capture now (let's see how good it works)
* full cleanup of the html/js part
  * endless loop which GETs the actual image every 500ms has been replaced by:
  * a websocket based approach which syncs the snapshots only when a new one has arrived
* fastMode: 640x480@90fps V1 camera and 640x480@120fps V2 camera. (in shiny conditions)
# Version 0.6.1: 2019-11-30
* Disable wifi access point by default. (problems with wifi client setup)
# Version 0.6: 2019-11-28
* Raspberry 4 Support
  * New OS version (Buster)
* Python-3.8
  * Latest picamera version installed (1.14)
  * opencv-4.1.2
* tracking is bypassed for very fast and big objects
* exposure compensation implementated (amount of backlighting)
* framerate adapts to lightning conditions automatically
* track creation conditions are more strict now
* allow more moving elements in one frame to enable bigger objects
* wifi access point started in parallel to wifi connection. (very alpha!)
* short video sequences (h264) are stored in parallel to motion vectors when debugging is enabled.
  * stored under: /home/pi/piCAMTracker/media/videos
  * the VLC player should be able to play back this files
# Version 0.5.1: 2019-02-20
* handling of webpage enhanced
* test button in screen page implemented
* crossing detection in Y direction simplified again
* debugmotion enhanced again
# Version 0.5: 2019-02-10
* Enable detection from on side only (baseB)
* Detection is more robust now.
* bigger objects are detected better
* Turn detection enabled on GPIO 27
* status led is now working on GPIO 23
* debugmotion enhanced. (it is still an alpha prototype)
* garbage collection disabled
* detection in x direction
# Version 0.4: 2018-08-19
* older versions: please check git for commits 'git log'
