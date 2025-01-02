```
gst-launch-1.0 -v videotestsrc ! video/x-raw,framerate=20/1,width=640,height=480,format=I420 ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! rtph264pay config-interval=10 pt=96 ! udpsink host=172.20.10.7 port=5000
```

```
gst-launch-1.0 -v videotestsrc ! video/x-raw,framerate=20/1,width=1920,height=1080,format=I420 ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! rtph264pay config-interval=10 pt=96 ! udpsink host=172.20.10.7 port=5000
```