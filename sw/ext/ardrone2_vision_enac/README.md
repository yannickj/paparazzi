ardrone2_vision
===============

ARDrone2 Onboard Image Processing

Directories:
------------


 - ardrone2_gstreamer:	compiled gst library and includes for gstreamer on ardrone and gst plugins
 - cv: Computer vision
 - lib: support libraries
 - modules:	paparazzi vision modules
 - modules/XXX/gst_plugin:	gstreamer plugins
 - modules/XXX/

Using:
-----

 - **git clone https://github.com/tudelft/ardrone2_vision** ./paparazzi/sw/ext/ardrone2_vision
 - **make** (will also download submodule, and makes the gstreamer plugins with your custom code)
 - **make drone** (called from ardrone2_gstreamer folder: only needed 1 time per drone: will put the framework on your drone)
put the libPlugin.so on the drone, and start gstreamer using command as in sourcecode

 - (if first make fails at the end) **make install** (will install scratchbox2 and qemu if you didn't have them)
 - (if make is still not working) gedit ./ardrone2_gstreamer/Makefile edit the path to your ardrone crosscompiler and make install again to setup sb2

