

all:
	make get_gstreamer
	make -C ./ardrone2_gstreamer
	sb2 make -C ./modules/ObstacleAvoidSkySegmentation/gst_plugin clean all
	sb2 make -C ./standalone/rtp_test clean all
	sb2 make -C ./standalone/mjpeg clean all
	sb2 make -C ./standalone/skysegment clean all

get_gstreamer:
	git submodule sync
	git submodule init
	git submodule update

clean:
	make -C ./ardrone2_gstreamer clean
	sb2 make -C ./modules/ObstacleAvoidSkySegmentation/gst_plugin clean
	sb2 make -C ./standalone/rtp_test clean
	sb2 make -C ./standalone/mjpeg clean
	sb2 make -C ./standalone/skysegment clean

install:
	make -C ./ardrone2_gstreamer install

drone:
	make -C ./ardrone2_gstreamer drone

cleanspaces:
	find . -name '*.[ch]' -exec sed -i {} -e 's/[ \t]*$$//' \;
	find . -name 'Makefile*' -exec sed -i {} -e 's/[ \t]*$$//' \;
