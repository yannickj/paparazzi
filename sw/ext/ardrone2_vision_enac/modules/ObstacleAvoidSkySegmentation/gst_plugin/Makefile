# Include master makefile
include ../../../Makefile.include

# Include gstreamer makefile
include $(DIR_GSTREAMER)/Makefile.include

# Specify paths of interest
override CFLAGS += -I$(DIR_LIB)
override CFLAGS += -I$(DIR_CV)
override CFLAGS += -I$(DIR_MODULES)
override CFLAGS += -I../

all:

gst_plugin := libObstacleAvoidSkySegmentation.so

OBJECTS  =  	gst_obstacleavoidskysegmentation_plugin.o \
		../obstacleavoidskysegmentation_code.o \
		$(DIR_CV)/skysegmentation/skysegmentation.o \
		$(DIR_CV)/trig.o \
		$(DIR_LIB)/udp/socket.o

$(gst_plugin): $(OBJECTS)
$(gst_plugin): override CFLAGS += $(GST_CFLAGS) -D VERSION='"$(version)"' -D DSPDIR='"$(dspdir)"'
$(gst_plugin): override LIBS += $(GST_LIBS)

upload:
	sb2 make -C ./ all && $(DRONE_TOOL) upload_gst_module $(gst_plugin)

doc: $(gst_plugin)
	$(MAKE) -C doc

doc-install: doc
	$(MAKE) -C doc install

all: $(gst_plugin)

%.so: override CFLAGS += -fPIC

%.o:: %.c $(GST_H)
	$(QUIET_CC)$(CC) $(CFLAGS) -MMD -MP -o $@ -c $<

$(bins):
	$(QUIET_LINK)$(CC) $(LDFLAGS) $^ $(LIBS) -o $@

%.so::
	$(QUIET_LINK)$(CC) $(LDFLAGS) -shared $^ $(LIBS) -o $@

%.a::
	$(QUIET_LINK)$(AR) rcs $@ $^

clean:
	echo $(PROCESSOR)
	$(QUIET_CLEAN)$(RM) -v $(gst_plugin) *.o *.d tidsp/*.d tidsp/*.o $(OBJECTS) $(OBJECTS:.o=.d)

dist: base := gst-dsp-$(version)
dist:
	git archive --format=tar --prefix=$(base)/ HEAD > /tmp/$(base).tar
	mkdir -p $(base)
	echo $(version) > $(base)/.version
	chmod 664 $(base)/.version
	tar --append -f /tmp/$(base).tar --owner root --group root $(base)/.version
	rm -r $(base)
	gzip /tmp/$(base).tar

-include *.d tidsp/*.d
