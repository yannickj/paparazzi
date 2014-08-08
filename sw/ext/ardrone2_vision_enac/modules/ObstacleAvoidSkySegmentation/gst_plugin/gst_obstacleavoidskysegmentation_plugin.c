
/**
 * SECTION:element-obstacleavoidskysegmentation
 *
 * The ObstacleAvoidSkySegmentation Plugin for GStreamer.
 *
 * <refsect2>
 * <title>ObstacleAvoidSkySegmentation launch line</title>
 * |[
 * gst-launch v4l2src device=/dev/video1 ! videorate ! 'video/x-raw-yuv,framerate
=15/1' ! videoscale ! video/x-raw-yuv, width=160, height=120 ! obstacleavoidskys
egmentation adjust_factor=5 verbose=2 tcp_port=2000 ! dspmp4venc ! rtpmp4vpay co
nfig-interval=2 ! udpsink host=192.168.1.255 port=5000
 * ]|
 * </refsect2>
 */

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <gst/gst.h>

#include "gst_obstacleavoidskysegmentation_plugin.h"
#include "obstacleavoidskysegmentation_code.h"



GST_DEBUG_CATEGORY_STATIC (gst_obstacleavoidskysegmentation_debug);
#define GST_CAT_DEFAULT gst_obstacleavoidskysegmentation_debug

/* Filter signals and args */
enum
{
  /* FILL ME */
  LAST_SIGNAL
};

//adjustable parameters
enum
{
  PROP_0,
  PROP_SILENT,
  PROP_TCP,
  PROP_ADJUST_FACTOR,
  PROP_VERBOSE
};

/* the capabilities of the inputs and outputs.
 *
 * describe the real formats here.
 */
static GstStaticPadTemplate sink_factory = GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS ("video/x-raw-yuv, format=(fourcc)UYVY"	)
);

static GstStaticPadTemplate src_factory = GST_STATIC_PAD_TEMPLATE ("src",
    GST_PAD_SRC,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS ("video/x-raw-yuv, format=(fourcc)UYVY"	)
);

GST_BOILERPLATE (Gstobstacleavoidskysegmentation, gst_obstacleavoidskysegmentation, GstElement,
    GST_TYPE_ELEMENT);

static void gst_obstacleavoidskysegmentation_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec);
static void gst_obstacleavoidskysegmentation_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec);

static gboolean gst_obstacleavoidskysegmentation_set_caps (GstPad * pad, GstCaps * caps);
static GstFlowReturn gst_obstacleavoidskysegmentation_chain (GstPad * pad, GstBuffer * buf);

/* GObject vmethod implementations */

static void
gst_obstacleavoidskysegmentation_base_init (gpointer gclass)
{

  GstElementClass *element_class = GST_ELEMENT_CLASS (gclass);

  gst_element_class_set_details_simple(element_class,
      "obstacleavoidskysegmentation",
      "Passthrough element",
      "Calculates stuff on the video, to be fed to an autopilot",
      "Kevin van Hecke");

  gst_element_class_add_pad_template (element_class,
      gst_static_pad_template_get (&src_factory));
  gst_element_class_add_pad_template (element_class,
      gst_static_pad_template_get (&sink_factory));
}

/* initialize the obstacleavoidskysegmentation's class */
static void
gst_obstacleavoidskysegmentation_class_init (GstobstacleavoidskysegmentationClass * klass)
{
  GObjectClass *gobject_class;
  //GstElementClass *gstelement_class;

  gobject_class = (GObjectClass *) klass;

  gobject_class->set_property = gst_obstacleavoidskysegmentation_set_property;
  gobject_class->get_property = gst_obstacleavoidskysegmentation_get_property;

  g_object_class_install_property (gobject_class, PROP_SILENT,
      g_param_spec_boolean ("silent", "Silent", "Produce verbose output.",
          FALSE, G_PARAM_READWRITE));

  g_object_class_install_property (gobject_class, PROP_TCP,
      g_param_spec_uint ("tcp_port", "TCP port", "Output results over tcp",0,65535,
          0, G_PARAM_READWRITE));

  g_object_class_install_property (gobject_class, PROP_ADJUST_FACTOR,
      g_param_spec_uint ("adjust_factor", "adjust factor", "Find more or less ground",0,20,
          0, G_PARAM_READWRITE));

  g_object_class_install_property (gobject_class, PROP_VERBOSE,
      g_param_spec_uint ("verbose", "verbose", "Say More Or Less",0,10,
          0, G_PARAM_READWRITE));
}

/* initialize the new element
 * instantiate pads and add them to element
 * set pad calback functions
 * initialize instance structure
 */
static void
gst_obstacleavoidskysegmentation_init (Gstobstacleavoidskysegmentation * filter,
    GstobstacleavoidskysegmentationClass * gclass)
{

  filter->sinkpad = gst_pad_new_from_static_template (&sink_factory, "sink");
  gst_pad_set_setcaps_function (filter->sinkpad,
      GST_DEBUG_FUNCPTR(gst_obstacleavoidskysegmentation_set_caps));
  gst_pad_set_getcaps_function (filter->sinkpad,
      GST_DEBUG_FUNCPTR(gst_pad_proxy_getcaps));
  gst_pad_set_chain_function (filter->sinkpad,
      GST_DEBUG_FUNCPTR(gst_obstacleavoidskysegmentation_chain));

  filter->srcpad = gst_pad_new_from_static_template (&src_factory, "src");
  gst_pad_set_getcaps_function (filter->srcpad,
      GST_DEBUG_FUNCPTR(gst_pad_proxy_getcaps));

  gst_element_add_pad (GST_ELEMENT (filter), filter->sinkpad);
  gst_element_add_pad (GST_ELEMENT (filter), filter->srcpad);
  filter->silent = FALSE;

}

static void
gst_obstacleavoidskysegmentation_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec)
{
  Gstobstacleavoidskysegmentation *filter = GST_OBSTACLEAVOIDSKYSEGMENTATION (object);

  switch (prop_id) {
  case PROP_SILENT:
    filter->silent = g_value_get_boolean (value);
    break;
  case PROP_TCP:
    tcp_port = g_value_get_uint (value);
    break;
  case PROP_ADJUST_FACTOR:
    adjust_factor = g_value_get_uint (value);
    break;
  case PROP_VERBOSE:
    verbose = g_value_get_uint (value);
    break;
  default:
    G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
    break;
  }
}

static void
gst_obstacleavoidskysegmentation_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec)
{
  Gstobstacleavoidskysegmentation *filter = GST_OBSTACLEAVOIDSKYSEGMENTATION (object);

  switch (prop_id) {
  case PROP_SILENT:
    g_value_set_boolean (value, filter->silent);
    break;
  case PROP_TCP:
    g_value_set_uint (value, tcp_port);
    break;
  case PROP_ADJUST_FACTOR:
    g_value_set_uint (value, adjust_factor);
    break;
  case PROP_VERBOSE:
    g_value_set_uint (value, verbose);
    break;
  default:
    G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
    break;
  }
}

/* GstElement vmethod implementations */

/* this function handles the link with other elements */
static gboolean
gst_obstacleavoidskysegmentation_set_caps (GstPad * pad, GstCaps * caps)
{
  Gstobstacleavoidskysegmentation *filter;
  GstPad *otherpad;

  filter = GST_OBSTACLEAVOIDSKYSEGMENTATION (gst_pad_get_parent (pad));
  otherpad = (pad == filter->srcpad) ? filter->sinkpad : filter->srcpad;
  gst_object_unref (filter);

  //make the image size known
  const GstStructure *str;
  str = gst_caps_get_structure (caps, 0);
  gint tmp;
  gst_structure_get_int (str, "width", &tmp);
  imgWidth = (unsigned int)tmp;
  gst_structure_get_int (str, "height", &tmp);
  imgHeight = (unsigned int)tmp;
  g_print ("The video size is %dx%d\n", imgWidth, imgHeight);

  my_plugin_init();

  return gst_pad_set_caps (otherpad, caps);
}


/* chain function
 * this function does the actual processing
 */
static GstFlowReturn gst_obstacleavoidskysegmentation_chain (GstPad * pad, GstBuffer * buf)
{
  Gstobstacleavoidskysegmentation *filter;

  filter = GST_OBSTACLEAVOIDSKYSEGMENTATION (GST_OBJECT_PARENT (pad));

  unsigned char * img = GST_BUFFER_DATA(buf);

  my_plugin_run(img);

  return gst_pad_push (filter->srcpad, buf);
}

/* entry point to initialize the plug-in
 * initialize the plug-in itself
 * register the element factories and other features
 */
static gboolean
obstacleavoidskysegmentation_init (GstPlugin * obstacleavoidskysegmentation)
{
  /* debug category for filtering log messages
   */

  GST_DEBUG_CATEGORY_INIT (gst_obstacleavoidskysegmentation_debug, "obstacleavoidskysegmentation",
      0, "The ObstacleAvoidSkySegmentation will do something.");

  return gst_element_register (obstacleavoidskysegmentation, "obstacleavoidskysegmentation", GST_RANK_NONE,
      GST_TYPE_OBSTACLEAVOIDSKYSEGMENTATION);
}

/* PACKAGE: this is usually set by autotools depending on some _INIT macro
 * in configure.ac and then written into and defined in config.h, but we can
 * just set it ourselves here in case someone doesn't use autotools to
 * compile this code. GST_PLUGIN_DEFINE needs PACKAGE to be defined.
 */
#ifndef PACKAGE
#define PACKAGE "ObstacleAvoidSkySegmentation"
#endif

/* gstreamer looks for this structure to register obstacleavoidskysegmentation
 */
GST_PLUGIN_DEFINE (
    GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    "obstacleavoidskysegmentation",
    "The ObstacleAvoidSkySegmentation will do something",
    obstacleavoidskysegmentation_init,
    VERSION,
    "LGPL",
    "ObstacleAvoidSkySegmentation",
    "http://gstreamer.net/"
)
