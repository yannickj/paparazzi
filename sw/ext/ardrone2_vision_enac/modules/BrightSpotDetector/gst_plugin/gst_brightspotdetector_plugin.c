
/**
 * SECTION:element-brightspotdetector
 *
 * The BrightSpotDetector Plugin for GStreamer.
 *
 * <refsect2>
 * <title>BrightSpotDetector launch line</title>
 * |[
 * gst-launch v4l2src device=/dev/video1 ! videorate ! 'video/x-raw-yuv,framerate=5/1' ! videoscale ! video/x-raw-yuv, width=640, height=368 ! brightspotdetector ! fakesink

 * ]|
 * </refsect2>
 */

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <gst/gst.h>

#include "gst_brightspotdetector_plugin.h"
#include "brightspotdetector_code.h"



GST_DEBUG_CATEGORY_STATIC (gst_brightspotdetector_debug);
#define GST_CAT_DEFAULT gst_brightspotdetector_debug

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
  PROP_THRESHTUNE
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

GST_BOILERPLATE (Gstbrightspotdetector, gst_brightspotdetector, GstElement,
    GST_TYPE_ELEMENT);

static void gst_brightspotdetector_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec);
static void gst_brightspotdetector_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec);

static gboolean gst_brightspotdetector_set_caps (GstPad * pad, GstCaps * caps);
static GstFlowReturn gst_brightspotdetector_chain (GstPad * pad, GstBuffer * buf);

/* GObject vmethod implementations */

static void
gst_brightspotdetector_base_init (gpointer gclass)
{

  GstElementClass *element_class = GST_ELEMENT_CLASS (gclass);

  gst_element_class_set_details_simple(element_class,
      "brightspotdetector",
      "Passthrough element",
      "Calculates stuff on the video, to be fed to an autopilot",
      "Kevin van Hecke");

  gst_element_class_add_pad_template (element_class,
      gst_static_pad_template_get (&src_factory));
  gst_element_class_add_pad_template (element_class,
      gst_static_pad_template_get (&sink_factory));
}

/* initialize the brightspotdetector's class */
static void
gst_brightspotdetector_class_init (GstbrightspotdetectorClass * klass)
{
  GObjectClass *gobject_class;
  //GstElementClass *gstelement_class;

  gobject_class = (GObjectClass *) klass;

  gobject_class->set_property = gst_brightspotdetector_set_property;
  gobject_class->get_property = gst_brightspotdetector_get_property;

  g_object_class_install_property (gobject_class, PROP_SILENT,
      g_param_spec_boolean ("silent", "Silent", "Produce verbose output.",
          FALSE, G_PARAM_READWRITE));

  g_object_class_install_property (gobject_class, PROP_TCP,
      g_param_spec_uint ("tcp_port", "TCP port", "Output results over tcp",0,65535,
          0, G_PARAM_READWRITE));

  g_object_class_install_property (gobject_class, PROP_THRESHTUNE,
      g_param_spec_uint ("threshtune", "threshtune tune", "Changes output of binary image function",0,65535,
          0, G_PARAM_READWRITE));
}

/* initialize the new element
 * instantiate pads and add them to element
 * set pad calback functions
 * initialize instance structure
 */
static void
gst_brightspotdetector_init (Gstbrightspotdetector * filter,
    GstbrightspotdetectorClass * gclass)
{

  filter->sinkpad = gst_pad_new_from_static_template (&sink_factory, "sink");
  gst_pad_set_setcaps_function (filter->sinkpad,
      GST_DEBUG_FUNCPTR(gst_brightspotdetector_set_caps));
  gst_pad_set_getcaps_function (filter->sinkpad,
      GST_DEBUG_FUNCPTR(gst_pad_proxy_getcaps));
  gst_pad_set_chain_function (filter->sinkpad,
      GST_DEBUG_FUNCPTR(gst_brightspotdetector_chain));

  filter->srcpad = gst_pad_new_from_static_template (&src_factory, "src");
  gst_pad_set_getcaps_function (filter->srcpad,
      GST_DEBUG_FUNCPTR(gst_pad_proxy_getcaps));

  gst_element_add_pad (GST_ELEMENT (filter), filter->sinkpad);
  gst_element_add_pad (GST_ELEMENT (filter), filter->srcpad);
  filter->silent = FALSE;

}

static void
gst_brightspotdetector_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec)
{
  Gstbrightspotdetector *filter = GST_BRIGHTSPOTDETECTOR (object);

  switch (prop_id) {
  case PROP_SILENT:
    filter->silent = g_value_get_boolean (value);
    break;
  case PROP_TCP:
    tcp_port = g_value_get_uint (value);
    break;
  case PROP_THRESHTUNE:
    threshtune = g_value_get_uint (value);
    break;
  default:
    G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
    break;
  }
}

static void
gst_brightspotdetector_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec)
{
  Gstbrightspotdetector *filter = GST_BRIGHTSPOTDETECTOR (object);

  switch (prop_id) {
  case PROP_SILENT:
    g_value_set_boolean (value, filter->silent);
    break;
  case PROP_TCP:
    g_value_set_uint (value, tcp_port);
    break;
  case PROP_THRESHTUNE:
    g_value_set_uint (value, threshtune);
    break;
  default:
    G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
    break;
  }
}

/* GstElement vmethod implementations */

/* this function handles the link with other elements */
static gboolean
gst_brightspotdetector_set_caps (GstPad * pad, GstCaps * caps)
{
  Gstbrightspotdetector *filter;
  GstPad *otherpad;

  filter = GST_BRIGHTSPOTDETECTOR (gst_pad_get_parent (pad));
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
static GstFlowReturn gst_brightspotdetector_chain (GstPad * pad, GstBuffer * buf)
{
  Gstbrightspotdetector *filter;

  filter = GST_BRIGHTSPOTDETECTOR (GST_OBJECT_PARENT (pad));

  unsigned char * img = GST_BUFFER_DATA(buf);

  my_plugin_run(img);

  return gst_pad_push (filter->srcpad, buf);
}

/* entry point to initialize the plug-in
 * initialize the plug-in itself
 * register the element factories and other features
 */
static gboolean
brightspotdetector_init (GstPlugin * brightspotdetector)
{
  /* debug category for filtering log messages
   */

  GST_DEBUG_CATEGORY_INIT (gst_brightspotdetector_debug, "brightspotdetector",
      0, "The BrightSpotDetector will output the location of the brightest spot");

  return gst_element_register (brightspotdetector, "brightspotdetector", GST_RANK_NONE,
      GST_TYPE_BRIGHTSPOTDETECTOR);
}

/* PACKAGE: this is usually set by autotools depending on some _INIT macro
 * in configure.ac and then written into and defined in config.h, but we can
 * just set it ourselves here in case someone doesn't use autotools to
 * compile this code. GST_PLUGIN_DEFINE needs PACKAGE to be defined.
 */
#ifndef PACKAGE
#define PACKAGE "BrightSpotDetector"
#endif

/* gstreamer looks for this structure to register brightspotdetector
 */
GST_PLUGIN_DEFINE (
    GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    "brightspotdetector",
    "The BrightSpotDetector will output the location of the brightest spot",
    brightspotdetector_init,
    VERSION,
    "LGPL",
    "BrightSpotDetector",
    "http://gstreamer.net/"
)
