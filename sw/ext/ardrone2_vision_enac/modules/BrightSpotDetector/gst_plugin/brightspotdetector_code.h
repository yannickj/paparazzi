

// Settable by pluging
extern unsigned int imgWidth, imgHeight;
extern unsigned int tcp_port;
extern unsigned char threshtune;

// Called by plugin
void my_plugin_init(void);
void my_plugin_run(unsigned char *frame);

