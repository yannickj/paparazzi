

// Primary Function
void brightspotDetector(unsigned char *frame_buf, int blob[], unsigned int * max_idx,unsigned int * max_idy);

// Secundary Functions
void get1DHist(unsigned char *frame_buf, unsigned int * OneDHist);
unsigned char getThreshold(unsigned int * OneDHist);
void createBinaryImage(unsigned char threshold, unsigned char * frame_buf);
void get2DHist(unsigned char * frame_buf, unsigned int * hist_x, unsigned int * hist_y);
unsigned int cmpfunc (const void * a, const void * b);
unsigned int getMedian(unsigned int * hist,  unsigned int size);

void blobLabeling(unsigned char *frame_buf, unsigned int x, unsigned int y, unsigned int current_label);

