#ifndef _IMAGE_
#define _IMAGE_

typedef struct {
  unsigned int  width;
  unsigned int  height;
  unsigned int  bytes_per_pixel; /* 2:RGB16, 3:RGB, 4:RGBA */
  unsigned char pixel_data[128 * 64 * 3 + 1];
} image;

#endif
