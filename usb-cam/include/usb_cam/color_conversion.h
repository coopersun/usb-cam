#ifndef USB_CAM_COLOR_CONVERSION_H
#define USB_CAM_COLOR_CONVERSION_H

#include <string.h>

namespace usb_cam {

/** Clip a value to the range 0<val<255. For speed this is done using an
 * array, so can only cope with numbers in the range -128<val<383.
 */
extern const unsigned char uchar_clipping_table[];

inline unsigned char CLIPVALUE(int val)
{
  // Old method (if)
  /*   val = val < 0 ? 0 : val; */
  /*   return val > 255 ? 255 : val; */

  // New method (array)
  const int clipping_table_offset = 128;
  return uchar_clipping_table[val + clipping_table_offset];
}

void YUV2RGB(const unsigned char y, const unsigned char u, const unsigned char v, unsigned char* r,
                    unsigned char* g, unsigned char* b);

void uyvy2rgb(char *YUV, char *RGB, int NumPixels);

void yuyv2rgb(char *YUV, char *RGB, int NumPixels);

inline void rgb242rgb(char *YUV, char *RGB, int NumPixels)
{
  memcpy(RGB, YUV, NumPixels * 3);
}
void mono102mono8(char *RAW, char *MONO, int NumPixels);

}

#endif
