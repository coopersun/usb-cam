#ifndef COLOR_CONVERSION_CUH
    #define COLOR_CONVERSION_CUH
    extern "C" void yuyv2rgb_cuda(char *YUV, char *RGB, int NumPixels, int num_blocks, int block_size);
#endif
