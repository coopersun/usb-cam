/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#define __STDC_CONSTANT_MACROS
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <poll.h>

#include <ros/ros.h>
#include <boost/lexical_cast.hpp>
#include <sensor_msgs/fill_image.h>

#include <usb_cam/usb_cam.h>
#include <usb_cam/color_conversion.h>

#ifdef USE_CUDA
#include <cuda_runtime.h>
#include "cuda/color_conversion.cuh"
#endif

#define CLEAR(x) memset (&(x), 0, sizeof (x))
/*

xavier一共三个trigger信号，每个信号的占空比50%
trigger_1	相机触发信号，板载直接连接,相机目前只支持20fps trigger，正在与厂商联系，后期支持10fps
trigger_2	预留触发信号，通过线束留给客户使用
trigger_3	预留触发信号，通过线束留给客户使用

例：
	TZTEK_CPLD_CAMERA_TRIGGER_1_20_FPS
		相机的trigger，设置为20fps
	TZTEK_CPLD_CAMERA_TRIGGER_1_START      
		相机的trigger，开始输出波形
	TZTEK_CPLD_CAMERA_TRIGGER_1_STOP
		相机的trigger，停止输出波形
*/


//#define TZTEK_CPLD_CAMERA_TRIGGER_ALL_STOP      0X00
//
//#define TZTEK_CPLD_CAMERA_TRIGGER_1_START      0XF0
//#define TZTEK_CPLD_CAMERA_TRIGGER_2_START      0XF1
//#define TZTEK_CPLD_CAMERA_TRIGGER_3_START      0XF2
//
//#define TZTEK_CPLD_CAMERA_TRIGGER_1_STOP      0XF4
//#define TZTEK_CPLD_CAMERA_TRIGGER_2_STOP      0XF5
//#define TZTEK_CPLD_CAMERA_TRIGGER_3_STOP      0XF6
//
//#define TZTEK_CPLD_CAMERA_TRIGGER_1_10_FPS      0X11
//#define TZTEK_CPLD_CAMERA_TRIGGER_1_20_FPS      0X12
//#define TZTEK_CPLD_CAMERA_TRIGGER_1_30_FPS      0X13
//
//#define TZTEK_CPLD_CAMERA_TRIGGER_2_10_FPS      0X21
//#define TZTEK_CPLD_CAMERA_TRIGGER_2_20_FPS      0X22
//#define TZTEK_CPLD_CAMERA_TRIGGER_2_30_FPS      0X23
//
//#define TZTEK_CPLD_CAMERA_TRIGGER_3_10_FPS      0X31
//#define TZTEK_CPLD_CAMERA_TRIGGER_3_20_FPS      0X32
//#define TZTEK_CPLD_CAMERA_TRIGGER_3_30_FPS      0X33

namespace usb_cam {

static void errno_exit(const char * s)
{
  ROS_ERROR("%s error %d, %s", s, errno, strerror(errno));
  exit(EXIT_FAILURE);
}

static int xioctl(int fd, int request, void * arg)
{
  int r;

  do
    r = ioctl(fd, request, arg);
  while (-1 == r && EINTR == errno);

  return r;
}

#ifdef USE_CUDA
static void printDevProp(cudaDeviceProp devProp)
{
    printf("Major revision number:         %d\n",  devProp.major);
    printf("Minor revision number:         %d\n",  devProp.minor);
    printf("Name:                          %s\n",  devProp.name);
    printf("Total global memory:           %lu\n",  devProp.totalGlobalMem);
    printf("Total shared memory per block: %lu\n",  devProp.sharedMemPerBlock);
    printf("Total registers per block:     %d\n",  devProp.regsPerBlock);
    printf("Warp size:                     %d\n",  devProp.warpSize);
    printf("Maximum memory pitch:          %lu\n",  devProp.memPitch);
    printf("Maximum threads per block:     %d\n",  devProp.maxThreadsPerBlock);
    for (int i = 0; i < 3; ++i)
    printf("Maximum dimension %d of block:  %d\n", i, devProp.maxThreadsDim[i]);
    for (int i = 0; i < 3; ++i)
    printf("Maximum dimension %d of grid:   %d\n", i, devProp.maxGridSize[i]);
    printf("Clock rate:                    %d\n",  devProp.clockRate);
    printf("Total constant memory:         %lu\n",  devProp.totalConstMem);
    printf("Texture alignment:             %lu\n",  devProp.textureAlignment);
    printf("Concurrent copy and execution: %s\n",  (devProp.deviceOverlap ? "Yes" : "No"));
    printf("Number of multiprocessors:     %d\n",  devProp.multiProcessorCount);
    printf("Kernel execution timeout:      %s\n",  (devProp.kernelExecTimeoutEnabled ? "Yes" : "No"));
    return;
}
#endif

UsbCam::UsbCam()
  : io_(IO_METHOD_MMAP), fd_(-1), fd_trigger_(-1), fd_trigger_event_(-1), buffers_(NULL), n_buffers_(0), avframe_camera_(NULL),
    avframe_rgb_(NULL), avcodec_(NULL), avoptions_(NULL), avcodec_context_(NULL),
    avframe_camera_size_(0), avframe_rgb_size_(0), video_sws_(NULL), image_(NULL), is_capturing_(false) {
#ifdef USE_CUDA
    image_yuyv_cuda_ = NULL;
    image_rgb_cuda_ = NULL;
#endif
}
UsbCam::~UsbCam()
{
  shutdown();
}

int UsbCam::init_mjpeg_decoder(int image_width, int image_height)
{
  avcodec_register_all();

  avcodec_ = avcodec_find_decoder(AV_CODEC_ID_MJPEG);
  if (!avcodec_)
  {
    ROS_ERROR("Could not find MJPEG decoder");
    return 0;
  }

  avcodec_context_ = avcodec_alloc_context3(avcodec_);
#if LIBAVCODEC_VERSION_MAJOR < 55
  avframe_camera_ = avcodec_alloc_frame();
  avframe_rgb_ = avcodec_alloc_frame();
#else
  avframe_camera_ = av_frame_alloc();
  avframe_rgb_ = av_frame_alloc();
#endif

  avpicture_alloc((AVPicture *)avframe_rgb_, AV_PIX_FMT_RGB24, image_width, image_height);

  avcodec_context_->codec_id = AV_CODEC_ID_MJPEG;
  avcodec_context_->width = image_width;
  avcodec_context_->height = image_height;

#if LIBAVCODEC_VERSION_MAJOR > 52
  avcodec_context_->pix_fmt = AV_PIX_FMT_YUV422P;
  avcodec_context_->codec_type = AVMEDIA_TYPE_VIDEO;
#endif

  avframe_camera_size_ = avpicture_get_size(AV_PIX_FMT_YUV422P, image_width, image_height);
  avframe_rgb_size_ = avpicture_get_size(AV_PIX_FMT_RGB24, image_width, image_height);

  /* open it */
  if (avcodec_open2(avcodec_context_, avcodec_, &avoptions_) < 0)
  {
    ROS_ERROR("Could not open MJPEG Decoder");
    return 0;
  }
  return 1;
}

void UsbCam::mjpeg2rgb(char *MJPEG, int len, char *RGB, int NumPixels)
{
  int got_picture;

  memset(RGB, 0, avframe_rgb_size_);

#if LIBAVCODEC_VERSION_MAJOR > 52
  int decoded_len;
  AVPacket avpkt;
  av_init_packet(&avpkt);

  avpkt.size = len;
  avpkt.data = (unsigned char*)MJPEG;
  decoded_len = avcodec_decode_video2(avcodec_context_, avframe_camera_, &got_picture, &avpkt);

  if (decoded_len < 0)
  {
    ROS_ERROR("Error while decoding frame.");
    return;
  }
#else
  avcodec_decode_video(avcodec_context_, avframe_camera_, &got_picture, (uint8_t *) MJPEG, len);
#endif

  if (!got_picture)
  {
    ROS_ERROR("Webcam: expected picture but didn't get it...");
    return;
  }

  int xsize = avcodec_context_->width;
  int ysize = avcodec_context_->height;
  int pic_size = avpicture_get_size(avcodec_context_->pix_fmt, xsize, ysize);
  if (pic_size != avframe_camera_size_)
  {
    ROS_ERROR("outbuf size mismatch.  pic_size: %d bufsize: %d", pic_size, avframe_camera_size_);
    return;
  }

  video_sws_ = sws_getContext(xsize, ysize, avcodec_context_->pix_fmt, xsize, ysize, AV_PIX_FMT_RGB24, SWS_BILINEAR, NULL,
			      NULL,  NULL);
  sws_scale(video_sws_, avframe_camera_->data, avframe_camera_->linesize, 0, ysize, avframe_rgb_->data,
            avframe_rgb_->linesize);
  sws_freeContext(video_sws_);

  int size = avpicture_layout((AVPicture *)avframe_rgb_, AV_PIX_FMT_RGB24, xsize, ysize, (uint8_t *)RGB, avframe_rgb_size_);
  if (size != avframe_rgb_size_)
  {
    ROS_ERROR("webcam: avpicture_layout error: %d", size);
    return;
  }
}

void UsbCam::process_image(const void * src, int len, camera_image_t *dest)
{
  if (pixelformat_ == V4L2_PIX_FMT_YUYV)
  {
    if (monochrome_)
    { //actually format V4L2_PIX_FMT_Y16, but xioctl gets unhappy if you don't use the advertised type (yuyv)
      mono102mono8((char*)src, dest->image, dest->width * dest->height);
    }
    else
    {
      yuyv2rgb((char*)src, dest->image, dest->width * dest->height);
    }
  }
  else if (pixelformat_ == V4L2_PIX_FMT_UYVY)
    uyvy2rgb((char*)src, dest->image, dest->width * dest->height);
  else if (pixelformat_ == V4L2_PIX_FMT_MJPEG)
    mjpeg2rgb((char*)src, len, dest->image, dest->width * dest->height);
  else if (pixelformat_ == V4L2_PIX_FMT_RGB24)
    rgb242rgb((char*)src, dest->image, dest->width * dest->height);
  else if (pixelformat_ == V4L2_PIX_FMT_GREY)
    memcpy(dest->image, (char*)src, dest->width * dest->height);
}

#ifdef USE_CUDA
void UsbCam::process_image_cuda(const void* src, int len, camera_image_t *dest) {

    if (V4L2_PIX_FMT_YUYV == pixelformat_) {
        int yuv_size = dest->width * dest->height * 2 * sizeof(char);
        cudaError_t ret = cudaMemcpy(image_yuyv_cuda_,  src,  yuv_size, cudaMemcpyHostToDevice);
        if (cudaSuccess != ret) {
            ROS_ERROR("cudaMemcpy fail %d\n", ret);
            exit(EXIT_FAILURE);
        }
        const int N = yuv_size / 4;
        const int block_size = 256;   // times of warp size and less than total threads per block
        const int num_blocks = (N + block_size - 1) / block_size;
//        ROS_INFO("kernel_num_blocks: %d\n", num_blocks);
//        ROS_INFO("kernel_block_size: %d\n", block_size);
        yuyv2rgb_cuda(image_yuyv_cuda_, image_rgb_cuda_, dest->width * dest->height ,num_blocks, block_size);
        int rgb_size = dest->width * dest->height * 3 * sizeof(char);
        ret = cudaMemcpy(dest->image, image_rgb_cuda_, rgb_size, cudaMemcpyDeviceToHost);
        if (cudaSuccess != ret) {
            ROS_ERROR("cudaMemcpy fail %d\n", ret);
            exit(EXIT_FAILURE);
        }
    } else {
        ROS_ERROR("color conversion not supported on cuda\n");
        exit(EXIT_FAILURE);
    }
}
#endif

int UsbCam::read_frame()
{
  struct v4l2_buffer buf;
  unsigned int i;
  int len;

  switch (io_)
  {
    case IO_METHOD_READ:
      len = read(fd_, buffers_[0].start, buffers_[0].length);
      if (len == -1)
      {
        switch (errno)
        {
          case EAGAIN:
            return 0;

          case EIO:
            /* Could ignore EIO, see spec. */

            /* fall through */

          default:
            errno_exit("read");
        }
      }

      image_->stamp.sec  = ros::Time::now().sec;
      image_->stamp.nsec = ros::Time::now().nsec;
      process_image(buffers_[0].start, len, image_);

      break;

    case IO_METHOD_MMAP:
      CLEAR(buf);

      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;

      if (-1 == xioctl(fd_, VIDIOC_DQBUF, &buf))
      {
        switch (errno)
        {
          case EAGAIN:
            return 0;

          case EIO:
            /* Could ignore EIO, see spec. */

            /* fall through */

          default:
            ROS_ERROR("VIDIOC_DQBUF Error");
            return 0;
        }
      }

      assert(buf.index < n_buffers_);
      len = buf.bytesused;

      image_->stamp.sec  = ros::Time::now().sec;
      image_->stamp.nsec = ros::Time::now().nsec;


      struct timeval t_start, t_end;
      long long start_time, end_time;
      gettimeofday(&t_start, NULL);
      start_time = (long long) t_start.tv_sec * 1e3 + t_start.tv_usec * 1e-3;
#ifdef USE_CUDA
      process_image_cuda(buffers_[buf.index].start, len, image_);
#else
      process_image(buffers_[buf.index].start, len, image_);
#endif
      gettimeofday(&t_end, NULL);
      end_time = (long long) t_end.tv_sec * 1e3 + t_end.tv_usec * 1e-3;
      ROS_INFO("color conversion time duration: %ld ms\n", abs(end_time - start_time));

      if (-1 == xioctl(fd_, VIDIOC_QBUF, &buf)) {
        ROS_ERROR("VIDIOC_QBUF");
	return 0;
      }

      break;

    case IO_METHOD_USERPTR:
      CLEAR(buf);

      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_USERPTR;

      if (-1 == xioctl(fd_, VIDIOC_DQBUF, &buf))
      {
        switch (errno)
        {
          case EAGAIN:
            return 0;

          case EIO:
            /* Could ignore EIO, see spec. */

            /* fall through */

          default:
            errno_exit("VIDIOC_DQBUF");
        }
      }

      for (i = 0; i < n_buffers_; ++i)
        if (buf.m.userptr == (unsigned long)buffers_[i].start && buf.length == buffers_[i].length)
          break;

      assert(i < n_buffers_);
      len = buf.bytesused;

      image_->stamp.sec  = ros::Time::now().sec;
      image_->stamp.nsec = ros::Time::now().nsec;
      process_image((void *)buf.m.userptr, len, image_);

      if (-1 == xioctl(fd_, VIDIOC_QBUF, &buf))
        errno_exit("VIDIOC_QBUF");

      break;
  }

  return 1;
}

bool UsbCam::is_capturing() {
  return is_capturing_;
}

void UsbCam::stop_capturing(void)
{
  if(!is_capturing_) return;

  is_capturing_ = false;
  enum v4l2_buf_type type;

  switch (io_)
  {
    case IO_METHOD_READ:
      /* Nothing to do. */
      break;

    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      if (-1 == xioctl(fd_, VIDIOC_STREAMOFF, &type))
        //errno_exit("VIDIOC_STREAMOFF");
        return;

      break;
  }
}

void UsbCam::start_capturing(void)
{

  if(is_capturing_) return;

  unsigned int i;
  enum v4l2_buf_type type;

  switch (io_)
  {
    case IO_METHOD_READ:
      /* Nothing to do. */
      break;

    case IO_METHOD_MMAP:
      for (i = 0; i < n_buffers_; ++i)
      {
        struct v4l2_buffer buf;

        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (-1 == xioctl(fd_, VIDIOC_QBUF, &buf))
          errno_exit("VIDIOC_QBUF");
      }

      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      if (-1 == xioctl(fd_, VIDIOC_STREAMON, &type))
        errno_exit("VIDIOC_STREAMON");

      break;

    case IO_METHOD_USERPTR:
      for (i = 0; i < n_buffers_; ++i)
      {
        struct v4l2_buffer buf;

        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_USERPTR;
        buf.index = i;
        buf.m.userptr = (unsigned long)buffers_[i].start;
        buf.length = buffers_[i].length;

        if (-1 == xioctl(fd_, VIDIOC_QBUF, &buf))
          errno_exit("VIDIOC_QBUF");
      }

      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      if (-1 == xioctl(fd_, VIDIOC_STREAMON, &type))
        errno_exit("VIDIOC_STREAMON");

      break;
  }
  is_capturing_ = true;
}

void UsbCam::uninit_device(void)
{
  unsigned int i;

  switch (io_)
  {
    case IO_METHOD_READ:
      free(buffers_[0].start);
      break;

    case IO_METHOD_MMAP:
      for (i = 0; i < n_buffers_; ++i)
        if (-1 == munmap(buffers_[i].start, buffers_[i].length))
          errno_exit("munmap");
      break;

    case IO_METHOD_USERPTR:
      for (i = 0; i < n_buffers_; ++i)
        free(buffers_[i].start);
      break;
  }

  if (buffers_)
    {
      free(buffers_);
      buffers_ = NULL;
    }

//  if (-1 == ioctl(fd_trigger_, TZTEK_CPLD_CAMERA_TRIGGER_ALL_STOP, NULL)) {
//    errno_exit("set trigger mode all stop");
//  }
//  usleep(50000);
}

void UsbCam::init_read(unsigned int buffer_size)
{
  buffers_ = (buffer*)calloc(1, sizeof(*buffers_));

  if (!buffers_)
  {
    ROS_ERROR("Out of memory");
    exit(EXIT_FAILURE);
  }

  buffers_[0].length = buffer_size;
  buffers_[0].start = malloc(buffer_size);

  if (!buffers_[0].start)
  {
    ROS_ERROR("Out of memory");
    exit(EXIT_FAILURE);
  }
}

bool UsbCam::init_mmap(void)
{
  struct v4l2_requestbuffers req;

  CLEAR(req);

  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (-1 == xioctl(fd_, VIDIOC_REQBUFS, &req))
  {
    if (EINVAL == errno)
    {
      ROS_ERROR_STREAM(camera_dev_ << " does not support memory mapping");
      exit(EXIT_FAILURE);
    }
    else
    {
      ROS_ERROR("VIDIOC_REQBUFS");
      return false;
    }
  }

  if (req.count < 2)
  {
    ROS_ERROR_STREAM("Insufficient buffer memory on " << camera_dev_);
    exit(EXIT_FAILURE);
  }

  buffers_ = (buffer*)calloc(req.count, sizeof(*buffers_));

  if (!buffers_)
  {
    ROS_ERROR("Out of memory");
    exit(EXIT_FAILURE);
  }

  for (n_buffers_ = 0; n_buffers_ < req.count; ++n_buffers_)
  {
    struct v4l2_buffer buf;

    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = n_buffers_;

    if (-1 == xioctl(fd_, VIDIOC_QUERYBUF, &buf))
      errno_exit("VIDIOC_QUERYBUF");

    buffers_[n_buffers_].length = buf.length;
    buffers_[n_buffers_].start = mmap(NULL /* start anywhere */, buf.length, PROT_READ | PROT_WRITE /* required */,
				      MAP_SHARED /* recommended */,
				      fd_, buf.m.offset);

    if (MAP_FAILED == buffers_[n_buffers_].start)
      errno_exit("mmap");
  }
  return true;
}

void UsbCam::init_userp(unsigned int buffer_size)
{
  struct v4l2_requestbuffers req;
  unsigned int page_size;

  page_size = getpagesize();
  buffer_size = (buffer_size + page_size - 1) & ~(page_size - 1);

  CLEAR(req);

  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_USERPTR;

  if (-1 == xioctl(fd_, VIDIOC_REQBUFS, &req))
  {
    if (EINVAL == errno)
    {
      ROS_ERROR_STREAM(camera_dev_ << " does not support "
                "user pointer i/o");
      exit(EXIT_FAILURE);
    }
    else
    {
      errno_exit("VIDIOC_REQBUFS");
    }
  }

  buffers_ = (buffer*)calloc(4, sizeof(*buffers_));

  if (!buffers_)
  {
    ROS_ERROR("Out of memory");
    exit(EXIT_FAILURE);
  }

  for (n_buffers_ = 0; n_buffers_ < 4; ++n_buffers_)
  {
    buffers_[n_buffers_].length = buffer_size;
    buffers_[n_buffers_].start = memalign(/* boundary */page_size, buffer_size);

    if (!buffers_[n_buffers_].start)
    {
      ROS_ERROR("Out of memory");
      exit(EXIT_FAILURE);
    }
  }
}

bool UsbCam::init_device(int image_width, int image_height, int framerate)
{
  struct v4l2_capability cap;
  struct v4l2_cropcap cropcap;
  struct v4l2_crop crop;
  struct v4l2_format fmt;
  unsigned int min;

  if (-1 == xioctl(fd_, VIDIOC_QUERYCAP, &cap))
  {
    if (EINVAL == errno)
    {
      ROS_ERROR_STREAM(camera_dev_ << " is no V4L2 device");
      exit(EXIT_FAILURE);
    }
    else
    {
      errno_exit("VIDIOC_QUERYCAP");
    }
  }

  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
  {
    ROS_ERROR_STREAM(camera_dev_ << " is no video capture device");
    exit(EXIT_FAILURE);
  }

  switch (io_)
  {
    case IO_METHOD_READ:
      if (!(cap.capabilities & V4L2_CAP_READWRITE))
      {
        ROS_ERROR_STREAM(camera_dev_ << " does not support read i/o");
        exit(EXIT_FAILURE);
      }

      break;

    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
      if (!(cap.capabilities & V4L2_CAP_STREAMING))
      {
        ROS_ERROR_STREAM(camera_dev_ << " does not support streaming i/o");
        exit(EXIT_FAILURE);
      }

      break;
  }

  /* Select video input, video standard and tune here. */

  CLEAR(cropcap);

  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (0 == xioctl(fd_, VIDIOC_CROPCAP, &cropcap))
  {
    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    crop.c = cropcap.defrect; /* reset to default */

    if (-1 == xioctl(fd_, VIDIOC_S_CROP, &crop))
    {
      switch (errno)
      {
        case EINVAL:
          /* Cropping not supported. */
          break;
        default:
          /* Errors ignored. */
          break;
      }
    }
  }
  else
  {
    /* Errors ignored. */
  }

  CLEAR(fmt);

//  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
//  fmt.fmt.pix.width = 640;
//  fmt.fmt.pix.height = 480;
//  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
//  fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = image_width;
  fmt.fmt.pix.height = image_height;
  fmt.fmt.pix.pixelformat = pixelformat_;
  fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

  /* Note VIDIOC_S_FMT may change width and height. */

  /* Buggy driver paranoia. */
  min = fmt.fmt.pix.width * 2;
  if (fmt.fmt.pix.bytesperline < min)
    fmt.fmt.pix.bytesperline = min;
  min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
  if (fmt.fmt.pix.sizeimage < min)
    fmt.fmt.pix.sizeimage = min;

  image_width = fmt.fmt.pix.width;
  image_height = fmt.fmt.pix.height;

  if (-1 == xioctl(fd_, VIDIOC_S_FMT, &fmt))
    errno_exit("VIDIOC_S_FMT");

  //trigger enable
//  if (-1 == ioctl(fd_trigger_, TZTEK_CPLD_CAMERA_TRIGGER_1_STOP, NULL)) {
//    errno_exit("set trigger mode 1 stop");
//  }
//  usleep(50000);
//  if (-1 == ioctl(fd_trigger_, TZTEK_CPLD_CAMERA_TRIGGER_1_20_FPS, NULL)) {
//    errno_exit("set trigger 1 frame rate");
//  }
//  usleep(1000);
//  if (-1 == ioctl(fd_trigger_, TZTEK_CPLD_CAMERA_TRIGGER_1_START, NULL)) {
//    errno_exit("set trigger 1 mode start");
//  }


  //struct v4l2_streamparm stream_params;
  //memset(&stream_params, 0, sizeof(stream_params));
  //stream_params.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  //if (xioctl(fd_, VIDIOC_G_PARM, &stream_params) < 0)
  //  errno_exit("Couldn't query v4l fps!");

  //ROS_DEBUG("Capability flag: 0x%x", stream_params.parm.capture.capability);

  //stream_params.parm.capture.timeperframe.numerator = 1;
  //stream_params.parm.capture.timeperframe.denominator = framerate;
  //if (xioctl(fd_, VIDIOC_S_PARM, &stream_params) < 0)
  //  ROS_WARN("Couldn't set camera framerate");
  //else
  //  ROS_DEBUG("Set framerate to be %i", framerate);

  switch (io_)
  {
    case IO_METHOD_READ:
      init_read(fmt.fmt.pix.sizeimage);
      break;

    case IO_METHOD_MMAP:
      return init_mmap();
      break;

    case IO_METHOD_USERPTR:
      init_userp(fmt.fmt.pix.sizeimage);
      break;
  }
  return true;
}

void UsbCam::close_device(void)
{
  if (-1 == close(fd_))
    errno_exit("close");
//  if (-1 == close(fd_trigger_))
//    errno_exit("close");
  if (-1 == close(fd_trigger_event_))
    errno_exit("close");
  fd_ = -1;
  fd_trigger_ = -1;
  fd_trigger_event_ = -1;
}

void UsbCam::open_device(void)
{
  struct stat st;

  if (-1 == stat(camera_dev_.c_str(), &st))
  {
    ROS_ERROR_STREAM("Cannot identify '" << camera_dev_ << "': " << errno << ", " << strerror(errno));
    exit(EXIT_FAILURE);
  }

  if (!S_ISCHR(st.st_mode))
  {
    ROS_ERROR_STREAM(camera_dev_ << " is no device");
    exit(EXIT_FAILURE);
  }

  fd_ = open(camera_dev_.c_str(), O_RDWR /* required */| O_NONBLOCK, 0);

  if (-1 == fd_)
  {
    ROS_ERROR_STREAM("Cannot open '" << camera_dev_ << "': " << errno << ", " << strerror(errno));
    exit(EXIT_FAILURE);
  }

  //TODO, option to choose usb or tz
//  fd_trigger_ = open("/dev/tztek_i2c_cpld_enable_trigger", O_RDWR /* required */ /*| O_NONBLOCK*/, 0);
//  if (-1 == fd_trigger_)
//  {
//    ROS_ERROR_STREAM("Cannot open tztek_i2c_cpld_enable_trigger'" << "': " << errno << ", " << strerror(errno));
//    exit(EXIT_FAILURE);
//  }
  fd_trigger_event_ = open("/dev/camera_trigger", O_RDWR);
  if (-1 == fd_trigger_event_)
  {
    ROS_ERROR_STREAM("Cannot open camera_trigger'" << "': " << errno << ", " << strerror(errno));
    exit(EXIT_FAILURE);
  }
}

#ifdef USE_CUDA
void UsbCam::query_cuda_device(void)
{
    // Number of CUDA devices
    int devCount = 0;
    cudaGetDeviceCount(&devCount);
    printf("CUDA Device Query...\n");
    printf("There are %d CUDA devices.\n", devCount);

    if (0 == devCount) 
        errno_exit("No cuda device found\n");
 
    // Iterate through devices
    for (int i = 0; i < devCount; ++i)
    {
        // Get device properties
        printf("\nCUDA Device #%d\n", i);
        cudaDeviceProp devProp;
        cudaGetDeviceProperties(&devProp, i);
        printDevProp(devProp);
    }
}
#endif

bool UsbCam::start(const std::string& dev, io_method io_method,
		   pixel_format pixel_format, int image_width, int image_height,
		   int framerate)
{
  camera_dev_ = dev;

  io_ = io_method;
  monochrome_ = false;
  if (pixel_format == PIXEL_FORMAT_YUYV)
    pixelformat_ = V4L2_PIX_FMT_YUYV;
  else if (pixel_format == PIXEL_FORMAT_UYVY)
    pixelformat_ = V4L2_PIX_FMT_UYVY;
  else if (pixel_format == PIXEL_FORMAT_MJPEG)
  {
    pixelformat_ = V4L2_PIX_FMT_MJPEG;
    init_mjpeg_decoder(image_width, image_height);
  }
  else if (pixel_format == PIXEL_FORMAT_YUVMONO10)
  {
    //actually format V4L2_PIX_FMT_Y16 (10-bit mono expresed as 16-bit pixels), but we need to use the advertised type (yuyv)
    pixelformat_ = V4L2_PIX_FMT_YUYV;
    monochrome_ = true;
  }
  else if (pixel_format == PIXEL_FORMAT_RGB24)
  {
    pixelformat_ = V4L2_PIX_FMT_RGB24;
  }
  else if (pixel_format == PIXEL_FORMAT_GREY)
  {
    pixelformat_ = V4L2_PIX_FMT_GREY;
    monochrome_ = true;
  }
  else
  {
    ROS_ERROR("Unknown pixel format.");
    exit(EXIT_FAILURE);
  }

  open_device();
  if (!init_device(image_width, image_height, framerate)) { 
	ROS_ERROR("fail to init device");
	is_capturing_ = false;
	return false;
  }
#ifdef USE_CUDA
  query_cuda_device();
#endif
  start_capturing();

  image_ = (camera_image_t *)calloc(1, sizeof(camera_image_t));

  image_->width = image_width;
  image_->height = image_height;
  image_->bytes_per_pixel = 3;      //corrected 11/10/15 (BYTES not BITS per pixel)

  image_->image_size = image_->width * image_->height * image_->bytes_per_pixel;
  image_->is_new = 0;
  image_->image = (char *)calloc(image_->image_size, sizeof(char));
  memset(image_->image, 0, image_->image_size * sizeof(char));
  image_->stamp.sec = 0;
  image_->stamp.nsec = 0;

#ifdef USE_CUDA
  //Allocate cuda memory for device
  int yuyv_image_size = image_->image_size * 2 / 3;
  cudaError_t ret = cudaMalloc((void **)&image_yuyv_cuda_,  yuyv_image_size * sizeof(char));
  if (cudaSuccess != ret) {
    ROS_ERROR("Fail to allocate cuda memory %d\n", ret);
    exit(EXIT_FAILURE);
  }
  ret = cudaMemset((void *)image_yuyv_cuda_, 0, yuyv_image_size * sizeof(char));
  if (cudaSuccess != ret) {
    ROS_ERROR("Fail to set cuda memory %d\n", ret);
    exit(EXIT_FAILURE);
  }
  ret = cudaMalloc((void **)&image_rgb_cuda_,  image_->image_size * sizeof(char));
  if (cudaSuccess != ret) {
    ROS_ERROR("Fail to allocate cuda memory %d\n", ret);
    exit(EXIT_FAILURE);
  }
  ret = cudaMemset((void *)image_rgb_cuda_, 0, image_->image_size * sizeof(char));
  if (cudaSuccess != ret) {
    ROS_ERROR("Fail to set cuda memory %d\n", ret);
    exit(EXIT_FAILURE);
  }
#endif
}

void UsbCam::shutdown(void)
{
  stop_capturing();
  uninit_device();
  close_device();

  if (avcodec_context_)
  {
    avcodec_close(avcodec_context_);
    av_free(avcodec_context_);
    avcodec_context_ = NULL;
  }
  if (avframe_camera_)
    av_free(avframe_camera_);
  avframe_camera_ = NULL;
  if (avframe_rgb_)
    av_free(avframe_rgb_);
  avframe_rgb_ = NULL;
  if(image_)
      if (image_->image)
          free(image_->image);
    free(image_);
  image_ = NULL;

#ifdef USE_CUDA
  if(image_yuyv_cuda_) {
    cudaFree(image_yuyv_cuda_);
    image_yuyv_cuda_ = NULL;
  }
  if(image_rgb_cuda_) {
    cudaFree(image_rgb_cuda_);
    image_rgb_cuda_ = NULL;
  }
#endif
}

bool UsbCam::grab_image(sensor_msgs::Image* msg)
{
  // grab the image
  bool ret = grab_image();
  if (!ret) return ret;
  // stamp the image
  msg->header.stamp = ros::Time(image_->stamp.sec, image_->stamp.nsec);
  // fill the info
  if (monochrome_)
  {
    fillImage(*msg, "mono8", image_->height, image_->width, image_->width,
        image_->image);
  }
  else
  {
    fillImage(*msg, "rgb8", image_->height, image_->width, 3 * image_->width,
        image_->image);
  }
  return true;
}

bool UsbCam::grab_image()
{
//  fd_set fds;
//  struct timeval tv;
//  int r;
//
//  FD_ZERO(&fds);
//  FD_SET(fd_, &fds);
//
//  /* Timeout. */
//  tv.tv_sec = 5;
//  tv.tv_usec = 0;
//
//  r = select(fd_ + 1, &fds, NULL, NULL, &tv);
//
//  if (-1 == r)
//  {
//    if (EINTR == errno)
//      return false;
//
//    errno_exit("select");
//  }
//
//  if (0 == r)
//  {
//    ROS_ERROR("select timeout");
//    return false;
//  }
//
  struct pollfd fds[1];
  fds[0].fd = fd_trigger_event_;
  fds[0].events = POLLIN;
  if (0 == poll(fds, 1, 1000))
  {
    ROS_ERROR("select timeout");
    is_capturing_ = false;
    return false;
  }

  if (!read_frame()) return false;
  image_->is_new = 1;
  return true;
}

// enables/disables auto focus
void UsbCam::set_auto_focus(int value)
{
  struct v4l2_queryctrl queryctrl;
  struct v4l2_ext_control control;

  memset(&queryctrl, 0, sizeof(queryctrl));
  queryctrl.id = V4L2_CID_FOCUS_AUTO;

  if (-1 == xioctl(fd_, VIDIOC_QUERYCTRL, &queryctrl))
  {
    if (errno != EINVAL)
    {
      perror("VIDIOC_QUERYCTRL");
      return;
    }
    else
    {
      ROS_INFO("V4L2_CID_FOCUS_AUTO is not supported");
      return;
    }
  }
  else if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
  {
    ROS_INFO("V4L2_CID_FOCUS_AUTO is not supported");
    return;
  }
  else
  {
    memset(&control, 0, sizeof(control));
    control.id = V4L2_CID_FOCUS_AUTO;
    control.value = value;

    if (-1 == xioctl(fd_, VIDIOC_S_CTRL, &control))
    {
      perror("VIDIOC_S_CTRL");
      return;
    }
  }
}

/**
* Set video device parameter via call to v4l-utils.
*
* @param param The name of the parameter to set
* @param param The value to assign
*/
void UsbCam::set_v4l_parameter(const std::string& param, int value)
{
  set_v4l_parameter(param, boost::lexical_cast<std::string>(value));
}
/**
* Set video device parameter via call to v4l-utils.
*
* @param param The name of the parameter to set
* @param param The value to assign
*/
void UsbCam::set_v4l_parameter(const std::string& param, const std::string& value)
{
  // build the command
  std::stringstream ss;
  ss << "v4l2-ctl --device=" << camera_dev_ << " -c " << param << "=" << value << " 2>&1";
  std::string cmd = ss.str();

  // capture the output
  std::string output;
  int buffer_size = 256;
  char buffer[buffer_size];
  FILE *stream = popen(cmd.c_str(), "r");
  if (stream)
  {
    while (!feof(stream))
      if (fgets(buffer, buffer_size, stream) != NULL)
        output.append(buffer);
    pclose(stream);
    // any output should be an error
    if (output.length() > 0)
      ROS_WARN("%s", output.c_str());
  }
  else
    ROS_WARN("usb_cam_node could not run '%s'", cmd.c_str());
}

UsbCam::io_method UsbCam::io_method_from_string(const std::string& str)
{
  if (str == "mmap")
    return IO_METHOD_MMAP;
  else if (str == "read")
    return IO_METHOD_READ;
  else if (str == "userptr")
    return IO_METHOD_USERPTR;
  else
    return IO_METHOD_UNKNOWN;
}

UsbCam::pixel_format UsbCam::pixel_format_from_string(const std::string& str)
{
    if (str == "yuyv")
      return PIXEL_FORMAT_YUYV;
    else if (str == "uyvy")
      return PIXEL_FORMAT_UYVY;
    else if (str == "mjpeg")
      return PIXEL_FORMAT_MJPEG;
    else if (str == "yuvmono10")
      return PIXEL_FORMAT_YUVMONO10;
    else if (str == "rgb24")
      return PIXEL_FORMAT_RGB24;
    else if (str == "grey")
      return PIXEL_FORMAT_GREY;
    else
      return PIXEL_FORMAT_UNKNOWN;
}

}
