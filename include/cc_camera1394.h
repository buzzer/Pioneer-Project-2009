#ifndef _CC_CAMERA1394_H_
#define _CC_CAMERA1394_H_

#include <libraw1394/raw1394.h>
#include <libdc1394/dc1394_control.h>

typedef enum
{
  AF_off=0,
  AF_global,
  AF_locIni,
  AF_local
} focusModeTy;

typedef enum
{
  low=0,
  mid,
  high
} focusITy;

typedef struct
{
  int val,sum;
} focusTy;

#define FOCUS_MIN 0
#define FOCUS_MAX 447

class Single1394
{
  public:
    unsigned char* captureBuf;
    int initCam(int width,int height)
    {
      int fwNodeNum,fwCamNum;
      nodeid_t *fwCamNodes;
      unsigned int speed;

      this->width=width;
      this->height=height;
      imagelen=width*height*2;
      captureBuf=(unsigned char *)malloc(imagelen);
      // Create DC1394 handle
      if ((fwHandle=dc1394_create_handle(1))==NULL)
      {
        printf("-E- failed to get raw1394 handle\n");
        return 0;
      }
      // Get DC1394 node
      fwNodeNum=raw1394_get_nodecount(fwHandle);
printf("%x   %d nodes\n",fwCamNodes,fwNodeNum);
      fwCamNodes=dc1394_get_camera_nodes(fwHandle,&fwCamNum,1);
printf("%x:%d",fwCamNodes,fwCamNum);
      if (fwCamNum<1)
      {
        printf("-E- no camera found\n");
        dc1394_free_camera_nodes(fwCamNodes);
        dc1394_destroy_handle(fwHandle);
        return 0;
      }
      if (fwCamNum>1)
      {
        printf("-E- too many cameras: %i\n", fwCamNum);
        dc1394_free_camera_nodes(fwCamNodes);
        dc1394_destroy_handle(fwHandle);
        return 0;
      }
      fwCamera.node = fwCamNodes[0];
      // Get iso channel
      if (dc1394_get_iso_channel_and_speed(fwHandle,fwCamera.node,(unsigned int *)&fwCamera.channel,(unsigned int *)&speed)!=DC1394_SUCCESS)
      {
        printf("-E- unable to get the iso channel number\n");
        dc1394_free_camera_nodes(fwCamNodes);
        dc1394_destroy_handle(fwHandle);
        return 0;
      }
#ifdef INFO
      // Get feature set
      if (dc1394_get_camera_feature_set(fwHandle,fwCamera.node,&camFeatures)!=DC1394_SUCCESS) printf("-W- unable to get camera feature set\n");
      else dc1394_print_feature_set(&camFeatures);
#endif
      // Camera dma setup
      fwCamera.num_dma_buffers=1;
      fwCamera.drop_frames=0;
      fwCamera.dma_device_file=NULL;
      if (width!=1280 || height!=960) printf("Currently the resolution is only supported by 1280x960!\nBe careful...\n");
      if (dc1394_dma_setup_capture(
	fwHandle,
	fwCamera.node,
	fwCamera.channel,
	FORMAT_SVGA_NONCOMPRESSED_2,
	MODE_1280x960_YUV422,SPEED_400,
	FRAMERATE_7_5,
	fwCamera.num_dma_buffers,
	fwCamera.drop_frames,
	fwCamera.dma_device_file,
	&fwCamera)!=DC1394_SUCCESS) 
      {
        printf("-E- unable to setup camera; check line %d of %s\n",__LINE__,__FILE__);
        dc1394_free_camera_nodes(fwCamNodes);
        dc1394_destroy_handle(fwHandle);
        return 0;
      }
      // Start DC1394 iso trasmission
      if (dc1394_start_iso_transmission(fwHandle,fwCamera.node)!=DC1394_SUCCESS)
      {
        printf("-E- unable to start camera iso transmission\n");
        dc1394_free_camera_nodes(fwCamNodes);
        return 0;
      }
      dc1394_free_camera_nodes(fwCamNodes);
      return 1;
    }

    void initFocus()
    {
      int focRange;
      printf("-I- AF: on     - global");
      focusCnt=0;
      focusMode=AF_global;
      focusI=low;
      focusLo=FOCUS_MIN;
      focusHi=FOCUS_MAX;
      focRange=focusHi-focusLo+1;
      focus[low].val=focRange/4;
      focus[mid].val=focRange/2;
      focus[high].val=focRange*3/4;
      dc1394_set_focus(fwHandle, fwCamera.node, focus[high].val); // now the focus is set to MID
      printf("\n");
    }

    void cleanup()
    {
      free(captureBuf);

      dc1394_stop_iso_transmission(fwHandle,fwCamera.node);
      dc1394_dma_unlisten(fwHandle,&fwCamera);
      dc1394_dma_release_camera(fwHandle,&fwCamera);
      dc1394_destroy_handle(fwHandle);
    }

    int captureImage()
    {
      dc1394_dma_single_capture_poll(&fwCamera);
      dc1394_dma_done_with_buffer(&fwCamera);
      memcpy(captureBuf,fwCamera.capture_buffer,imagelen);
      return 1;
    }

  private:
    raw1394handle_t fwHandle;
    dc1394_cameracapture fwCamera;

    focusTy focus[3];
    unsigned int focusCnt;
    focusModeTy focusMode;
    focusITy focusI;
    int focusLo,focusHi;

    int width;
    int height;
    int imagelen;
};

#endif
