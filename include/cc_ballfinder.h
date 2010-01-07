#ifndef _CC_BALLFINDER_H_
#define _CC_BALLFINDER_H_

#include <cxcore.h>
#include <cv.h>
#include <highgui.h>

const double cc_pi=3.14159265;

struct Ball
{
  int num;
  double* dist;
  double* angle;
};

class BallFinder
{
  public:
    int Init(int width,int height)
    {
      cx=705;
      cy=492;
      min_radius=3;
      this->width=width;
      this->height=height;
      srcImage=cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,3);
      fltImage=cvCreateImage(cvGetSize(srcImage),8,1);
      smImage=cvCreateImage(cvGetSize(srcImage),8,1);
      hsv=cvCreateImage(cvGetSize(srcImage),8,3);
      imageCircles=cvCreateImage(cvGetSize(srcImage),8,3);
      imageBlobs=cvCreateImage(cvGetSize(srcImage),8,1);
      cvNamedWindow("openCVwindow",CV_WINDOW_AUTOSIZE);
      return 0;
    }

    Ball* DetectBall(unsigned char *img)
    {
      CvSeq* contour;
      CvMemStorage* storBlob=cvCreateMemStorage(0);
      CvRect rect;
      Ball* bs;
      int bx,by,br;
      double angle;

      YUV422toBGR(img,srcImage);
      cvCvtColor(srcImage,hsv,CV_BGR2HSV);
      cvInRangeS(hsv,cvScalar(140,140,30,0),cvScalar(171,256,256,0),fltImage);
      cvSmooth(fltImage,smImage,CV_MEDIAN,3,3);
//      cvSmooth(fltImage,smImage,CV_MEDIAN,9,9);
      cvFindContours(smImage,storBlob,&contour,sizeof(CvContour),CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);

      bx=by=br=0;

      for(;contour!=0;contour=contour->h_next)
      {
        rect=((CvContour*)contour)->rect;
        if (rect.width<min_radius||rect.height<min_radius) continue;
        // create an image with only this segment
        cvZero(imageBlobs);
        cvDrawContours(imageBlobs,contour,CV_RGB(255,255,255),CV_RGB(255,255,255),-1,CV_FILLED,8);
        // Hough transform this blob
        CvSeq* circles=cvHoughCircles(imageBlobs,storBlob,CV_HOUGH_GRADIENT,2,imageBlobs->height/4,200,20);
        // if a circle was found
        if (0<circles->total)
        {
          // keep only the largest circle
          float* p=(float*)cvGetSeqElem(circles,0);
          if (p[2]>br)
          {
            bx=p[0];
            by=p[1];
            br=p[2];
          }
        }
        // draw this segment onto the output image with a random color
      }

      bs=new Ball;
      bs->num=1;
      if (br<=0)
      {
        cvShowImage("openCVwindow",srcImage);
        bs->num=0;
        return bs;
      }
      bs->dist=new double[bs->num];
      bs->angle=new double[bs->num];

      cvCircle(srcImage,cvPoint(bx,by),br,CV_RGB(255,0,0),3);
      cvLine(srcImage,cvPoint(cx,cy),cvPoint(bx,by),CV_RGB(255,0,0),3);
      angle=atan2(bx-cx,by-cy);
      //cvEllipse(srcImage,cvPoint(cx,cy),cvSize(20,20),angle/pi*360,270,270-angle/pi*360,CV_RGB(255,0,0),2);
      bs->angle[0]=angle;
      bs->dist[0]=(by-cy)*(by-cy)+(bx-cx)*(bx-cx);

      cvShowImage("openCVwindow",srcImage);
      cvReleaseMemStorage(&storBlob);
      return bs;
    }

    int Over()
    {
      cvDestroyWindow("openCVwindow");
      cvReleaseImage(&fltImage);
      cvReleaseImage(&smImage);
      cvReleaseImage(&hsv);
      cvReleaseImage(&imageCircles);
      cvReleaseImage(&srcImage);
      cvReleaseImage(&imageBlobs);
      return 0;
    }

    bool IsContinue()
    {
      if (cvWaitKey(10)==1048603) return false;
      return true;
    }

  private:
    int width;
    int height;
    IplImage *srcImage;
    IplImage* fltImage;
    IplImage* smImage;
    IplImage* hsv;
    IplImage* imageCircles;
    IplImage* imageBlobs;

    int cx; // the x center of omni-image
    int cy; // the y center of omni-image
    int min_radius; // the ball smaller than this size will be ignored

    void YUV422toBGR(unsigned char *src,IplImage *img)
    {
      int i,j,k,u,y1,v,y2;
      // Top-Bottom, Left-Right
      for (i=0;i<height;i++)
      {
        for (j=0;j<width;j+=2)
        {
          k=(i*width+j);
          v=src[k*2];
          y1=src[k*2+1];
          u=src[k*2+2];
          y2=src[k*2+3];
          img->imageData[k*3]=y1;
          img->imageData[k*3+1]=u;
          img->imageData[k*3+2]=v;
          img->imageData[k*3+3]=y2;
          img->imageData[k*3+4]=u;
          img->imageData[k*3+5]=v;
        }
      }
      cvCvtColor(img,img,CV_YCrCb2BGR);
    }
};

#endif

