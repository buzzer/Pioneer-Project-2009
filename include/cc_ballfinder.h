#ifndef _CC_BALLFINDER_H_
#define _CC_BALLFINDER_H_

//#define _DEBUG

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
      lm=lp=0;

      cx=705;
      cy=490;
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
#ifdef _DEBUG
      cvNamedWindow("DEBUG1",CV_WINDOW_AUTOSIZE);
#endif
      return 0;
    }

    Ball* DetectBall(unsigned char *img)
    {
      CvSeq* contour;
      CvMemStorage* storBlob=cvCreateMemStorage(0);
      CvRect rect;
      Ball* bs;
      int i,j;
      int bx,by,br;
      int mx,my,mr,ms,tx,ty;
      double angle;

      YUV422toBGR(img,srcImage);
      cvCvtColor(srcImage,hsv,CV_BGR2HSV);
      cvInRangeS(hsv,cvScalar(140,140,30,0),cvScalar(171,256,256,0),fltImage);
#ifdef _DEBUG
      cvShowImage("DEBUG1",fltImage);
#endif
      cvSmooth(fltImage,smImage,CV_MEDIAN,3,3);
//      cvSmooth(fltImage,smImage,CV_MEDIAN,9,9);
      cvFindContours(smImage,storBlob,&contour,sizeof(CvContour),CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);

      bx=by=br=0;
      mx=my=mr=0;
      ms=64;

      for(;contour!=0;contour=contour->h_next)
      {
        rect=((CvContour*)contour)->rect;
        if (rect.width<min_radius||rect.height<min_radius) continue;
        if (abs(rect.width-rect.height)<3 && rect.width*rect.height>ms)
        {
          tx=rect.x+rect.width/2;
          ty=rect.y+rect.height/2;
          if (CircleInRange(tx,ty))
          {
            mx=tx;
            my=ty;
            ms=rect.width*rect.height;
            mr=rect.width;
            if (rect.height<mr) mr=rect.height;
          }
        }
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
      if (!CircleInRange(bx,by)) bx=by=br=0;
      // Ball tracks recording
      if (br>0)
        BallTrackRecord(bx,by);
      else
      {
        // Lost ball judge
        bx=mx;
        by=my;
        br=mr;
        j=0;
        if (lm>=10)
        {
          for (i=0;i<10;i++) j+=(bx-lx[i])*(bx-lx[i])+(by-ly[i])*(by-ly[i]);
          //printf("\nCompare error=%d\n",j);
          if (j>2000000) bx=by=br=0;
          else j=0; // ok, here is just for convinient
        }
        if (j==0) BallTrackRecord(bx,by);
      }

      bs=new Ball;
      if (br>0)
      {
        bs->num=1;
        bs->dist=new double[bs->num];
        bs->angle=new double[bs->num];

//        cvCircle(srcImage,cvPoint(cx,cy),90,CV_RGB(0,255,255),1); // Inner circle
//        cvCircle(srcImage,cvPoint(cx,cy),470,CV_RGB(0,255,255),1); // Outer circle
        cvCircle(srcImage,cvPoint(bx,by),br,CV_RGB(255,0,0),3);
        cvLine(srcImage,cvPoint(cx,cy),cvPoint(bx,by),CV_RGB(255,0,0),3);
        angle=atan2(bx-cx,by-cy);
        //cvEllipse(srcImage,cvPoint(cx,cy),cvSize(20,20),angle/pi*360,270,270-angle/pi*360,CV_RGB(255,0,0),2);
        bs->angle[0]=-angle;
        bs->dist[0]=((by-cy)*(by-cy)+(bx-cx)*(bx-cx))*0.00075;
      }
      else bs->num=0;
      cvShowImage("openCVwindow",srcImage);
      cvReleaseMemStorage(&storBlob);
      return bs;
    }

    int Over()
    {
#ifdef _DEBUG
      cvDestroyWindow("DEBUG1");
#endif
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
#ifdef _DEBUG
    IplImage* dbImage;
#endif
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

    int lx[10],ly[10]; // trace of the ball in last 10 points
    int lp; // pointer to current overwrite position in *lx
    int lm; // point num in *lx

    void BallTrackRecord(int x,int y)
    {
      lx[lp]=x;
      ly[lp]=y;
      lp++;
      if (lp>9) lp=0;
      lm++;
      if (lm>10) lp=10;
    }

    bool CircleInRange(int x,int y)
    {
      int r=(x-cx)*(x-cx)+(y-cy)*(y-cy);
      if (r>95*95 && r<460*460) return true;
      return false;
    }

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

