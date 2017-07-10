/********************************************************
Author: Avinash Reddy , Manoj Sree Harsha
********************************************************/
#include <numeric>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/videoio/videoio.hpp"
#include <iostream>
using namespace cv;
using namespace std;
extern "C"
{
  #include "api_scilab.h"
  #include "Scierror.h"
  #include "BOOL.h"
  #include <localization.h>
  #include "sciprint.h"
  #include "../common.h"

int opencv_opticalflow(char *fname, unsigned long fname_len)
{

    //Error management variables
    SciErr sciErr;

    //variable info
    int intErr      = 0;
    int iLen        = 0;
    int iRows       = 0;
    int iCols       = 0;
    int piRows      = 0;
    int piCols      = 0;  
    int *piAddr     = NULL;      
    int *piLen      = NULL;
    char **pstData  = NULL;
    int **pstData1  = NULL;
    int i,j,k       = 0;

    //Checking number of input and output arguments (enviromnet variable, min arguments, max arguments)
    CheckInputArgument(pvApiCtx, 1, 1);
    CheckOutputArgument(pvApiCtx, 1, 1);
    
    //get Address of inputs
    sciErr = getVarAddressFromPosition(pvApiCtx, 1, &piAddr);
    if (sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }
    //Now, we will retrieve the string from the input parameter. For this, we will require 3 calls
    //first call to retrieve dimensions
    sciErr = getMatrixOfString(pvApiCtx, piAddr, &iRows, &iCols, NULL, NULL);
    if(sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }
    piLen = (int*)malloc(sizeof(int) * iRows * iCols);
    //second call to retrieve length of each string
    sciErr = getMatrixOfString(pvApiCtx, piAddr, &iRows, &iCols, piLen, NULL);
    if(sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }
    pstData = (char**)malloc(sizeof(char*) * iRows * iCols);
    for(i = 0 ; i < iRows * iCols ; i++)
    {
      pstData[i] = (char*)malloc(sizeof(char) * (piLen[i] + 1));
    }
    //third call to retrieve data
    sciErr = getMatrixOfString(pvApiCtx, piAddr, &iRows, &iCols, piLen, pstData);
    if(sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }

    //Application Code
    try
    {
      string filename=pstData[0];
      VideoCapture cap(filename);
      if( !cap.isOpened() )
      {
          return 0;
      }
      Mat   flow,frame;
      UMat  flowUmat, prevgray;
      VideoWriter video("out.avi",CV_FOURCC('M','J','P','G'),10,Size(640,480),true);
      for (;;)
      {
        bool Is = cap.grab();
        if (Is == false) 
        {           
          cout << "Video Capture Fail" << endl;
          break;
        }
        else 
        {
          Mat img;
          Mat original;
          cap.retrieve(img, CV_CAP_OPENNI_BGR_IMAGE);
          resize(img, img, Size(640, 480));
          img.copyTo(original);
          cvtColor(img, img, COLOR_BGR2GRAY);
          if (prevgray.empty() == false ) 
          {
            // calculate optical flow 
            calcOpticalFlowFarneback(prevgray, img, flowUmat, 0.4, 1, 12, 2, 8, 1.2, 0);
            // copy Umat container to standard Mat
            flowUmat.copyTo(flow);                   
            // By y += 5, x += 5 you can specify the grid 
            for (int y = 0; y < original.rows; y += 5) 
            {
                for (int x = 0; x < original.cols; x += 5)
                  {
                      const Point2f flowatxy = flow.at<Point2f>(y, x) * 10;
                      line(original, Point(x, y), Point(cvRound(x + flowatxy.x), cvRound(y + flowatxy.y)), Scalar(0,255,0));// draw initial point
                      circle(original, Point(x, y), 1, Scalar(0, 0, 0), -1);
                  }
            }   
            //writing video               
            video.write(original);
            img.copyTo(prevgray);
           }

           // fill previous image in case prevgray.empty() == true
           else 
           {                       
              img.copyTo(prevgray);
           }
        }

       }
       cap.release();
    }
    catch(cv::Exception& e)
    {
        const char* err=e.what();
        Scierror(999,("%s",err));
    } 
    return 0;
  }
}

