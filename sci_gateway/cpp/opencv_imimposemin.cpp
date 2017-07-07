/********************************************************
Author: Vinay Bhat
********************************************************
Usage: return_image = imimposemin(mask, marker)
********************************************************/

#include <numeric>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
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

  void imimposemin_imreconstruct(Mat, Mat, Mat&);
  
  int opencv_imimposemin(char *fname, unsigned long fname_len)
  {

    SciErr sciErr;
    int intErr = 0;
    int *piAddr = NULL;
    int i, j;

    //checking input argument
    CheckInputArgument(pvApiCtx, 2, 2);
    CheckOutputArgument(pvApiCtx, 1, 1) ;

    // Get the input image from the Scilab environment
    Mat mask, marker;
    retrieveImage(mask, 1);
    retrieveImage(marker, 2);
    Mat gray_mask, gray_marker;
    try
    {
      mask.convertTo(gray_mask,CV_8UC1);
      marker.convertTo(gray_marker,CV_8UC1);
    }
    catch(cv::Exception&e)
    {
      const char* err=e.what();
      Scierror(999,e.what());
    }  

    if (marker.rows != mask.rows || marker.cols != mask.cols)
    {
      Scierror(999,"Marker image and mask image must be of equal size.");
    }
    for (i = 0; i < marker.cols; i++)
    {
      for (j = 0; j < marker.rows; j++)
      {

        if (!(gray_marker.at<uchar>(i,j) == 0 || gray_marker.at<uchar>(i,j) == 255))
        {
          Scierror(999,"Please ensure marker image (second argument) is binary.");
        }
      }
    }
//*************************************Actual Processing****************************//
    Mat m, dst;
    try 
    {
      min((gray_mask + 1), gray_marker, m);
      imimposemin_imreconstruct(m, gray_marker, dst);
    }
    catch(cv::Exception&e)
    {
      const char* err=e.what();
      Scierror(999,e.what());
    }   
//*************************************** Output generation************************//    
    string tempstring = type2str(dst.type());
    char *checker;
    checker = (char *)malloc(tempstring.size() + 1);
    memcpy(checker, tempstring.c_str(), tempstring.size() + 1);
    returnImage(checker, dst, 1);
    free(checker);

    //Assigning the list as the Output Variable
    AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
    //Returning the Output Variables as arguments to the Scilab environment
    ReturnArguments(pvApiCtx);
    return 0;

  }
  void imimposemin_imreconstruct(Mat g, Mat f, Mat& dest)
  {
    Mat m0, m1, m;
    m1 = f;
    do {
      m0 = m1.clone();
      erode(m0, m, Mat());
      max(g, m, m1);
    } while(countNonZero(m1 != m0) != 0);
    dest = m1.clone();
  }  

/* ==================================================================== */
}