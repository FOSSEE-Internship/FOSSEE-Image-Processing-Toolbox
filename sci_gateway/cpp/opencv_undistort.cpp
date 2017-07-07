/***************************************************
Author : Sukul Bagai, Shubheksha Jalan,Gursimar
***************************************************/

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
  #include "../common.h"
  #include "sciprint.h"  
  
  int opencv_undistort(char *fname, unsigned long fname_len)
  {
    SciErr sciErr;
    int iRows=0,iCols=0;
    int *piAddr2  = NULL;
    int *piAddr3  = NULL;
    int *piAddr4  = NULL;
    int *piAddr5  = NULL;
    int i,j,k,n,nb;
    double cameraMatrix [3][3], newCameraMatrix [3][3];
    double *temp = NULL;
    Mat map1,map2;
    double distCoeffs[n];
 //checking input argument
    CheckInputArgument(pvApiCtx, 3, 4);
    CheckOutputArgument(pvApiCtx, 1, 1) ;
    nb=*getNbInputArgument(pvApiCtx);
    Mat image;
    retrieveImage(image, 1); 

	//for camera matrix 
    sciErr = getVarAddressFromPosition(pvApiCtx,2,&piAddr2);
    if (sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }
    sciErr = getMatrixOfDouble(pvApiCtx, piAddr2, &iRows, &iCols, &temp);
    if(sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }
    //retrieving Camera matrix
    for(i=0;i<3;i++)
    {
        for(j=0;j<3;j++)
        {
            cameraMatrix[i][j]=temp[(j*3)+i];
           
        }
    }

    Mat cameraMat(3,3,CV_64F,&cameraMatrix);
   
//for array of coefficients
    sciErr = getVarAddressFromPosition(pvApiCtx,3,&piAddr3);
    if (sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }
    sciErr = getMatrixOfDouble(pvApiCtx, piAddr3, &iRows, &iCols, &temp);
    if(sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }

    
    if(iRows==1)
        n=iCols;
    else
        n=iRows;
    sciprint("%d\n",n);
    for(i=0;i<n;i++)
        distCoeffs[i]=temp[i];

    Mat distCoeffsActual(1,4,CV_64F,&distCoeffs);
   
//for optional parameter newCameraMatrix
    Mat newCameraMat(3,3,CV_64F);
    if (nb>3)
    { 
        sciErr = getVarAddressFromPosition(pvApiCtx,4,&piAddr4);
    if (sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }
    sciErr = getMatrixOfDouble(pvApiCtx, piAddr4, &iRows, &iCols , &temp);
    if(sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    } 

    for(i=0;i<3;i++)
    {
        for(j=0;j<3;j++)
        {
            newCameraMatrix[i][j]=temp[(j*3)+i];
        }
    }

    Mat newCameraMat(3,3,CV_64F,&newCameraMatrix);

    }
   
    
  
  Mat new_image;
  Size imageSize;
  imageSize=image.size();
  try
  {
    cv::initUndistortRectifyMap(cameraMat, distCoeffsActual, Mat(),getOptimalNewCameraMatrix(cameraMat, distCoeffsActual, imageSize,1,imageSize,0),image.size(), CV_16SC2, map1, map2);
    remap(image, new_image, map1, map2, INTER_LINEAR);
  }
    catch(cv::Exception&e)
    {
        const char* err=e.what();
        Scierror(999,e.what());
    }
    
    string tempstring = type2str(new_image.type());
    char *checker;
    checker = (char *)malloc(tempstring.size() + 1);
    memcpy(checker, tempstring.c_str(), tempstring.size() + 1);
    returnImage(checker,new_image,1);
    free(checker); 

    //Assigning the list as the Output Variable
    AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
    //Returning the Output Variables as arguments to the Scilab environment
    ReturnArguments(pvApiCtx);
    return 0;

  }
/* ==================================================================== */
}