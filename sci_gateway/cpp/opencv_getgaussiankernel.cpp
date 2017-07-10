/********************************************************
    Author: Shubheksha Jalan
*********************************************************
Mat getGaussianKernel(int ksize, double sigma, int ktype=CV_64F )
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
  #include <sciprint.h>
  #include "../common.h"

  int opencv_getgaussiankernel(char *fname, unsigned long fname_len)
  {

    SciErr sciErr;
    int intErr=0;
    int iRows=0,iCols=0;
    int *piLen = NULL;
    int *piAddr = NULL;
    int *piAddrNew = NULL;
    int *piAddr2  = NULL;
    int *piAddr3  = NULL;
    double ksize, sigma;
    char **kType = NULL;
    int i, j, k;

      //checking input argument
    CheckInputArgument(pvApiCtx, 3, 3);
    CheckOutputArgument(pvApiCtx, 1, 1) ;



    //for value of ksize
    try{
    sciErr = getVarAddressFromPosition(pvApiCtx, 1, &piAddr);
    if (sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }
     intErr = getScalarDouble(pvApiCtx, piAddr, &ksize);
     if(intErr){
       return intErr;
     }
    if(ksize < 0 || (int)ksize % 2 == 0)
    {
      Scierror(999, _("%s: Wrong type for input argument #%d:ksize should be positive and odd.\n"), fname, 1);
      return 0;
    }


    //for first value of size
    sciErr = getVarAddressFromPosition(pvApiCtx,2,&piAddr2);
    if (sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }
     intErr = getScalarDouble(pvApiCtx, piAddr2, &sigma);
     if(intErr)
    {
        return intErr;
    }
    if(sigma<=0){
      sigma=0.3*((ksize-1)*0.5 - 1) + 0.8;
    }

    sciErr = getVarAddressFromPosition(pvApiCtx, 3, &piAddr3);
    if (sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }

      //Now, we will retrieve the string from the input parameter. For this, we will require 3 calls
    //first call to retrieve dimensions
    sciErr = getMatrixOfString(pvApiCtx, piAddr3, &iRows, &iCols, NULL, NULL);
    if(sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }
    piLen = (int*)malloc(sizeof(int) * iRows * iCols);
    //second call to retrieve length of each string
    sciErr = getMatrixOfString(pvApiCtx, piAddr3, &iRows, &iCols, piLen, NULL);
    if(sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }

    kType = (char**)malloc(sizeof(char*) * iRows * iCols);
    for(i = 0 ; i < iRows * iCols ; i++)
        kType[i] = (char*)malloc(sizeof(char) * (piLen[i] + 1));//+ 1 for null termination

    //third call to retrieve data
    sciErr = getMatrixOfString(pvApiCtx, piAddr3, &iRows, &iCols, piLen, kType);
    if(sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }


    Mat temp;

    if(strcmp(kType[0], "CV_32F") == 0)
    	temp = getGaussianKernel(ksize, sigma, CV_32F);
    else if(strcmp(kType[0], "CV_64F") == 0)
    	temp = getGaussianKernel(ksize, sigma, CV_64F);
    else{
      Scierror(999, _("%s: Wrong type for input argument #%d:CV_32F or CV_64F expected.\n"), fname, 3);
      return 0;
    }
     double *m = (double *)malloc(temp.rows*temp.cols*sizeof(double));
    for(i=0;i<temp.rows;i++)
    {
      for(j=0;j<temp.cols;j++)
      {
        uchar intensity = temp.at<uchar>(i, j);
        *(m + i*temp.cols + j) = intensity;
      }
    }

    sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 1, temp.rows, temp.cols, m);
  }
  catch(cv::Exception& e){
   const char* err=e.what();
   Scierror(999,("%s",err));
  }
	//Assigning the list as the Output Variable
    AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
    //Returning the Output Variables as arguments to the Scilab environment
    ReturnArguments(pvApiCtx);
    return 0;

  }
/* ==================================================================== */
}
