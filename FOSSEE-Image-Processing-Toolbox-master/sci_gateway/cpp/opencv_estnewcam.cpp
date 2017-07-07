/********************************************************
Author: Suraj Prakash
return_image = undistortImage(image, _cameraMatrix, _distCoeffs, _newCameraMatrix);
********************************************************/

#include <numeric>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

extern "C"{
  #include "api_scilab.h"
  #include "Scierror.h"
  #include "BOOL.h"
  #include <localization.h>
  #include <sciprint.h>
  #include "../common.h"
  
  int opencv_estnewcam(char *fname, unsigned long fname_len)
  {

    /// Error management variable
    SciErr sciErr;
    
    /// Variables
    int i, j, n = 0;

    int iRows = 0;
    int iCols = 0;
    int iRows1 = 0;
    int iCols1 = 0;
    int *piLen = NULL;
    int *piAddr2 = NULL;
    int *piAddr3 = NULL;
    int *piAddr4 = NULL;
    int *piAddr5 = NULL;
    double *pdblReal = NULL;
    
    Mat newcamMat;
    Size a;
    char **pstData = NULL;
    char *currentArg = NULL;
    bool *providedArgs = NULL; 

    double cameraMatrix[3][3]; 
    double *rectificationmatrix = NULL;
    double *distCoeffs = NULL;

    /// checking input argument
    //  1-> image
    //  2-> cameraMatrix
    //  3-> distCoeffs
    //  4-> newCameraMatrix

    CheckInputArgument(pvApiCtx, 4, 4);
    CheckOutputArgument(pvApiCtx, 1, 1);

    /// Take distorted image
    Mat image;
    retrieveImage(image, 1);

    rectificationmatrix = (double*)malloc(sizeof(double) * 3 * 3);
    memset(rectificationmatrix, 0, sizeof(rectificationmatrix));

    /// Taking input for cameraMatrix
    sciErr = getVarAddressFromPosition(pvApiCtx, 2, &piAddr2);
    if (sciErr.iErr){
        printError(&sciErr, 0);
        return 0;
    }

    sciErr = getMatrixOfDouble(pvApiCtx, piAddr2, &iRows, &iCols, &pdblReal);
    if(sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }

    /// Take input in cameraMatrix
    Mat _cameraMatrix(3, 3, CV_64F);
    for(i = 0; i < 3; ++i)
        for(j = 0;j < 3; ++j)
            _cameraMatrix.at<double>(i, j) = pdblReal[(j * 3) + i];
    

       sciprint("hello1");
        

           sciErr = getVarAddressFromPosition(pvApiCtx, 3, &piAddr4); 
                  
             if (sciErr.iErr)
                 {
                   printError(&sciErr, 0); 
                   return 0; 
                 }
                    
           sciErr = getMatrixOfDouble(pvApiCtx, piAddr4, &iRows1, &iCols1, &pdblReal); 
                   
             if(sciErr.iErr)
                  {
                    printError(&sciErr, 0); 
                    return 0; 
                  }
                  
             if(iRows1 == 1 or iCols1 == 1)
                n = iCols1 * iRows1;
                    
             else{
                 Scierror(999, "Incorrect dimension of vector for argument\n"); 
                 return 0; 
                 }

                    // if(n != 4 or n != 5 or n != 8){
                    //     Scierror(999, "Incorrect dimension of matrix for argument. Only size of 4, 5, or 8 required\n"); 
                    //     return 0; 
                    // }

                  distCoeffs = (double*)malloc(sizeof(double) * n);

                  for(i = 0; i < n; ++i)
                  {
                    distCoeffs[i] = pdblReal[i]; 
                  }
                    
               sciprint("hello2");
               
             sciErr = getVarAddressFromPosition(pvApiCtx, 4, &piAddr5); 
                    
                if (sciErr.iErr)
                {
                   printError(&sciErr, 0); 
                   return 0; 
                 }
                    
                  sciErr = getMatrixOfDouble(pvApiCtx, piAddr5, &iRows1, &iCols1, &rectificationmatrix); 
                    
                if(sciErr.iErr)
                {
                  printError(&sciErr, 0); 
                  return 0; 
                }

                if(iRows1 !=3 and iCols1 != 3)
                {
                  Scierror(999, "Incorrect dimension of matrix for argument\n"); 
                  return 0; 
                 }
        
                    
      sciprint("hello3");

    try
    {
    Mat _distCoeffs = Mat::zeros(1, n, CV_64F);
    
    sciprint("hello41");

    /// if distCoeffs was provided else it will be NULL / empty
    
      for(i = 0; i < n; ++i)
        _distCoeffs.at<double>(0, i) = distCoeffs[i];
    
     sciprint("hello42");
    /// By default the _newCameraMatrix has the same value of CameraMatrix
    Mat _newrectificationmatrix(3, 3, CV_64F, &rectificationmatrix);

    
     /* for(i = 0; i < 3; ++i)
        for(j = 0; j < 3; ++j)
          _newrectificationmatrix.at<double>(i, j) = rectificationmatrix[j * 3 + i];
    */

      sciprint("hello4");
    cv::fisheye::estimateNewCameraMatrixForUndistortRectify( _cameraMatrix, _distCoeffs,image.size(),_newrectificationmatrix,newcamMat,0.0,a,1.0);
     sciprint("hello5");

    /*string tempstring = type2str(new_image.type());
    char *checker;
    checker = (char *)malloc(tempstring.size() + 1);
    memcpy(checker, tempstring.c_str(), tempstring.size() + 1);
    returnImage(checker, new_image, 1);
    free(checker);*/
    }
    
        	catch(cv::Exception &e)
        	{
           		const char *err = e.what();
           		sciprint("%s",err);
        	}
        	
    //size=newcamMat.size();
    sciprint("hello6");
    double *points = (double*) malloc(sizeof(double) * 3*3);
    
     for(i = 0; i < 3; ++i)
        for(j = 0; j < 3; ++j)
          points[j * 3 + i]=newcamMat.at<double>(i, j);
    
    sciprint("hello7");
	
	sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 1, 3 , 3, points);
	if(sciErr.iErr)
	{
		printError(&sciErr, 0);
		return 0;
	}
	    	

    //Assigning the list as the Output Variable
    AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;

    //Returning the Output Variables as arguments to the Scilab environment
    ReturnArguments(pvApiCtx);
    return 0;

  }
/* ==================================================================== */
}
