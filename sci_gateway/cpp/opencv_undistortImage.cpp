/********************************************************
Author: Suraj Prakash , Shreyash Sharma
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
  
  int opencv_undistortImage(char *fname, unsigned long fname_len){

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
    double *pdblReal = NULL;
 
    char **pstData = NULL;
    char *currentArg = NULL;
    bool *providedArgs = NULL; 

    double cameraMatrix[3][3]; 
    double *newCameraMatrix = NULL;
    double *distCoeffs = NULL;

    /// checking input argument
    //  1-> image
    //  2-> cameraMatrix
    //  3-> distCoeffs
    //  4-> newCameraMatrix

    CheckInputArgument(pvApiCtx, 2, 6);
    CheckOutputArgument(pvApiCtx, 1, 1);

    /// Take distorted image
    Mat image;
    retrieveImage(image, 1);

    newCameraMatrix = (double*)malloc(sizeof(double) * 3 * 3);
    memset(newCameraMatrix, 0, sizeof(newCameraMatrix));

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
    

    int nbInputArguments = *getNbInputArgument(pvApiCtx);


    providedArgs = (bool*) malloc(sizeof(bool) * 2);
    memset(providedArgs, 0, 2);
 
    for(int iter = 3; iter <= nbInputArguments; ++iter){

        /// Address of next argument
        
        sciErr = getVarAddressFromPosition(pvApiCtx, iter, &piAddr3); 
        if (sciErr.iErr){
            printError(&sciErr, 0); 
            return 0; 
        }

        // Three calls to getMatrixOfString
        sciErr = getMatrixOfString(pvApiCtx, piAddr3, &iRows, &iCols, NULL, NULL); 
        if (sciErr.iErr){
            printError(&sciErr, 0); 
            return 0; 
        }
        
        piLen = (int*) malloc(sizeof(int) * iRows * iCols); 
        
        sciErr = getMatrixOfString(pvApiCtx,  piAddr3,  &iRows,  &iCols,  piLen,  NULL); 
        if (sciErr.iErr){
            printError(&sciErr, 0); 
            return 0; 
        }
        
        pstData = (char**) malloc(sizeof(char*) * iRows * iCols); 
        for(int k = 0; k < iRows * iCols; ++k)
        {
            pstData[k] = (char*) malloc(sizeof(char) * piLen[k] + 1); 
        }
        
        sciErr = getMatrixOfString(pvApiCtx, piAddr3, &iRows, &iCols, piLen, pstData); 
        if (sciErr.iErr){
            printError(&sciErr, 0); 
            return 0; 
        }

        currentArg = pstData[0]; 

        if(strcmp(currentArg, "distCoeffs") == 0){

            if(iter+1 <= nbInputArguments and !providedArgs[0]){
                    
                    sciErr = getVarAddressFromPosition(pvApiCtx, ++iter, &piAddr4); 
                    
                    if (sciErr.iErr){
                        printError(&sciErr, 0); 
                        return 0; 
                    }
                    
                    sciErr = getMatrixOfDouble(pvApiCtx, piAddr4, &iRows1, &iCols1, &pdblReal); 
                    
                    if(sciErr.iErr){
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

                    for(i = 0; i < n; ++i){
                      distCoeffs[i] = pdblReal[i]; 
                    }
                    
                    // Checking if values are in proper range. Same for all optional arguments
                    providedArgs[0] = 1;
                }
                // Send an error message if an argument is provided more than once. Same for all optional arguments.

                else if(providedArgs[0]){
                    Scierror(999, "Please provide optional arguments only once.\n"); 
                    return 0; 
                }

                // Send an error message if name of argument is given but type is incorrect. Same for all optional arguments.

                else {
                    Scierror(999, "Incorrect number of arguments provided. Please check the documentation for more information.\n"); 
                    return 0; 
                }
        }

        else if(strcmp(currentArg, "newCameraMatrix") == 0){

          if(iter+1 <= nbInputArguments and !providedArgs[1]){
                    
                    sciErr = getVarAddressFromPosition(pvApiCtx, ++iter, &piAddr4); 
                    
                    if (sciErr.iErr){
                        printError(&sciErr, 0); 
                        return 0; 
                    }
                    
                    sciErr = getMatrixOfDouble(pvApiCtx, piAddr4, &iRows1, &iCols1, &newCameraMatrix); 
                    
                    if(sciErr.iErr){
                        printError(&sciErr, 0); 
                        return 0; 
                    }

                    if(iRows1 !=3 and iCols1 != 3){
                      Scierror(999, "Incorrect dimension of matrix for argument\n"); 
                      return 0; 
                    }
          
                    // Checking if values are in proper range. Same for all optional arguments
                    providedArgs[1] = 1;
                }
                // Send an error message if an argument is provided more than once. Same for all optional arguments.

                else if(providedArgs[1]){
                    Scierror(999, "Please provide optional arguments only once.\n"); 
                    return 0; 
                }

                // Send an error message if name of argument is given but type is incorrect. Same for all optional arguments.

                else {
                    Scierror(999, "Incorrect number of arguments provided. Please check the documentation for more information.\n"); 
                    return 0; 
                }
        }
        else{
          sciprint("Invalid argument provided; Using defalult values\n");
        }

    }

    try
    {
    Mat _distCoeffs = Mat::zeros(1, n, CV_64F);

    /// if distCoeffs was provided else it will be NULL / empty
    if(providedArgs[0]){
      for(i = 0; i < n; ++i)
        _distCoeffs.at<double>(0, i) = distCoeffs[i];
    }

    /// By default the _newCameraMatrix has the same value of CameraMatrix
    // Mat _newCameraMatrix(3, 3, CV_64F, &cameraMatrix);

    // if(providedArgs[1]){
    //   for(i = 0; i < 3; ++i)
    //     for(j = 0; j < 3; ++j)
    //       _newCameraMatrix.at<double>(i, j) = newCameraMatrix[j * 3 + i];
    // }
    Matx33d knew= _cameraMatrix;
    Mat new_image;
    Size a;
    cv::fisheye::undistortImage(image, new_image, _cameraMatrix, _distCoeffs, knew);


    string tempstring = type2str(new_image.type());
    char *checker;
    checker = (char *)malloc(tempstring.size() + 1);
    memcpy(checker, tempstring.c_str(), tempstring.size() + 1);
    returnImage(checker, new_image, 1);
    free(checker);
    }
    
    catch (cv::Exception & e) 
    {

         Scierror(999,e.what());
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
