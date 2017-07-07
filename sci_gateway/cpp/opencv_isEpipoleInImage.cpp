/********************************************************
Author: Suraj Prakash
[isepi, epipole] = isEpipoleInImage(fundamental_matrix, imagesize)
********************************************************/

#include <numeric>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>

using namespace cv;
using namespace std;

extern "C"{
  #include "api_scilab.h"
  #include "Scierror.h"
  #include "BOOL.h"
  #include <localization.h>
  #include <sciprint.h>
  #include "../common.h"
  
  int opencv_isEpipoleInImage(char *fname, unsigned long fname_len){

    /// Error management variable
    SciErr sciErr;
    
    /// Variables
    int i, j, n = 0;

    int iRows = 0;
    int iCols = 0;
    
    int *piLen = NULL;
    int *piAddr = NULL;
    int *piAddr2 = NULL;
    
    double *pdblReal = NULL;
    double *imagesize = NULL;
    
    int isepi = 0;
    double *epipole = NULL;
    double inf = 1e17;
    /// take the fundamental matrix 
    Mat fundamental_matrix(3, 3, CV_64F);

    epipole = (double*)malloc(sizeof(double*) * 2);
    memset(epipole, 0, 2);
   
    sciErr = getVarAddressFromPosition(pvApiCtx, 1, &piAddr);
    if (sciErr.iErr){
        printError(&sciErr, 0);
        return 0;
    }

    sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &iRows, &iCols, &pdblReal);
    if(sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }

    if(iRows != 3 and iCols != 3){
        Scierror(999, "Incorrect dimension of martrix for argument\n"); 
        return 0; 
    }

    for(i = 0; i < 3; ++i)
        for(j = 0; j < 3; ++j)
            fundamental_matrix.at<double>(i, j) = pdblReal[i + j * 3];


    /// Get image size    
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

    /*if(iRows != 1 and iCols != 2){
        Scierror(999, "Invalid array for imagesize provided\n"); 
        return 0; 
    }*/
    
    try
    {
    imagesize = (double*)malloc(sizeof(double) * 2);
    imagesize[0] = pdblReal[0];
    imagesize[1] = pdblReal[1];

    /// compute the SVD of the matrix provided
    Mat w, u, vt;
    SVD::compute(fundamental_matrix, w, u, vt);

    transpose(vt, vt);

    /// getting the last column of the vt
    Mat epipoleHmg(1, 3, CV_64F);

    epipoleHmg.at<double>(0,0) = vt.at<double>(0, 2);
    epipoleHmg.at<double>(0,1) = vt.at<double>(1, 2);
    epipoleHmg.at<double>(0,2) = vt.at<double>(2, 2);


    if(epipoleHmg.at<double>(0,2) != 0){
        /// location of epipolecl

        epipole[0] = epipoleHmg.at<double>(0, 0) / epipoleHmg.at<double>(0, 2);
        epipole[1] = epipoleHmg.at<double>(0, 1) / epipoleHmg.at<double>(0, 2);

        double *imageOrigin = NULL;
        imageOrigin = (double*)malloc(sizeof(double) * 2);
        memset(imageOrigin, 0.5, 2);

        double *imageEnd = NULL;
        imageEnd = (double*)malloc(sizeof(double) * 2);
        memset(imageEnd, 0, 2);

        /// imageend = imageorigin + imagesize(::-1)
        imageEnd[0] = imageOrigin[0] + imagesize[1];
        imageEnd[1] = imageOrigin[1] + imagesize[0];

        if(epipole[0] >= imageOrigin[0] and epipole[1] >= imageOrigin[1] and epipole[0] <= imageEnd[0] and epipole[1] <= imageEnd[1])
            isepi = 1;
        else
            isepi = 0;
    }

    else{
        epipole[0] = epipoleHmg.at<double>(0, 0) > 0 ? inf : -inf;
        epipole[1] = epipoleHmg.at<double>(0, 1) > 0 ? inf : -inf;
    }

    
    }
    catch (cv::Exception & e) 
    {

         Scierror(999,e.what());
         return 0;

    }


    int iRet = createScalarBoolean(pvApiCtx, nbInputArgument(pvApiCtx) + 1, isepi);
    if(iRet){
        printError(&sciErr, 0); 
        return 0; 
    }
    
    sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 2, 1, 2, epipole); 
    if(sciErr.iErr){
        printError(&sciErr, 0); 
        return 0; 
    }
    //Assigning the list as the Output Variable
    AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
    AssignOutputVariable(pvApiCtx, 2) = nbInputArgument(pvApiCtx) + 2; 

    //Returning the Output Variables as arguments to the Scilab environment
    ReturnArguments(pvApiCtx);
    return 0;
    }
    
/* ==================================================================== */
}
