/***************************************************
Author : Deepshikha
***************************************************/

#include <numeric>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/calib3d/calib3d.hpp"
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
  
  int opencv_triangulatePoints(char *fname, unsigned long fname_len)
  {
    //variables
    int i,j,n,m;
    int iRows=0,iCols=0;
    int *piAddr1  = NULL;
    int *piAddr2  = NULL;
    int *piAddr3  = NULL;
    int *piAddr4  = NULL;
    
    double *pdblReal = NULL;

    SciErr sciErr;

    /*------------------------------------------- Checking Input Argument ----------------------------------------------------*/
    
    CheckInputArgument(pvApiCtx, 4, 4);
    CheckOutputArgument(pvApiCtx, 1, 1) ;

    /*------------------------------------- Projection matrix 1 :- First argument of the input -------------------------------------*/
    
    sciErr = getVarAddressFromPosition(pvApiCtx, 1, &piAddr1);
    if (sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }
    sciErr = getMatrixOfDouble(pvApiCtx, piAddr1, &iRows, &iCols, &pdblReal);
    if(sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }

    if(iRows!=3 or iCols!=4){
        Scierror(iRows,"Projection matrix (arg 1)  must be a 3 X 4 matrix");
        return 0;
    }
    Mat projMat1(iRows, iCols, CV_64F);
    for(i = 0; i < iRows; i++)
        for(j = 0; j < iCols; j++){
           projMat1.at<double>(i,j) = pdblReal[i + (j * iRows)];
        }
    
    /* ------------------------------------ Projection matrix 2 :- Second argument of the input -------------------------------------*/

    sciErr = getVarAddressFromPosition(pvApiCtx, 2, &piAddr2);
    if (sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }
    sciErr = getMatrixOfDouble(pvApiCtx, piAddr2, &iRows, &iCols, &pdblReal);
    if(sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }

    if(iRows!=3 or iCols!=4){
        Scierror(iRows,"Projection matrix (arg 2)  must be a 3 X 4 matrix");
        return 0;
    }
    Mat projMat2(iRows, iCols, CV_64F);
    for(i=0;i<iRows;i++)
        for(j=0;j<iCols;j++)
            projMat2.at<double>(i,j) = pdblReal[i + (j * iRows)];


    /* ------------------------------------ Points matrix 1 :- Third argument of the input -------------------------------------*/
    
    sciErr = getVarAddressFromPosition(pvApiCtx,3,&piAddr3);
    if (sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }
    sciErr = getMatrixOfDouble(pvApiCtx, piAddr3, &iRows, &iCols, &pdblReal);
    if(sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }
    n = iCols;
    if(iRows!=2){
        Scierror(iRows,"Points matrix (arg 3) must be a 2 X N matrix");
        return 0;
    }
    Mat newPoint1(2, iCols, CV_64F);
    for(i = 0; i < 2; i++)
        for(j = 0; j < iCols; j++)
            newPoint1.at<double>(i,j) = pdblReal[ i + ( j * iRows )];
    

    /* ------------------------------------ Points matrix 2 :- Fourth argument of the input -------------------------------------*/
    
    sciErr = getVarAddressFromPosition(pvApiCtx, 4, &piAddr4);
    if (sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }
    sciErr = getMatrixOfDouble(pvApiCtx, piAddr4, &iRows, &iCols, &pdblReal);
    if(sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }
    
    m = iCols;
    if(iRows!=2){
        Scierror(iRows,"Points matrix (arg 4) must be a 2 X N matrix");
        return 0;
    }
    Mat newPoint2(iRows, iCols, CV_64F);
    for(i = 0; i < iRows; i++)
        for(j = 0; j < iCols; j++)
            newPoint2.at<double>(i,j) = pdblReal[ i + ( j * iRows )];

    if(n!=m){
        Scierror(n,"No of columns of arg 3 and arg 4 must be same");
        return 0;
    }
    Mat new_image(4, n, CV_64F);

    triangulatePoints(projMat1, projMat2, newPoint1, newPoint2, new_image);

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