/*
        Author-Nihar Rao


        This function computes the corners in a image using openCV's cornerHarris function.

      Mat ans= void cornerHarris(InputArray src, OutputArray dst, int blockSize, int ksize, double k, int borderType=BORDER_DEFAULT )
*/

#include <numeric>
#include<bits/stdc++.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <iostream>
#include <math.h>
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




  int opencv_detectMinEigenFeatures(char *fname, unsigned long fname_len)
  {


     SciErr sciErr;
    int intErr = 0;
    int *piAddrNew = NULL;
    int *piAddr1  = NULL;
    int *piAddr2  = NULL;
    int *piAddr3  = NULL;
    int *piAddr4  = NULL;
    int *piAddr5  = NULL;
    int blocksize,ksize,border_type;
    double k;

    CheckInputArgument(pvApiCtx, 4, 5);
    CheckOutputArgument(pvApiCtx, 1, 1);
    int n=*getNbInputArgument();//get number of input arguments


    Mat image;
    retrieveImage(image,1);

    //for blocksize
        sciErr = getVarAddressFromPosition(pvApiCtx,2,&piAddr1);
        if (sciErr.iErr)
        {
        printError(&sciErr, 0);
        return 0;
        }
        intErr= getScalarUnsignedInteger16(pvApiCtx, piAddr1, &blocksize);
        if(intErr)
        return intErr;

     //for ksize
        sciErr = getVarAddressFromPosition(pvApiCtx,3,&piAddr2);
        if (sciErr.iErr)
        {
        printError(&sciErr, 0);
        return 0;
        }
        intErr= getScalarUnsignedInteger16(pvApiCtx, piAddr2, &ksize);
        if(intErr)
        return intErr;

        //for k

        sciErr = getVarAddressFromPosition(pvApiCtx,4,&piAddr3);
        if (sciErr.iErr)
        {
        printError(&sciErr, 0);
        return 0;
        }
        intErr = getScalarDouble(pvApiCtx, piAddr3 ,&k);
        if(intErr)
        return intErr;

        int flag=0;
        //get optional argument bordder_type if passed by the user
        if(n==5)
        {
        flag=1;
        sciErr = getVarAddressFromPosition(pvApiCtx,5,&piAddr4);
        if (sciErr.iErr)
        {
        printError(&sciErr, 0);
        return 0;
        }
        intErr= getScalarUnsignedInteger16(pvApiCtx, piAddr4, &border_type);
        if(intErr)
        return intErr;
        }
        Mat new_image;


        //finally call to function

        if(flag)
            cornerHarris(image,new_image,blocksize,k,border_type);
        else
            cornerHarris(image,new_image,blocksize,k);


     //return the image to scilab memory
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







}
