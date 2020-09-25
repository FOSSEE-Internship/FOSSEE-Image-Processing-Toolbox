/********************************************************
Author: Vinay, 
	Shubham Lohakare, NITK Surathkal

Function: ind2gray(image, choice)
image: input image
choice: choice of the colormap which the user wants to apply
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

  int opencv_ind2gray(char *fname, unsigned long fname_len)
  {

    SciErr sciErr;
    int intErr = 0;
    int iRows=0,iCols=0;
    int cRows=0,cCols=0;
    int *piAddr = NULL;
    int *piAddrNew = NULL;
	
int *piAddr2 = NULL;//Variables for 2nd Input Argument
		int intErr2 = 0;
		double choice = 0;
    //checking input argument
    CheckInputArgument(pvApiCtx, 2, 2);
    CheckOutputArgument(pvApiCtx, 1, 1) ;

    Mat image, imgcpy;
    retrieveImage(image, 1);
	image.convertTo(image,CV_8U);
  
    iRows = image.rows;
    iCols = image.cols;
    image.convertTo(imgcpy, CV_64F);
	//cvtColor(image,image,CV_BGR2GRAY);
    Mat cmap, cmapcpy;
    // to get the argument #2
		sciErr = getVarAddressFromPosition( pvApiCtx, 2, &piAddr2);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}

		intErr2 = getScalarDouble(pvApiCtx, piAddr2, &choice);
		if(intErr2)	
		{
			return intErr2;
		}

	if(choice==0)
	applyColorMap(image, cmap, COLORMAP_AUTUMN);

	else if(choice==1)
	applyColorMap(image, cmap, COLORMAP_BONE);

	else if(choice==2)
	applyColorMap(image, cmap, COLORMAP_JET);
	
	else if(choice==3)
	applyColorMap(image, cmap, COLORMAP_WINTER);

	else if(choice==4)
	applyColorMap(image, cmap, COLORMAP_RAINBOW);

	else if(choice==5)
	applyColorMap(image, cmap, COLORMAP_OCEAN);

	else if(choice==6)
	applyColorMap(image, cmap, COLORMAP_SUMMER);

	else if(choice==7)
	applyColorMap(image, cmap, COLORMAP_SPRING);

	else if(choice==8)
	applyColorMap(image, cmap, COLORMAP_COOL);

	else if(choice==9)
	applyColorMap(image, cmap, COLORMAP_HSV);

	else if(choice==10)
	applyColorMap(image, cmap, COLORMAP_PINK);

	else if(choice==11)
	applyColorMap(image, cmap, COLORMAP_HOT);

	else if(choice==12)
	applyColorMap(image, cmap, COLORMAP_PARULA);
	
	else
	printf("Wrong input of choice");

	
    cRows = cmap.rows;
    cCols = cmap.cols;
    cmap.convertTo(cmapcpy, CV_64F);

    /*for (int i=0; i<cRows; i++) {
        for (int j=0; j<cCols; j++) {
            if (cmapcpy.at<double>(i,j)<0 || cmapcpy.at<double>(i,j)>1) {
                sciprint("Invalid colormap");
                return 0;
            }
        }
    }*/

    Mat gray = Mat::zeros(image.size(), CV_64F);


    for (int i=0; i<iRows; i++) {
        for (int j=0; j<iCols; j++) {
            unsigned int temp = (unsigned int)imgcpy.at<double>(i, j);
            if (temp >= cRows) {
                temp = cRows - 1;
            }
            if (!true) {
                if (temp!=0) {temp-=1;}
            }
            gray.at<double>(i,j) = (0.2989 * cmapcpy.at<double>(temp, 0) + 0.5870 * cmapcpy.at<double>(temp, 1) + 0.1140 * cmapcpy.at<double>(temp, 2)) ;

        }
    }

    Mat grayimage;
    gray.convertTo(grayimage, CV_64F);
    string tempstring = type2str(grayimage.type());//Returning the image to Scilab console
        	char* checker = (char *)malloc(tempstring.size() + 1);
        	memcpy(checker, tempstring.c_str(), tempstring.size() + 1);
        	returnImage(checker,grayimage,1);
        	AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
        	ReturnArguments(pvApiCtx);           
    return 0;

  }
/* ==================================================================== */
}
