/********************************************************
Author: Vinay

Function: ind2rgb(image, colormap)
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

  int opencv_ind2rgb(char *fname, unsigned long fname_len)
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
    //unsigned short int *image = NULL;
    double *map = NULL;
    int i,j,k;
    double x, y, width, height;

    //checking input argument
    CheckInputArgument(pvApiCtx, 2, 2);
    CheckOutputArgument(pvApiCtx, 1, 1) ;

    Mat image, imgcpy;
    retrieveImage(image, 1);
    
    iRows = image.rows;
    iCols = image.cols;
    image.convertTo(imgcpy, CV_64F);

    Mat cmap, cmapcpy;
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

 
    

    double *r,*g,*b;
    r=(double *)malloc(sizeof(double)*iRows*iCols);
    g=(double *)malloc(sizeof(double)*iRows*iCols);
    b=(double *)malloc(sizeof(double)*iRows*iCols);
    int m = 0;


    for (int i=0; i<iRows; i++) {
        for (int j=0; j<iCols; j++) {
            unsigned int temp = (unsigned int)imgcpy.at<double>(i, j);
            if (temp >= cRows) {
                temp = cRows - 1;
            }
            if (!true) {
                if (temp!=0) {temp-=1;}
            }
            r[i + iRows*j] = cmapcpy.at<double>(temp, 0);
            g[i + iRows*j] = cmapcpy.at<double>(temp, 1);
            b[i + iRows*j] = cmapcpy.at<double>(temp, 2);
        }
    }

    sciErr = createList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, 3, &piAddrNew);
    if(sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }


    //Adding the R value matrix to the list
    //Syntax : createMatrixOfInteger32InList(void* _pvCtx, int _iVar, int* _piParent, int _iItemPos, int _iRows, int _iCols, const int* _piData)
    sciErr = createMatrixOfDoubleInList(pvApiCtx, nbInputArgument(pvApiCtx)+1 , piAddrNew, 1,iRows, iCols, r);
    free(r);
    if(sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }

    //Adding the G value matrix to the list
    sciErr = createMatrixOfDoubleInList(pvApiCtx, nbInputArgument(pvApiCtx)+1 , piAddrNew, 2,iCols, iRows, g);
    free(g);
    if(sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }

    //Adding the B value matrix to the list
    sciErr = createMatrixOfDoubleInList(pvApiCtx, nbInputArgument(pvApiCtx)+1 , piAddrNew, 3,   iCols, iRows, b);
    free(b);
    if(sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }


    AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
    //Returning the Output Variables as arguments to the Scilab environment
    ReturnArguments(pvApiCtx);            
    return 0;

  }
/* ==================================================================== */
}
