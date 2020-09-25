/*
This is the .cpp gateway file for the 'denoiseColor' scilab function.

It includes the following OpenCV functions, belonging to the Photo module of OpenCV 3.0.0 : 

void cv::fastNlMeansDenoisingColored (InputArray src, OutputArray dst, float h = 3,float hColor = 3, int templateWindowSize = 7,int searchWindowSize = 21 ) 	

	Removes noise from the input image. Also removes colored noise.

Author : Shubham Lohakare, NITK Surathkal
	 Ashish Mantosh Barik, NIT Rourkela
*/

#include<numeric>
#include"opencv2/core/core.hpp"
#include"opencv2/highgui/highgui.hpp"
#include"opencv2/opencv.hpp"
#include"opencv2/shape/shape_transformer.hpp"
#include"opencv2/shape.hpp"
#include"opencv2/imgcodecs.hpp"
#include"opencv2/imgproc/imgproc.hpp"
#include"opencv2/features2d/features2d.hpp"
#include"opencv2/core/utility.hpp"
#include<string>
#include<iostream>
#include<cstdlib>

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

  	int opencv_denoiseColor(char *fname, unsigned long fname_len)
  	{	SciErr sciErr;	
		int i;
		
		CheckInputArgument(pvApiCtx ,5 ,5 );//Checking number of Input Arguments
		CheckOutputArgument(pvApiCtx, 1, 1);//Checking number of Output Arguments
		
		Mat src,dst;

		int *piAddr2 = NULL;//Variables for 2nd Input Argument
		int intErr2 = 0;
		double filterStrength = 0;
		
		int *piAddr3 = NULL;//Variables for 3rd Input Argument
		int intErr3 = 0;
		double filterStrengthColor=0;

		int *piAddr4 = NULL;//Variables for 4th Input Argument
		int intErr4 = 0;
		double templateWindowSize=0;

		int *piAddr5 = NULL;//Variables for 5th Input Argument
		int intErr5 = 0;
		double searchWindowSize=0;
	
		
		// to get the argument #2
		sciErr = getVarAddressFromPosition( pvApiCtx, 2, &piAddr2);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}

		intErr2 = getScalarDouble(pvApiCtx, piAddr2, &filterStrength);
		if(intErr2)	
		{
			return intErr2;
		}

		// to get the argument #3
		sciErr = getVarAddressFromPosition( pvApiCtx, 3, &piAddr3);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}

		intErr3 = getScalarDouble(pvApiCtx, piAddr3, &filterStrengthColor);
		if(intErr3)	
		{
			return intErr3;
		}

		// to get the argument #4
		sciErr = getVarAddressFromPosition( pvApiCtx, 4, &piAddr4);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}

		intErr4 = getScalarDouble(pvApiCtx, piAddr4, &templateWindowSize);
		if(intErr4)	
		{
			return intErr4;
		}
		
		// to get the argument #5
		sciErr = getVarAddressFromPosition( pvApiCtx, 5, &piAddr5);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}

		intErr5 = getScalarDouble(pvApiCtx, piAddr5, &searchWindowSize);
		if(intErr5)	
		{	
			return intErr5;
		}
		try{
		
		retrieveImage(src,1);//Retrieving the input image
		src.convertTo(src,CV_8U);//Converting to 8 bit image
		fastNlMeansDenoisingColored(src, dst, filterStrength,filterStrengthColor, templateWindowSize, searchWindowSize);//Calling the denoising OpenCV function
		}
		catch(Exception& e)
		{
			const char* err = e.what();
			sciprint("%s", err);
		} 

		string tempstring = type2str(dst.type());//Returning the output to scilab console
        	char* checker = (char *)malloc(tempstring.size() + 1);
        	memcpy(checker, tempstring.c_str(), tempstring.size() + 1);
        	returnImage(checker,dst,1);
        	AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
        	ReturnArguments(pvApiCtx);
  	}

}
