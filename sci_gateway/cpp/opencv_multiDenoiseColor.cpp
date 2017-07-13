/*
This is the .cpp file for the 'multiDenoiseColor' scilab function.
It includes the OpenCV function fastNlMeansDenoisingColoredMulti()
It is used for removing colored noise from an image using multiple copies of the image captured in a very minute time interval. 
Author : Shubham Lohakare, NITK Surathkal
	 Ashish Manatosh Barik, NIT Rourkela
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

  	int opencv_multiDenoiseColor(char *fname, unsigned long fname_len)
  	{	SciErr sciErr;	
		int i;
		
		// Checking number of input and output arguments (enviromnet variable, min arguments, max arguments)
		CheckInputArgument(pvApiCtx, 10, 12);
		CheckOutputArgument(pvApiCtx, 1, 1);

		// variables required to read argument #1
		int *piAddr1 = NULL;
		int intErr1 = 0;
		double imgToDenoiseIndex = 0;

		// variables required to read argument #2
		int *piAddr2 = NULL;
		int intErr2 = 0;
		double temporalWindowSize = 0;		
			
		// variables required to read argument #3
		int *piAddr3 = NULL;
		int intErr3 = 0;
		double filterStrength = 0;

		// variables required to read argument #4
		int *piAddr4 = NULL;
		int intErr4 = 0;
		double filterStrengthColor = 0;

		// variables required to read argument #5
		int *piAddr5 = NULL;
		int intErr5 = 0;
		double templateWindowSize = 0;

		// variables required to read argument #6
		int *piAddr6 = NULL;
		int intErr6 = 0;
		double searchWindowSize = 0;

		// variables required to read argument #7
		int *piAddr7 = NULL;
		int intErr7 = 0;
		double n = 0;


		cv::Mat img1,img2,img3,dst;		

		// to get argument #1
		sciErr = getVarAddressFromPosition(pvApiCtx, 1, &piAddr1);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
		intErr1 = getScalarDouble(pvApiCtx, piAddr1, &imgToDenoiseIndex);
		if(intErr1)
		{
			return intErr1;
		}
		
		// to get argument #2
		sciErr = getVarAddressFromPosition(pvApiCtx, 2, &piAddr2);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
		intErr2 = getScalarDouble(pvApiCtx, piAddr2, &temporalWindowSize);
		if(intErr2)
		{
			return intErr2;
		}	
		
		// to get argument #3
                sciErr = getVarAddressFromPosition(pvApiCtx, 3, &piAddr3);
                if(sciErr.iErr)
                {
                        printError(&sciErr, 0);
                        return 0;
                }
                intErr3 = getScalarDouble(pvApiCtx, piAddr3, &filterStrength);
                if(intErr3)
                {
                        return intErr3;
                }		
		// to get argument #4
                sciErr = getVarAddressFromPosition(pvApiCtx, 4, &piAddr4);
                if(sciErr.iErr)
                {
                        printError(&sciErr, 0);
                        return 0;
                }
                intErr4 = getScalarDouble(pvApiCtx, piAddr4, &filterStrengthColor);
                if(intErr4)
                {
                        return intErr4;
                }	
		// to get argument #5
                sciErr = getVarAddressFromPosition(pvApiCtx, 5, &piAddr5);
                if(sciErr.iErr)
                {
                        printError(&sciErr, 0);
                        return 0;
                }
                intErr5 = getScalarDouble(pvApiCtx, piAddr5, &templateWindowSize);
                if(intErr5)
                {
                        return intErr5;
                }		

		// to get argument #6
                sciErr = getVarAddressFromPosition(pvApiCtx, 6, &piAddr6);
                if(sciErr.iErr)
                {
                        printError(&sciErr, 0);
                        return 0;
                }
                intErr6 = getScalarDouble(pvApiCtx, piAddr6, &searchWindowSize);
                if(intErr6)
                {
                        return intErr6;
                }	
		
		// to get argument #7
                sciErr = getVarAddressFromPosition(pvApiCtx, 7, &piAddr7);
                if(sciErr.iErr)
                {
                        printError(&sciErr, 0);
                        return 0;
                }
                intErr7 = getScalarDouble(pvApiCtx, piAddr7, &n);
                if(intErr7)
                {
                        return intErr7;
                }	
		
		try{		
		vector<Mat> images;
		retrieveImage(img1, 8);
		retrieveImage(img2, 9);
		retrieveImage(img3, 10);

		img1.convertTo(img1,CV_8U);
		

		img2.convertTo(img2,CV_8U);
		

		img3.convertTo(img3,CV_8U);
		
		Mat img4,img5;
		//denoise_TVL1(src,dst,1,30);
		if(n == 3)
			{

				images.push_back(img1);
				images.push_back(img2);
				images.push_back(img3);
				
				sciprint("yo");
				fastNlMeansDenoisingColoredMulti(images, dst, imgToDenoiseIndex, temporalWindowSize, filterStrength, filterStrengthColor,templateWindowSize, searchWindowSize);
			}
		else if(n == 4)
			{
				
				retrieveImage(img4, 10);

				img4.convertTo(img4,CV_8U);
				

				images.push_back(img1);
                                images.push_back(img2);
                                images.push_back(img3);
				images.push_back(img4);				

                                fastNlMeansDenoisingColoredMulti(images, dst, imgToDenoiseIndex, temporalWindowSize, filterStrength, filterStrengthColor,templateWindowSize, searchWindowSize);
			}
		else if(n == 5)
			{
				
				retrieveImage(img4, 10);
				retrieveImage(img5, 11);

				img4.convertTo(img4,CV_8U);
				img5.convertTo(img5,CV_8U);
				
				images.push_back(img1);
                                images.push_back(img2);
                                images.push_back(img3);
                                images.push_back(img4);
				images.push_back(img5);

                                fastNlMeansDenoisingColoredMulti(images, dst, imgToDenoiseIndex, temporalWindowSize, filterStrength, filterStrengthColor, templateWindowSize, searchWindowSize);
			}
		else
			{
				Scierror(999, "Wrong input for 'number of images'.", 1);
			}

			

		}
		catch(Exception& e)
		{
			const char* err=e.what();
			sciprint("%s",err);
		}

		string tempstring = type2str(dst.type());
           	char* checker = (char *)malloc(tempstring.size() + 1);
            	memcpy(checker, tempstring.c_str(), tempstring.size() + 1);
            	returnImage(checker,dst,1);
            	AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
            	ReturnArguments(pvApiCtx);	

		return 0; 

	}
}
