// Authors 
// Ashish Manatosh Barik, Shubham Lohakare
//
#include<iostream>
#include"opencv2/shape.hpp"
#include"opencv2/imgcodecs.hpp"
#include"opencv2/highgui.hpp"
#include"opencv2/imgproc.hpp"
#include"opencv2/features2d.hpp"
#include"opencv2/xfeatures2d.hpp"
#include"opencv2/xfeatures2d/nonfree.hpp"
#include<opencv2/core/utility.hpp>
#include"opencv2/core/core.hpp"
#include<string>
#include<vector>
#include<stdio.h>

using namespace cv;
using namespace std;
using namespace cv::xfeatures2d;

extern "C"
{
	#include"api_scilab.h"
	#include"Scierror.h"
	#include"BOOL.h"
	#include<localization.h>
	#include"sciprint.h"
	#include"../common.h"

	int opencv_detectSIFTFeatures(char* fname, unsigned long fname_len)
	{

		// error management variable
		SciErr sciErr;

		// Checking number of input and output arguments (enviromnet variable, min arguments, max arguments)
		CheckInputArgument(pvApiCtx, 1, 6);
		CheckOutputArgument(pvApiCtx, 1, 5);

		// input image
		Mat img;

		// to retrieve the input image from the hypermat that is passed in the scilab function
		retrieveImage(img, 1);
		
		// variables required to read varDouble #1
		int *piAddrVar1 = NULL;
		int intErrVar1 = 0;
		double varDouble1 = 0;

		// variables required to read varDouble #2
		int *piAddrVar2 = NULL;
		int intErrVar2 = 0;
		double varDouble2 = 0;

		// variables required to read varDouble #3
		int *piAddrVar3 = NULL;
		int intErrVar3 = 0;
		double varDouble3 = 0;

		// variables required to read varDouble #4
		int *piAddrVar4 = NULL;
		int intErrVar4 = 0;
		double varDouble4 = 0;

		// variables required to read varDouble #5
		int *piAddrVar5 = NULL;
		int intErrVar5 = 0;
		double varDouble5 = 0;

		// to get the number of input argument passed in the scilab function
		int n = *getNbInputArgument(pvApiCtx);

		// image used after converting to grayscale image
		Mat image;
		// keypoints of the input image
		vector<KeyPoint> keypoints;
		// descriptors of the input image
		Mat descriptors;

		// matrix used to output the keypoints extracted from the input image
		double *LocationData = NULL;

		// matrix used to output the descriptors extracted from the input image
		double *featureVector = NULL;  
        	int feature_rows=0;
        	int feature_cols=0;
		double numBits = 0;
        	double numFeatures = 0;

		try
		{
			// OpenCV functionalities 

			// converting the input images to 8-bit and 1 channel images
			img.convertTo(img, CV_8U);
			cvtColor(img, image, CV_BGR2GRAY);

			if(n == 1)
			{
				// using default OpenCV function values

				// SIFT object
				Ptr<SIFT> model = cv::xfeatures2d::SIFT::create();	

				// detecting keypoints and computing descriptors for the respective images
				model->detectAndCompute(image, Mat(), keypoints, descriptors, false);
	
			}
			else if(n == 6)
			{
				// user passed values for OpenCV functions				

				// to get varDouble #1
				sciErr = getVarAddressFromPosition(pvApiCtx, 2, &piAddrVar1);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErrVar1 = getScalarDouble(pvApiCtx, piAddrVar1, &varDouble1);
				if(intErrVar1)
				{
					return intErrVar1;
				}

				// to get varDouble #2
				sciErr = getVarAddressFromPosition(pvApiCtx, 3, &piAddrVar2);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErrVar2 = getScalarDouble(pvApiCtx, piAddrVar2, &varDouble2);
				if(intErrVar2)
				{
					return intErrVar2;
				}

				// to get varDouble #3
				sciErr = getVarAddressFromPosition(pvApiCtx, 4, &piAddrVar3);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErrVar3 = getScalarDouble(pvApiCtx, piAddrVar3, &varDouble3);
				if(intErrVar3)
				{
					return intErrVar3;
				}
				
				// to get varDouble #4
				sciErr = getVarAddressFromPosition(pvApiCtx, 5, &piAddrVar4);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErrVar4 = getScalarDouble(pvApiCtx, piAddrVar4, &varDouble4);
				if(intErrVar4)
				{
					return intErrVar4;
				}

				// to get varDouble #5
				sciErr = getVarAddressFromPosition(pvApiCtx, 6, &piAddrVar5);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErrVar5 = getScalarDouble(pvApiCtx, piAddrVar5, &varDouble5);
				if(intErrVar5)
				{
					return intErrVar5;
				}

				// SIFT object
				Ptr<SIFT> model = cv::xfeatures2d::SIFT::create(varDouble1, varDouble2, varDouble3, varDouble4, varDouble5);	

				// detecting keypoints and computing descriptors for the respective images
				model->detectAndCompute(image, Mat(), keypoints, descriptors, false);

			}				
		}
		catch(Exception& e)
		{
			const char* err=e.what();
			Scierror(999, "%s", err);
		}

		// to pass the computed desciptors as output in the form of a matrix
		numBits = descriptors.size[1];
            	numFeatures = descriptors.size[0];
		featureVector = (double*)malloc(sizeof(double)*descriptors.size[0]*descriptors.size[1]);
	        for( int i=0; i<descriptors.size[0]; i++)
		{
                	for( int j=0; j<descriptors.size[1]; j++)
                	{
                    		*(featureVector + j*descriptors.size[0] + i) = int( descriptors.at<uchar>(i,j));
                	}
		}
                feature_rows = descriptors.size[0];
                feature_cols = descriptors.size[1];    

		// to pass the detected keypoints as output in the form of a matrix
		int size = keypoints.size();
		LocationData = (double *)malloc(sizeof(double) * size * 2);
   		for(int i = 0; i < size; i++)
    		{
        		LocationData[i] = keypoints[i].pt.x;
        		LocationData[i + size] = keypoints[i].pt.y;
    		}

		// descriptor
		sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 1, feature_rows, feature_cols, featureVector);
        	if(sciErr.iErr)
	        {
        	    printError(&sciErr, 0);
        	    return 0;
        	}

		// descriptor bits
    		sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 2, 1, 1, &numBits);
        	if(sciErr.iErr)
        	{
            		printError(&sciErr, 0);
            		return 0;
        	}

		// number of descriptors
		sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 3, 1, 1, &numFeatures);
	        if(sciErr.iErr)
	        {
	            printError(&sciErr, 0);
	            return 0;
	        } 
    
		// keypoints
	        sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 4, size, 2, LocationData);
	        if(sciErr.iErr)
    		{
        		printError(&sciErr, 0);
        		return 0;
    		}

		// number of keypoints
		createScalarInteger32(pvApiCtx,nbInputArgument(pvApiCtx) + 5, size);

		// to return output to scilab 
	        for(int i=1;i<=5;i++)
	        {
 			AssignOutputVariable(pvApiCtx, i) = nbInputArgument(pvApiCtx) + i;   	
    		}

		// to commit the new variables to the Scilab engine	
    		ReturnArguments(pvApiCtx);

		return 0;
	}
    
}
    
