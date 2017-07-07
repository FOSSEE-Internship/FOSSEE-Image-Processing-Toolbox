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

	int opencv_detectDAISYDescriptors(char* fname, unsigned long fname_len)
	{

		// error management variable
		SciErr sciErr;

		// Checking number of input and output arguments (enviromnet variable, min arguments, max arguments)
		CheckInputArgument(pvApiCtx, 1, 14);
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

		// variables required to read varDouble #6
		int *piAddrVar6 = NULL;
		int intErrVar6 = 0;
		double varDouble6 = 0;

		// variables required to read varDouble #7
		int *piAddrVar7 = NULL;
		int intErrVar7 = 0;
		double varDouble7 = 0;

		// variables required to read varDouble #8
		int *piAddrVar8 = NULL;
		int intErrVar8 = 0;
		double varDouble8 = 0;

		// variables required to read varDouble #9
		int *piAddrVar9 = NULL;
		int intErrVar9 = 0;
		double varDouble9 = 0;

		// variables required to read varDouble #10
		int *piAddrVar10 = NULL;
		int intErrVar10 = 0;
		double varDouble10 = 0;

		// variable required to read varMatrix #1
		int *piAddrVar11 = NULL;
		int intErrVar11 = 0;
		int r = 0, c = 0;
		double *hMat;

		// variables required to read varBool #1
		int *piAddrVar12 = NULL;
		int intErrVar12 = 0;
		int varBool1 = true;	

		// variables required to read varBool #2
		int *piAddrVar13 = NULL;
		int intErrVar13 = 0;
		int varBool2 = false;

		// to get the number of input argument passed in the scilab function		
		int n = *getNbInputArgument(pvApiCtx);

		// keypoints of the input image
		vector<KeyPoint> keypoints;

		// image used after converting to grayscale image
		Mat image;
		// descriptors of the input image
		Mat descriptors;

		// matrix used to output the descriptors extracted from the input image
		double *featureVector = NULL;  
        	int feature_rows=0;
        	int feature_cols=0;
		double numBits = 0;
        	double numFeatures = 0;

		// matrix used to output the keypoints extracted from the input image
		double *LocationData = NULL;

		try
		{
			// OpenCV functionalities 

			// converting the input images to 8-bit and 1 channel images
			img.convertTo(img, CV_8U);
			cvtColor(img, image, CV_BGR2GRAY);


			if(n == 1)
			{
				// using default OpenCV function values
				
				Ptr<StarDetector> star = cv::xfeatures2d::StarDetector::create();

				star->detect(image, keypoints);

				// object
				Ptr<DAISY> model = cv::xfeatures2d::DAISY::create();//using default value for H

				// computing descriptors
                    		model->compute(image, keypoints, descriptors);

			}
			else if(n == 13)
			{
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

				// to get varDouble #6
				sciErr = getVarAddressFromPosition(pvApiCtx, 7, &piAddrVar6);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErrVar6 = getScalarDouble(pvApiCtx, piAddrVar6, &varDouble6);
				if(intErrVar6)
				{
					return intErrVar6;
				}

				// to get varDouble #7
				sciErr = getVarAddressFromPosition(pvApiCtx, 8, &piAddrVar7);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErrVar7 = getScalarDouble(pvApiCtx, piAddrVar7, &varDouble7);
				if(intErrVar7)
				{
					return intErrVar7;
				}

	
				// to get varDouble #8
				sciErr = getVarAddressFromPosition(pvApiCtx, 9, &piAddrVar8);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErrVar8 = getScalarDouble(pvApiCtx, piAddrVar8, &varDouble8);
				if(intErrVar8)
				{
					return intErrVar8;
				}

				// to get varDouble #9
				sciErr = getVarAddressFromPosition(pvApiCtx, 10, &piAddrVar9);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErrVar9 = getScalarDouble(pvApiCtx, piAddrVar9, &varDouble9);
				if(intErrVar9)
				{
					return intErrVar9;
				}


				// to get varDouble #10
				sciErr = getVarAddressFromPosition(pvApiCtx, 11, &piAddrVar10);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErrVar10 = getScalarDouble(pvApiCtx, piAddrVar10, &varDouble10);
				if(intErrVar10)
				{
					return intErrVar10;
				}
				
				// to get varBool #1
				sciErr = getVarAddressFromPosition(pvApiCtx, 12, &piAddrVar12);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErrVar12 = getScalarBoolean(pvApiCtx, piAddrVar12, &varBool1);
				if(intErrVar12)
				{
					return intErrVar12;
				}

				// to get varBool #2
				sciErr = getVarAddressFromPosition(pvApiCtx, 13, &piAddrVar13);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErrVar13 = getScalarBoolean(pvApiCtx, piAddrVar13, &varBool2);
				if(intErrVar13)
				{
					return intErrVar13;
				}

				Ptr<StarDetector> star = cv::xfeatures2d::StarDetector::create(varDouble1, varDouble2, varDouble3, varDouble4, varDouble5);

				star->detect(image, keypoints);
	
				
				// object
				Ptr<DAISY> model = cv::xfeatures2d::DAISY::create((float)varDouble6, varDouble7, varDouble8, varDouble9, varDouble10, Mat(), varBool1, varBool2); //using user given value for H

				// computing descriptors
                    		model->compute(image, keypoints, descriptors);
				
			}
			else if(n == 14)
			{
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

				// to get varDouble #6
				sciErr = getVarAddressFromPosition(pvApiCtx, 7, &piAddrVar6);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErrVar6 = getScalarDouble(pvApiCtx, piAddrVar6, &varDouble6);
				if(intErrVar6)
				{
					return intErrVar6;
				}

				// to get varDouble #7
				sciErr = getVarAddressFromPosition(pvApiCtx, 8, &piAddrVar7);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErrVar7 = getScalarDouble(pvApiCtx, piAddrVar7, &varDouble7);
				if(intErrVar7)
				{
					return intErrVar7;
				}

	
				// to get varDouble #8
				sciErr = getVarAddressFromPosition(pvApiCtx, 9, &piAddrVar8);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErrVar8 = getScalarDouble(pvApiCtx, piAddrVar8, &varDouble8);
				if(intErrVar8)
				{
					return intErrVar8;
				}

				// to get varDouble #9
				sciErr = getVarAddressFromPosition(pvApiCtx, 10, &piAddrVar9);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErrVar9 = getScalarDouble(pvApiCtx, piAddrVar9, &varDouble9);
				if(intErrVar9)
				{
					return intErrVar9;
				}


				// to get varDouble #10
				sciErr = getVarAddressFromPosition(pvApiCtx, 11, &piAddrVar10);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErrVar10 = getScalarDouble(pvApiCtx, piAddrVar10, &varDouble10);
				if(intErrVar10)
				{
					return intErrVar10;
				}

				// reading homographic matrix
				/* get Address of inputs */
                      		sciErr = getVarAddressFromPosition(pvApiCtx, 12, &piAddrVar11);
                        	if (sciErr.iErr)
                       		{
                           		printError(&sciErr, 0);
                            		return 0;
                        	}
                    		/* Check that the first input argument is a real matrix (and not complex) */
                    		if( !isDoubleType(pvApiCtx, piAddrVar11) ||  isVarComplex(pvApiCtx, piAddrVar11) )
                    		{
                        		Scierror(999, " Wrong type for input argument #%d: A real matrix expected.\n", fname, 16);
                        		return 0;
                    		}
                    		/* get matrix */
                    		sciErr = getMatrixOfDouble(pvApiCtx, piAddrVar11, &r, &c, &hMat);
                    		if (sciErr.iErr)
                   		{
                        		printError(&sciErr, 0);
                       			return 0;
                    		}
                    		if(!(r==3 && c==3))
                    		{
                   			Scierror(999, " Wrong size for input argument #%d: A 3x3 real matrix expected.\n", fname, 16);
                    		}
                    		/*declare vector*/
                    		vector<double> Homography (r*c);
                   
                    		for(int i = 0; i < r; i++) 
                    		{
                       			for(int j = 0; j < c; j++) 
                       			{
                           			Homography.at(i*c + j) = hMat[i + j*r];
                           
                       			} 
                    		}      
				
				// to get varBool #1
				sciErr = getVarAddressFromPosition(pvApiCtx, 13, &piAddrVar12);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErrVar12 = getScalarBoolean(pvApiCtx, piAddrVar12, &varBool1);
				if(intErrVar12)
				{
					return intErrVar12;
				}

				// to get varBool #2
				sciErr = getVarAddressFromPosition(pvApiCtx, 14, &piAddrVar13);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErrVar13 = getScalarBoolean(pvApiCtx, piAddrVar13, &varBool2);
				if(intErrVar13)
				{
					return intErrVar13;
				}

				Ptr<StarDetector> star = cv::xfeatures2d::StarDetector::create(varDouble1, varDouble2, varDouble3, varDouble4, varDouble5);

				star->detect(image, keypoints);

				
				// object
				Ptr<DAISY> model = cv::xfeatures2d::DAISY::create((float)varDouble6, varDouble7, varDouble8, varDouble9, varDouble10, Homography , varBool1, varBool2); //using user given value for H

				// computing descriptors
                    		model->compute(image, keypoints, descriptors);

				
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
