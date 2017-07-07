/*
This is the .cpp gateway file for the 'applyTransformer' scilab function.

OpenCV classes : 
1. Ptr< AffineTransformer > cv::createAffineTransformer (bool fullAffine)
2. Ptr< ThinPlateSplineShapeTransformer > cv::createThinPlateSplineShapeTransformer (double regularizationParameter=0) 

It includes the following OpenCV functions, belonging to the Shape Distance and Matching module of OpenCV 3.0.0 : 
1. estimateTransformation (InputArray transformingShape, InputArray targetShape, std::vector< DMatch > &matches)
   Estimate the transformation parameters of the current transformer algorithm, based on point matches.
2. warpImage (InputArray transformingImage, OutputArray output, int flags=INTER_LINEAR, int borderMode=BORDER_CONSTANT, const Scalar &borderValue=Scalar()) 
   Apply a transformation, given a pre-estimated transformation parameters, to an Image.

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
#include"opencv2/xfeatures2d.hpp"
#include"opencv2/core/utility.hpp"
#include<string>
#include<iostream>
#include<cstdlib>

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

	int opencv_applyTransformer(char *fname, unsigned long fname_len)
	{
		// Error management variable
		SciErr sciErr;
		int i;


		// variables required to read argument #3
		int *piAddr3 = NULL;
		int intErr3 = 0;
		double typeOfMethod = 0;

		// variables required to read argument #4
		int *piAddr4 = NULL;
		int intErr4 = 0;
		double hessianThreshold = 0;

		// variables required to read argument #5
		int *piAddr5 = NULL;
		int intErr5 = 0;
		double rpTPS = 0;

		// variables required to read argument #6
		int *piAddr6 = NULL;
		int intErr6 = 0;
		int sfAffine = false;

		// Checking number of input and output arguments (enviromnet variable, min arguments, max arguments) 
		CheckInputArgument(pvApiCtx, 6, 6);
		CheckOutputArgument(pvApiCtx, 1, 1);

		Mat img1, img2;

		// retrieving the input images
		int a = retrieveImage(img1, 1);
		if(a == 0)
		{
			sciprint("Error while retrieving the image1.");
			return 0;
		}
		int b = retrieveImage(img2, 2);
		if(b == 0)
		{
			sciprint("Error while retrieving the image2.");
		}
			

		// to get the argument #3
		sciErr = getVarAddressFromPosition(pvApiCtx, 3, &piAddr3);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
		intErr3 = getScalarDouble(pvApiCtx, piAddr3, &typeOfMethod);
		if(intErr3)
		{
			Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 3);
			return -1;
		}

		// to get the argument #4
		sciErr = getVarAddressFromPosition(pvApiCtx, 4, &piAddr4);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
		intErr4 = getScalarDouble(pvApiCtx, piAddr4, &hessianThreshold);
		if(intErr4)
		{
			Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 4);
			return -1;
		}

		// to get the argument #5
		sciErr = getVarAddressFromPosition(pvApiCtx, 5, &piAddr5);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
		intErr5 = getScalarDouble(pvApiCtx, piAddr5, &rpTPS);
		if(intErr5)
		{
			Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 5);
			return -1;
		}

		// to get the argument #6
		sciErr = getVarAddressFromPosition(pvApiCtx, 6, &piAddr6);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
		intErr6 = getScalarBoolean(pvApiCtx, piAddr6, &sfAffine);
		if(intErr6)
		{
			Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 6);
			return -1;
		} 

		
		Mat img_matches;
		Mat image1, image2;
	
		try{

			// OpenCV functionalities 

			// converting the input images to 8-bit and 1 channel images
			img1.convertTo(img1, CV_8U);
			cvtColor(img1, image1, CV_BGR2GRAY);

			img2.convertTo(img2, CV_8U);
			cvtColor(img2, image2, CV_BGR2GRAY);

			// detecting keypoints & computing descriptors
   	 		Ptr<SURF> surf = SURF::create(hessianThreshold);
    		
			vector<KeyPoint> keypoints1, keypoints2;
    			Mat descriptors1, descriptors2;
    
			surf->detectAndCompute(image1, Mat(), keypoints1, descriptors1);
    			surf->detectAndCompute(image2, Mat(), keypoints2, descriptors2);

			// matching descriptors
    			BFMatcher matcher(surf->defaultNorm());
    			vector<DMatch> matches;
    			matcher.match(descriptors1, descriptors2, matches);
		
			// extract points
    			vector<Point2f> pts1, pts2;
    			for (size_t ii=0; ii<keypoints1.size(); ii++)
        			pts1.push_back( keypoints1[ii].pt );
    			for (size_t ii=0; ii<keypoints2.size(); ii++)
        			pts2.push_back( keypoints2[ii].pt );

		
			// apply shape transformation
			if(typeOfMethod == 1)
			{
				// Affine transformation

				Ptr<AffineTransformer> model = createAffineTransformer(sfAffine);

				model->estimateTransformation(pts1, pts2, matches);
				model->warpImage(image2, image2);
			}
			else if(typeOfMethod == 2)
			{
				// TPS shape transformation
			
				Ptr<ThinPlateSplineShapeTransformer> model = createThinPlateSplineShapeTransformer(rpTPS);

                		model->estimateTransformation(pts1, pts2, matches);
				model->warpImage(image2, image2);
			}
			else
			{
				// incorrect input parameter of type-of-method

				Scierror(999, "Wrong input for Argument #3. Use '1' for 'Affine' and '2' for 'TPS' \n");
				return 0;
			}

		}
		catch(Exception& e)
		{
			const char* err=e.what();
			Scierror(999, "%s", err);
		}		


		// to return the output transformed image
		string tempstring1 = type2str(image2.type());
		char *checker1;
		checker1 = (char *)malloc(tempstring1.size() + 1);
		memcpy(checker1, tempstring1.c_str(), tempstring1.size() + 1);
		returnImage(checker1, image2, 1);
		free(checker1);
		AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;

		ReturnArguments(pvApiCtx);

		return 0;
	}
}
	
