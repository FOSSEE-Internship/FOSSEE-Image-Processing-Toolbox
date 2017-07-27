/*
This is the .cpp gateway file for the 'distanceExtractor' scilab function. Overloads two types of distance extractors :
OpenCV classes:
1. Ptr< ShapeContextDistanceExtractor > cv::createShapeContextDistanceExtractor (int nAngularBins=12, int nRadialBins=4, float innerRadius=0.2f, float outerRadius=2, int iterations=3, const Ptr< HistogramCostExtractor > &comparer=createChiHistogramCostExtractor(), const Ptr< ShapeTransformer > &transformer=createThinPlateSplineShapeTransformer())
2. Ptr< ThinPlateSplineShapeTransformer > cv::createThinPlateSplineShapeTransformer (double regularizationParameter=0)

It includes the following OpenCV functions, belonging to the Shape Distance and Matching module of OpenCV 3.0.0 : 
1. computeDistance (InputArray contour1, InputArray contour2)
   Compute the shape distance between two shapes defined by its contours.
*/

#include<iostream>
#include<string>
#include<algorithm>
#include<cstdlib>
#include<numeric>
#include"opencv2/opencv.hpp"
#include"opencv2/shape/shape_distance.hpp"
#include"opencv2/shape/shape_transformer.hpp"
#include"opencv2/shape/shape_transformer.hpp"
#include"opencv2/shape/hist_cost.hpp"

using namespace cv;
using namespace std;

extern "C"
{
	#include"api_scilab.h"
	#include"Scierror.h"
	#include"BOOL.h"
	#include<localization.h>
	#include"sciprint.h"
	#include"../common.h"

	static vector<Point> sampleContour(const Mat& image, int n=300)
	{
		vector<vector<Point> > _contours;
		vector<Point> all_points;
		findContours(image, _contours, RETR_LIST, CHAIN_APPROX_NONE);
		for(size_t i=0; i< _contours.size(); i++)
		{
			for(size_t j=0;j<_contours[i].size();j++)
			{
				all_points.push_back(_contours[i][j]);
			}
		}

		// if too little points, replicate them
		int dummy = 0;
		for(int add=(int)all_points.size();add<n;add++)
		{
			// adding dummy values
			all_points.push_back(all_points[dummy++]);
		}
	
		// sample uniformly
		random_shuffle(all_points.begin(), all_points.end());
		vector<Point> sampled;
		for(int i=0;i<n;i++)
		{
			sampled.push_back(all_points[i]);
		}
	
		return sampled;
	}	

	int opencv_distanceExtractor(char *fname, unsigned long fname_len)
	{

		// error management variable
		SciErr sciErr;
		int i;


		int intErr;

		// variables required to read argument #3
		int *piAddr3 = NULL;
		int intErr3 = 0;
		double typeOfMethod = 0;

		// variables required to read argument #4
		int *piAddr4 = NULL;
		int intErr4 = 0;
		double nAngularBins = 0;

		// variables required to read argument #5
		int *piAddr5 = NULL;
		int intErr5 = 0;
		double innerRadius = 0;

		// variables required to read argument #6
		int *piAddr6 = NULL;
		int intErr6 = 0;
		double nRadialBins = 0;		

		// variables required to read argument #7
		int *piAddr7 = NULL;
		int intErr7 = 0;
		double outerRadius = 0;

		// variables required to read argument #8
		int *piAddr8 = NULL;
		int intErr8 = 0;
		double iterations = 0;

		// variables required to read argument #9
		int *piAddr9 = NULL;
		int intErr9 = 0;
		double nDummies = 0;

		// variables required to read argument #10
		int *piAddr10 = NULL;
		int intErr10 = 0;
		double dC = 0;

		// variables required to read argument #11
		int *piAddr11 = NULL;
		int intErr11 = 0;
		double rpTps = 0;


		// Checking number of input and output arguments (enviromnet variable, min arguments, max arguments)
		CheckInputArgument(pvApiCtx, 3, 8);
		CheckOutputArgument(pvApiCtx, 1, 1);

		Mat img1, img2;

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
		
		// to get argument #3
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
	
		// returned value
		double dist;

		Size sz2Sh(300, 300);

		Mat image1, image2;
    		


		try
		{
			img1.convertTo(img1, CV_8U);
			cvtColor(img1, image1, CV_BGR2GRAY);

			img2.convertTo(img2, CV_8U);
			cvtColor(img2, image2, CV_BGR2GRAY);
	
        	        vector<Point> c1 = sampleContour(image1);
        	        vector<Point> c2 = sampleContour(image2);

			// OpenCV functionalitites

			// apply Shape Distant Extractor
			if(typeOfMethod == 1)
			{
				// Shape Context Distant Extractor


				// to get the argument #4
				sciErr = getVarAddressFromPosition(pvApiCtx, 4, &piAddr4);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErr4 = getScalarDouble(pvApiCtx, piAddr4, &nAngularBins);
				if(intErr4)
				{
					return intErr4;
				}

				// to get the argument #5
				sciErr = getVarAddressFromPosition(pvApiCtx, 5, &piAddr5);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErr5 = getScalarDouble(pvApiCtx, piAddr5, &innerRadius);
				if(intErr5)
				{
					return intErr5;
				}
			

				// to get the argument #6
				sciErr = getVarAddressFromPosition(pvApiCtx, 6, &piAddr6);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErr6 = getScalarDouble(pvApiCtx, piAddr6, &nRadialBins);
				if(intErr6)
				{
					return intErr6;
				} 
	
				// to get the argument #7
				sciErr = getVarAddressFromPosition(pvApiCtx, 7, &piAddr7);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErr7 = getScalarDouble(pvApiCtx, piAddr7, &outerRadius);
				if(intErr7)
				{
					return intErr7;
				}
	
				// to get the argument #8
				sciErr = getVarAddressFromPosition(pvApiCtx, 8, &piAddr8);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErr8 = getScalarDouble(pvApiCtx, piAddr8, &iterations);
				if(intErr8)
				{
					return intErr8;
				}

				// to get the argument #9
				sciErr = getVarAddressFromPosition(pvApiCtx, 9, &piAddr9);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErr9 = getScalarDouble(pvApiCtx, piAddr9, &nDummies);
				if(intErr9)
				{
					return intErr9;
				} 
	
				// to get the argument #10
				sciErr = getVarAddressFromPosition(pvApiCtx, 10, &piAddr10);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErr10 = getScalarDouble(pvApiCtx, piAddr10, &dC);
				if(intErr10)
				{
					return intErr10;
				}
	
				// to get the argument #11
				sciErr = getVarAddressFromPosition(pvApiCtx, 11, &piAddr11);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErr11 = getScalarDouble(pvApiCtx, piAddr11, &rpTps);
				if(intErr11)
				{
					return intErr11;
				}
		
				
				const Ptr<HistogramCostExtractor>& comparer = createChiHistogramCostExtractor(nDummies, dC);
				// Smart pointer to a ShapeTransformer, an algorithm that defines the aligning transformation. 
				const Ptr<ShapeTransformer>& transformer = createThinPlateSplineShapeTransformer(rpTps);	


				Ptr<ShapeContextDistanceExtractor> model = createShapeContextDistanceExtractor(nAngularBins, nRadialBins, innerRadius, outerRadius, iterations, comparer, transformer );


				dist = model->computeDistance(c1, c2);
			}
			else if(typeOfMethod == 2)
			{
				// Hausdorff Distant Extractor

				Ptr<HausdorffDistanceExtractor> model = createHausdorffDistanceExtractor();

				dist = model->computeDistance(c1, c2);
			
				cout<<dist<<endl;
			}
			else
			{
				// incorrect input parameter of type-of-method
				
				Scierror(999, "Wrong input for Argument #3. Use '1' for 'Shape Context Distance Extractor' and '2' for 'Hausdorff Distance Extractor' \n");
				
				return 0;
			}
		}	
		catch(Exception& e)
		{
			const char* err = e.what();
			Scierror(999, "%s", err);
		}

		intErr = createScalarDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 1, dist);
		if(intErr)
		{
			return intErr;
		}

		// returning output variable to scilab		
		AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;

		ReturnArguments(pvApiCtx);

		return 0;
	}
}
