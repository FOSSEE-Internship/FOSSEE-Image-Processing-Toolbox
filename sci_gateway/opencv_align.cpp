/*
This is the .cpp gateway file for the 'align' scilab function.

OpenCV classes : 
1. Ptr<AlignMTB> cv::createAlignMTB (int max_bits=6, int exclude_range=4, bool cut=true)
   // Creates AlignMTB object. 

It includes the following OpenCV functions, belonging to the Photo module of OpenCV 3.0.0 : 
1. void process (InputArrayOfArrays src, std::vector< Mat > &dst)
   // Aligns images.

*/

#include<numeric>
#include"opencv2/core/core.hpp"
#include"opencv2/highgui/highgui.hpp"
#include"opencv2/opencv.hpp"
#include"opencv2/shape.hpp"
#include"opencv2/imgcodecs.hpp"
#include"opencv2/imgproc/imgproc.hpp"
#include"opencv2/core/utility.hpp"
#include<string>
#include<iostream>
#include<cstdlib>
#include "opencv2/photo.hpp"

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

	int opencv_align(char* fname, unsigned long fname_len)
	{
		// Error management variable
		SciErr sciErr;

		Mat img1, img2, img3, img4, img5, img6;

		// retrieving images 
		retrieveImage(img1, 5);
		retrieveImage(img2, 6);
		retrieveImage(img3, 7);
		
		// variables required to read argument #1
		int *piAddr1 = NULL;
		int intErr1 = 0;
		double maxBits = 0;

		// variables required to read argument #2
		int *piAddr2 = NULL;
		int intErr2 = 0;
		double excludeRange = 0;

		// variables required to read argument #3
		int *piAddr3 = NULL;
		int intErr3 = 0;
		int cut = true;

		// variables required to read argument #4
		int *piAddr4 = NULL;
		int intErr4 = 0;
		double num = 0;

		// Checking number of input and output arguments (enviromnet variable, min arguments, max arguments) 
		CheckInputArgument(pvApiCtx, 7, 10);
		CheckOutputArgument(pvApiCtx, 3, 5);

		// to get the argument #1
		sciErr = getVarAddressFromPosition(pvApiCtx, 1, &piAddr1);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
		intErr1 = getScalarDouble(pvApiCtx, piAddr1, &maxBits);
		if(intErr1)
		{
			Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 1);
			return -1;
		}

		// to get the argument #2
		sciErr = getVarAddressFromPosition(pvApiCtx, 2, &piAddr2);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
		intErr2 = getScalarDouble(pvApiCtx, piAddr2, &excludeRange);
		if(intErr2)
		{
			Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 2);
			return -1;
		}

		// to get the argument #3
		sciErr = getVarAddressFromPosition(pvApiCtx, 3, &piAddr3);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
		intErr3 = getScalarBoolean(pvApiCtx, piAddr3, &cut);
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
		intErr4 = getScalarDouble(pvApiCtx, piAddr4, &num);
		if(intErr4)
		{
			Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 4);
			return -1;
		}

		vector<Mat> images;
		// output vector of aligned images
		vector<Mat> dst;
		
		
		try
		{
			// OpenCV functionalities

			// converting the input images to 8-bit and 1 channel images
			img1.convertTo(img1, CV_8U);
			img2.convertTo(img2, CV_8U);
			img3.convertTo(img3, CV_8U);

			// creating the AlignMTB object
			Ptr<AlignMTB> model = createAlignMTB (maxBits, excludeRange, cut);

			if(num == 3)
			{
				// pushing the images into vector<Mat>
				images.push_back(img1);
				images.push_back(img2);
				images.push_back(img3);

				// aligning images
				model->process(images, dst);

				// to return the output1 image
				string tempstring1 = type2str(dst[0].type());
				char *checker1;
				checker1 = (char *)malloc(tempstring1.size() + 1);
				memcpy(checker1, tempstring1.c_str(), tempstring1.size() + 1);
				returnImage(checker1, dst[0], 1);
				free(checker1);
				AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
	
				// to return the output2 image
				string tempstring2 = type2str(dst[1].type());
				char *checker2;
				checker2 = (char *)malloc(tempstring2.size() + 1);
				memcpy(checker2, tempstring2.c_str(), tempstring2.size() + 1);
				returnImage(checker2, dst[1], 2);
				free(checker2);
				AssignOutputVariable(pvApiCtx, 2) = nbInputArgument(pvApiCtx) + 2;

				// to return the output3 image
				string tempstring3 = type2str(dst[2].type());
				char *checker3;
				checker3 = (char *)malloc(tempstring2.size() + 1);
				memcpy(checker3, tempstring2.c_str(), tempstring2.size() + 1);
				returnImage(checker3, dst[2], 3);
				free(checker3);
				AssignOutputVariable(pvApiCtx, 3) = nbInputArgument(pvApiCtx) + 3;
				
			}
			else if(num == 4)
			{
				retrieveImage(img4, 8);

				img4.convertTo(img4, CV_8U);

				images.push_back(img1);
				images.push_back(img2);
				images.push_back(img3);
				images.push_back(img4);

				model->process(images, dst);

				// to return the output1 image
				string tempstring1 = type2str(dst[0].type());
				char *checker1;
				checker1 = (char *)malloc(tempstring1.size() + 1);
				memcpy(checker1, tempstring1.c_str(), tempstring1.size() + 1);
				returnImage(checker1, dst[0], 1);
				free(checker1);
				AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
	
				// to return the output2 image
				string tempstring2 = type2str(dst[1].type());
				char *checker2;
				checker2 = (char *)malloc(tempstring2.size() + 1);
				memcpy(checker2, tempstring2.c_str(), tempstring2.size() + 1);
				returnImage(checker2, dst[1], 2);
				free(checker2);
				AssignOutputVariable(pvApiCtx, 2) = nbInputArgument(pvApiCtx) + 2;

				// to return the output3 image
				string tempstring3 = type2str(dst[2].type());
				char *checker3;
				checker3 = (char *)malloc(tempstring3.size() + 1);
				memcpy(checker3, tempstring3.c_str(), tempstring3.size() + 1);
				returnImage(checker3, dst[2], 3);
				free(checker3);
				AssignOutputVariable(pvApiCtx, 3) = nbInputArgument(pvApiCtx) + 3;

				// to return the output4 image
				string tempstring4 = type2str(dst[3].type());
				char *checker4;
				checker4 = (char *)malloc(tempstring4.size() + 1);
				memcpy(checker4, tempstring4.c_str(), tempstring4.size() + 1);
				returnImage(checker4, dst[3], 4);
				free(checker4);
				AssignOutputVariable(pvApiCtx, 4) = nbInputArgument(pvApiCtx) + 4;



			}
			else if(num == 5)
			{
				retrieveImage(img4, 8);
				retrieveImage(img5, 9);

				img4.convertTo(img4, CV_8U);
				img5.convertTo(img5, CV_8U);

				images.push_back(img1);
				images.push_back(img2);
				images.push_back(img3);
				images.push_back(img4);
				images.push_back(img5);

				model->process(images, dst);

				// to return the output1 image
				string tempstring1 = type2str(dst[0].type());
				char *checker1;
				checker1 = (char *)malloc(tempstring1.size() + 1);
				memcpy(checker1, tempstring1.c_str(), tempstring1.size() + 1);
				returnImage(checker1, dst[0], 1);
				free(checker1);
				AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
	
				// to return the output2 image
				string tempstring2 = type2str(dst[1].type());
				char *checker2;
				checker2 = (char *)malloc(tempstring2.size() + 1);
				memcpy(checker2, tempstring2.c_str(), tempstring2.size() + 1);
				returnImage(checker2, dst[1], 2);
				free(checker2);
				AssignOutputVariable(pvApiCtx, 2) = nbInputArgument(pvApiCtx) + 2;

				// to return the output3 image
				string tempstring3 = type2str(dst[2].type());
				char *checker3;
				checker3 = (char *)malloc(tempstring3.size() + 1);
				memcpy(checker3, tempstring3.c_str(), tempstring3.size() + 1);
				returnImage(checker3, dst[2], 3);
				free(checker3);
				AssignOutputVariable(pvApiCtx, 3) = nbInputArgument(pvApiCtx) + 3;

				// to return the output4 image
				string tempstring4 = type2str(dst[3].type());
				char *checker4;
				checker4 = (char *)malloc(tempstring4.size() + 1);
				memcpy(checker4, tempstring4.c_str(), tempstring4.size() + 1);
				returnImage(checker4, dst[3], 4);
				free(checker4);
				AssignOutputVariable(pvApiCtx, 4) = nbInputArgument(pvApiCtx) + 4;

				// to return the output5 image
				string tempstring5 = type2str(dst[4].type());
				char *checker5;
				checker5 = (char *)malloc(tempstring5.size() + 1);
				memcpy(checker5, tempstring5.c_str(), tempstring5.size() + 1);
				returnImage(checker5, dst[4], 5);
				free(checker5);
				AssignOutputVariable(pvApiCtx, 5) = nbInputArgument(pvApiCtx) + 5;


			}
			else if(num == 6)
			{
				retrieveImage(img4, 8);
				retrieveImage(img5, 9);
				retrieveImage(img6, 10);

				img4.convertTo(img4, CV_8U);
				img5.convertTo(img5, CV_8U);	
				img6.convertTo(img6, CV_8U);

				images.push_back(img1);
				images.push_back(img2);
				images.push_back(img3);
				images.push_back(img4);
				images.push_back(img5);
				images.push_back(img6);

				model->process(images, dst);

				// to return the output1 image
				string tempstring1 = type2str(dst[0].type());
				char *checker1;
				checker1 = (char *)malloc(tempstring1.size() + 1);
				memcpy(checker1, tempstring1.c_str(), tempstring1.size() + 1);
				returnImage(checker1, dst[0], 1);
				free(checker1);
				AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
	
				// to return the output2 image
				string tempstring2 = type2str(dst[1].type());
				char *checker2;
				checker2 = (char *)malloc(tempstring2.size() + 1);
				memcpy(checker2, tempstring2.c_str(), tempstring2.size() + 1);
				returnImage(checker2, dst[1], 2);
				free(checker2);
				AssignOutputVariable(pvApiCtx, 2) = nbInputArgument(pvApiCtx) + 2;

				// to return the output3 image
				string tempstring3 = type2str(dst[2].type());
				char *checker3;
				checker3 = (char *)malloc(tempstring3.size() + 1);
				memcpy(checker3, tempstring3.c_str(), tempstring3.size() + 1);
				returnImage(checker3, dst[2], 3);
				free(checker3);
				AssignOutputVariable(pvApiCtx, 3) = nbInputArgument(pvApiCtx) + 3;

				// to return the output4 image
				string tempstring4 = type2str(dst[3].type());
				char *checker4;
				checker4 = (char *)malloc(tempstring4.size() + 1);
				memcpy(checker4, tempstring4.c_str(), tempstring4.size() + 1);
				returnImage(checker4, dst[3], 4);
				free(checker4);
				AssignOutputVariable(pvApiCtx, 4) = nbInputArgument(pvApiCtx) + 4;

				// to return the output5 image
				string tempstring5 = type2str(dst[4].type());
				char *checker5;
				checker5 = (char *)malloc(tempstring5.size() + 1);
				memcpy(checker5, tempstring5.c_str(), tempstring5.size() + 1);
				returnImage(checker5, dst[4], 5);
				free(checker5);
				AssignOutputVariable(pvApiCtx, 5) = nbInputArgument(pvApiCtx) + 5;

				// to return the output5 image
				string tempstring6 = type2str(dst[5].type());
				char *checker6;
				checker6 = (char *)malloc(tempstring6.size() + 1);
				memcpy(checker6, tempstring6.c_str(), tempstring6.size() + 1);
				returnImage(checker6, dst[5], 6);
				free(checker6);
				AssignOutputVariable(pvApiCtx, 6) = nbInputArgument(pvApiCtx) + 6;



			}
			else
			{
				Scierror(999, "Wrong argument #4, Number of input images. \n", 1);
			}
		}
		catch(Exception& e)
		{
			const char* err=e.what();
			Scierror(999, "%s", err);
		}		

		ReturnArguments(pvApiCtx);

		return 0;
	}
}
