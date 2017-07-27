/*
This is the .cpp gateway file for the 'align' scilab function.

OpenCV classes : 
1. Ptr< CalibrateDebevec > cv::createCalibrateDebevec (int samples=70, float lambda=10.0f, bool random=false)
2. Ptr< CalibrateRobertson > cv::createCalibrateRobertson (int max_iter=30, float threshold=0.01f)
3. Ptr< MergeDebevec > 	cv::createMergeDebevec ()
4. Ptr< MergeMertens > 	cv::createMergeMertens (float contrast_weight=1.0f, float saturation_weight=1.0f, float exposure_weight=0.0f)
5. Ptr< MergeRobertson > cv::createMergeRobertson ()
6. Ptr< Tonemap > cv::createTonemap (float gamma=1.0f)

It includes the following OpenCV functions, belonging to the Photo module of OpenCV 3.0.0 : 
1. void process (InputArrayOfArrays src, OutputArray dst, InputArray times)
2. void process (InputArrayOfArrays src, OutputArray dst, InputArray times, InputArray response)
3. void process (InputArrayOfArrays src, OutputArray dst)

*/

#include<iostream>
#include<string>
#include<cstdlib>
#include<numeric>
#include"opencv2/opencv.hpp"
#include"opencv2/core.hpp"
#include"opencv2/highgui.hpp"
#include"opencv2/imgproc.hpp"
#include"opencv2/photo.hpp"

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

	int opencv_makeHDR(char* fname, unsigned long fname_len)
	{

		// error management variable
		SciErr sciErr;

		// Checking number of input and output arguments (enviromnet variable, min arguments, max arguments)
		CheckInputArgument(pvApiCtx, 10, 17);
		CheckOutputArgument(pvApiCtx, 2, 2);

		// variables required to read argument #1
		int *piAddr1 = NULL;
		int intErr1 = 0;
		double typeOfMethod = 0;
		
		// variables required to read argument #2
		int *piAddr2 = NULL;
		int intErr2 = 0;
		double num = 0;

		// variables required to read exposure #1
		int *piAddrExp1 = NULL;
		int intErrExp1 = 0;
		double ex1 = 0;

		// variables required to read exposure #2
		int *piAddrExp2 = NULL;
		int intErrExp2 = 0;
		double ex2 = 0;

		// variables required to read exposure #3
		int *piAddrExp3 = NULL;
		int intErrExp3 = 0;
		double ex3 = 0;

		// variables required to read exposure #4
		int *piAddrExp4 = NULL;
		int intErrExp4 = 0;
		double ex4 = 0;

		// variables required to read exposure #5
		int *piAddrExp5 = NULL;
		int intErrExp5 = 0;
		double ex5 = 0;

		// variables required to read exposure #6
		int *piAddrExp6 = NULL;
		int intErrExp6 = 0;
		double ex6 = 0;

		// variables required to read argument #3
		int *piAddr3 = NULL;
		int intErr3 = 0;
		double var1 = 0;

		// variables required to read argument #4
		int *piAddr4 = NULL;
		int intErr4 = 0;
		double var2 = 0;

		// variables required to read argument #5
		int *piAddr5 = NULL;
		int intErr5 = 0;
		double var3 = 0;

		// variables required to read argument #6
		int *piAddr6 = NULL;
		int intErr6 = 0;
		int var4 = false;
		
		// to get argument #1
		sciErr = getVarAddressFromPosition(pvApiCtx, 1, &piAddr1);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
		intErr1 = getScalarDouble(pvApiCtx, piAddr1, &typeOfMethod);
		if(intErr1)
		{
			Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 1);
			return -1;
		}

		// to get argument #2
                sciErr = getVarAddressFromPosition(pvApiCtx, 2, &piAddr2);
                if(sciErr.iErr)
                {
                        printError(&sciErr, 0);
                        return 0;
                }
                intErr2 = getScalarDouble(pvApiCtx, piAddr2, &num);
                if(intErr2)
                {
                        Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 2);
			return -1;
                }

		Mat img1, img2, img3, img4, img5, img6;
		
		vector<Mat> images;
		vector<float> times;
		Mat response;
		Mat hdr, hdr8bit;
		Mat ldr, ldr8bit;
		Mat fusion;

		try
		{
			// OpenCV functionalities

			if(num == 3)
			{
				retrieveImage(img1, 3);
				retrieveImage(img2, 4);
				retrieveImage(img3, 5);

				img1.convertTo(img1, CV_8U);
				img2.convertTo(img2, CV_8U);
				img3.convertTo(img3, CV_8U);

				images.push_back(img1);
				images.push_back(img2);
				images.push_back(img3);

				// to get exposure #1
				sciErr = getVarAddressFromPosition(pvApiCtx, 6, &piAddrExp1);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErrExp1 = getScalarDouble(pvApiCtx, piAddrExp1, &ex1);
				if(intErrExp1)
				{
					Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 6);
					return -1;
				}

				// to get exposure #2
				sciErr = getVarAddressFromPosition(pvApiCtx, 7, &piAddrExp2);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErrExp2 = getScalarDouble(pvApiCtx, piAddrExp2, &ex2);
				if(intErrExp2)
				{
					Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 7);
					return -1;
				}		

				// to get exposure #3
				sciErr = getVarAddressFromPosition(pvApiCtx, 8, &piAddrExp3);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErrExp3 = getScalarDouble(pvApiCtx, piAddrExp3, &ex3);
				if(intErrExp3)
				{
					Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 8);
					return -1;
				}

				times.push_back((float)ex1);
			        times.push_back((float)ex2);
			        times.push_back((float)ex3);

				if(typeOfMethod == 1)
				{
					// Merge Robertson object

					// to get argument #9
					sciErr = getVarAddressFromPosition(pvApiCtx, 9, &piAddr3);
					if(sciErr.iErr)
					{
						printError(&sciErr, 0);
						return 0;
					}
					intErr3 = getScalarDouble(pvApiCtx, piAddr3, &var1);
					if(intErr3)
					{
						Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 9);
						return -1;
					}

					// to get argument #10
					sciErr = getVarAddressFromPosition(pvApiCtx, 10, &piAddr4);
					if(sciErr.iErr)
					{
						printError(&sciErr, 0);
						return 0;
					}
					intErr4 = getScalarDouble(pvApiCtx, piAddr4, &var2);
					if(intErr3)
					{
						Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 10);
						return -1;
					}

					Ptr<CalibrateRobertson> calibrate = createCalibrateRobertson(var1, var2);
					calibrate->process(images, response, times);

					Ptr<MergeRobertson> model = createMergeRobertson();
					model->process(images, hdr, times, response);

					Ptr<Tonemap> tonemap = createTonemap();
					tonemap->process(hdr, ldr);

					hdr = hdr * 255;
				        hdr.convertTo(hdr8bit, CV_8U);

					ldr = ldr*255;
				        ldr.convertTo(ldr8bit, CV_8U);

					// to return the output HDR image
					string tempstring1 = type2str(hdr8bit.type());
					char *checker1;
					checker1 = (char *)malloc(tempstring1.size() + 1);
					memcpy(checker1, tempstring1.c_str(), tempstring1.size() + 1);
					returnImage(checker1, hdr8bit, 1);
					free(checker1);
					AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;

					// to return the output LDR image
					string tempstring2 = type2str(ldr8bit.type());
					char *checker2;
					checker2 = (char *)malloc(tempstring2.size() + 1);
					memcpy(checker2, tempstring2.c_str(), tempstring2.size() + 1);
					returnImage(checker2, ldr8bit, 2);
					free(checker2);
					AssignOutputVariable(pvApiCtx, 2) = nbInputArgument(pvApiCtx) + 2;

				}
				else if(typeOfMethod == 2)
				{
					// Merge Devevec object

					// to get argument #9
					sciErr = getVarAddressFromPosition(pvApiCtx, 9, &piAddr3);
					if(sciErr.iErr)
					{
						printError(&sciErr, 0);
						return 0;
					}
					intErr3 = getScalarDouble(pvApiCtx, piAddr3, &var1);
					if(intErr3)
					{
						Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 9);
						return -1;
					}

					// to get argument #10
					sciErr = getVarAddressFromPosition(pvApiCtx, 10, &piAddr4);
					if(sciErr.iErr)
					{
						printError(&sciErr, 0);
						return 0;
					}
					intErr4 = getScalarDouble(pvApiCtx, piAddr4, &var2);
					if(intErr3)
					{
						Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 10);
						return -1;
					}

					// to get argument #11
					sciErr = getVarAddressFromPosition(pvApiCtx, 11, &piAddr6);
					if(sciErr.iErr)
					{
						printError(&sciErr, 0);
						return 0;
					}
					intErr6 = getScalarBoolean(pvApiCtx, piAddr6, &var4);
					if(intErr6)
					{
						Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 11);
						return -1;
					}


					Ptr<CalibrateDebevec> calibrate	= createCalibrateDebevec(var1, var2, var4);
					calibrate->process(images, response, times);

					Ptr<MergeRobertson> model = createMergeRobertson();
					model->process(images, hdr, times, response);

					Ptr<Tonemap> tonemap = createTonemap();
					tonemap->process(hdr, ldr);

					hdr = hdr * 255;
				        hdr.convertTo(hdr8bit, CV_8U);

					ldr = ldr*255;
				        ldr.convertTo(ldr8bit, CV_8U);

					// to return the output HDR image
					string tempstring1 = type2str(hdr8bit.type());
					char *checker1;
					checker1 = (char *)malloc(tempstring1.size() + 1);
					memcpy(checker1, tempstring1.c_str(), tempstring1.size() + 1);
					returnImage(checker1, hdr8bit, 1);
					free(checker1);
					AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;

					// to return the output LDR image
					string tempstring2 = type2str(ldr8bit.type());
					char *checker2;
					checker2 = (char *)malloc(tempstring2.size() + 1);
					memcpy(checker2, tempstring2.c_str(), tempstring2.size() + 1);
					returnImage(checker2, ldr8bit, 2);
					free(checker2);
					AssignOutputVariable(pvApiCtx, 2) = nbInputArgument(pvApiCtx) + 2;

				}
				else if(typeOfMethod == 3)
				{
					// Merge Mertens object
					
					// to get argument #9
					sciErr = getVarAddressFromPosition(pvApiCtx, 9, &piAddr3);
					if(sciErr.iErr)
					{
						printError(&sciErr, 0);
						return 0;
					}
					intErr3 = getScalarDouble(pvApiCtx, piAddr3, &var1);
					if(intErr3)
					{
						Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 9);
						return -1;
					}

					// to get argument #10
					sciErr = getVarAddressFromPosition(pvApiCtx, 10, &piAddr4);
					if(sciErr.iErr)
					{
						printError(&sciErr, 0);
						return 0;
					}
					intErr4 = getScalarDouble(pvApiCtx, piAddr4, &var2);
					if(intErr3)
					{
						Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 10);
						return -1;
					}

					// to get argument #11
					sciErr = getVarAddressFromPosition(pvApiCtx, 11, &piAddr5);
					if(sciErr.iErr)
					{
						printError(&sciErr, 0);
						return 0;
					}
					intErr5 = getScalarDouble(pvApiCtx, piAddr5, &var3);
					if(intErr5)
					{
						Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 11);
						return -1;
					}


					Ptr<MergeMertens> model = createMergeMertens(var1, var2, var3);
					model->process(images, ldr);

					ldr = ldr*255;
				        ldr.convertTo(ldr8bit, CV_8U);

					// to return the output HDR image
					string tempstring1 = type2str(ldr8bit.type());
					char *checker1;
					checker1 = (char *)malloc(tempstring1.size() + 1);
					memcpy(checker1, tempstring1.c_str(), tempstring1.size() + 1);
					returnImage(checker1, ldr8bit, 1);
					free(checker1);
					AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;

					// to return the output LDR image
					string tempstring2 = type2str(ldr8bit.type());
					char *checker2;
					checker2 = (char *)malloc(tempstring2.size() + 1);
					memcpy(checker2, tempstring2.c_str(), tempstring2.size() + 1);
					returnImage(checker2, ldr8bit, 2);
					free(checker2);
					AssignOutputVariable(pvApiCtx, 2) = nbInputArgument(pvApiCtx) + 2;

				}
				else
				{
					Scierror(999, "Invalid input for typeOfMethod. \n", 1);
				}
			}
			else if(num == 4)
			{
				retrieveImage(img1, 3);
				retrieveImage(img2, 4);
				retrieveImage(img3, 5);
				retrieveImage(img4, 6);

				img1.convertTo(img1, CV_8U);
				img2.convertTo(img2, CV_8U);
				img3.convertTo(img3, CV_8U);
				img4.convertTo(img4, CV_8U);

				images.push_back(img1);
				images.push_back(img2);
				images.push_back(img3);
				images.push_back(img4);

				// to get exposure #1
				sciErr = getVarAddressFromPosition(pvApiCtx, 7, &piAddrExp1);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErrExp1 = getScalarDouble(pvApiCtx, piAddrExp1, &ex1);
				if(intErrExp1)
				{
					Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 7);
					return -1;
				}

				// to get exposure #2
				sciErr = getVarAddressFromPosition(pvApiCtx, 8, &piAddrExp2);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErrExp2 = getScalarDouble(pvApiCtx, piAddrExp2, &ex2);
				if(intErrExp2)
				{
					Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 8);
					return -1;
				}		

				// to get exposure #3
				sciErr = getVarAddressFromPosition(pvApiCtx, 9, &piAddrExp3);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErrExp3 = getScalarDouble(pvApiCtx, piAddrExp3, &ex3);
				if(intErrExp3)
				{
					Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 9);
					return -1;
				}

				// to get exposure #4
				sciErr = getVarAddressFromPosition(pvApiCtx, 10, &piAddrExp4);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErrExp4 = getScalarDouble(pvApiCtx, piAddrExp4, &ex4);
				if(intErrExp4)
				{
					Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 10);
					return -1;
				}


				times.push_back((float)ex1);
			        times.push_back((float)ex2);
			        times.push_back((float)ex3);
				times.push_back((float)ex4);

				if(typeOfMethod == 1)
				{
					// Merge Robertson object

					// to get argument #11
					sciErr = getVarAddressFromPosition(pvApiCtx, 11, &piAddr3);
					if(sciErr.iErr)
					{
						printError(&sciErr, 0);
						return 0;
					}
					intErr3 = getScalarDouble(pvApiCtx, piAddr3, &var1);
					if(intErr3)
					{
						Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 11);
						return -1;
					}

					// to get argument #12
					sciErr = getVarAddressFromPosition(pvApiCtx, 12, &piAddr4);
					if(sciErr.iErr)
					{
						printError(&sciErr, 0);
						return 0;
					}
					intErr4 = getScalarDouble(pvApiCtx, piAddr4, &var2);
					if(intErr3)
					{
						Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 12);
						return -1;
					}

					Ptr<CalibrateRobertson> calibrate = createCalibrateRobertson(var1, var2);
					calibrate->process(images, response, times);

					Ptr<MergeRobertson> model = createMergeRobertson();
					model->process(images, hdr, times, response);

					Ptr<Tonemap> tonemap = createTonemap();
					tonemap->process(hdr, ldr);

					hdr = hdr * 255;
				        hdr.convertTo(hdr8bit, CV_8U);

					ldr = ldr*255;
				        ldr.convertTo(ldr8bit, CV_8U);

					// to return the output HDR image
					string tempstring1 = type2str(hdr8bit.type());
					char *checker1;
					checker1 = (char *)malloc(tempstring1.size() + 1);
					memcpy(checker1, tempstring1.c_str(), tempstring1.size() + 1);
					returnImage(checker1, hdr8bit, 1);
					free(checker1);
					AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;

					// to return the output LDR image
					string tempstring2 = type2str(ldr8bit.type());
					char *checker2;
					checker2 = (char *)malloc(tempstring2.size() + 1);
					memcpy(checker2, tempstring2.c_str(), tempstring2.size() + 1);
					returnImage(checker2, ldr8bit, 2);
					free(checker2);
					AssignOutputVariable(pvApiCtx, 2) = nbInputArgument(pvApiCtx) + 2;

				}
				else if(typeOfMethod == 2)
				{
					// Merge Devevec object

					// to get argument #11
					sciErr = getVarAddressFromPosition(pvApiCtx, 11, &piAddr3);
					if(sciErr.iErr)
					{
						printError(&sciErr, 0);
						return 0;
					}
					intErr3 = getScalarDouble(pvApiCtx, piAddr3, &var1);
					if(intErr3)
					{
						Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 11);
						return -1;
					}

					// to get argument #12
					sciErr = getVarAddressFromPosition(pvApiCtx, 12, &piAddr4);
					if(sciErr.iErr)
					{
						printError(&sciErr, 0);
						return 0;
					}
					intErr4 = getScalarDouble(pvApiCtx, piAddr4, &var2);
					if(intErr3)
					{
						Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 12);
						return -1;
					}

					// to get argument #13
					sciErr = getVarAddressFromPosition(pvApiCtx, 13, &piAddr6);
					if(sciErr.iErr)
					{
						printError(&sciErr, 0);
						return 0;
					}
					intErr6 = getScalarBoolean(pvApiCtx, piAddr6, &var4);
					if(intErr6)
					{
						Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 10);
						return -1;
					}


					Ptr<CalibrateDebevec> calibrate	= createCalibrateDebevec(var1, var2, var4);
					calibrate->process(images, response, times);

					Ptr<MergeRobertson> model = createMergeRobertson();
					model->process(images, hdr, times, response);

					Ptr<Tonemap> tonemap = createTonemap();
					tonemap->process(hdr, ldr);

					hdr = hdr * 255;
				        hdr.convertTo(hdr8bit, CV_8U);

					ldr = ldr*255;
				        ldr.convertTo(ldr8bit, CV_8U);

					// to return the output HDR image
					string tempstring1 = type2str(hdr8bit.type());
					char *checker1;
					checker1 = (char *)malloc(tempstring1.size() + 1);
					memcpy(checker1, tempstring1.c_str(), tempstring1.size() + 1);
					returnImage(checker1, hdr8bit, 1);
					free(checker1);
					AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;

					// to return the output LDR image
					string tempstring2 = type2str(ldr8bit.type());
					char *checker2;
					checker2 = (char *)malloc(tempstring2.size() + 1);
					memcpy(checker2, tempstring2.c_str(), tempstring2.size() + 1);
					returnImage(checker2, ldr8bit, 2);
					free(checker2);
					AssignOutputVariable(pvApiCtx, 2) = nbInputArgument(pvApiCtx) + 2;


				}
				else if(typeOfMethod == 3)
				{
					// Merge Mertens object
					
					// to get argument #11
					sciErr = getVarAddressFromPosition(pvApiCtx, 11, &piAddr3);
					if(sciErr.iErr)
					{
						printError(&sciErr, 0);
						return 0;
					}
					intErr3 = getScalarDouble(pvApiCtx, piAddr3, &var1);
					if(intErr3)
					{
						Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 11);
						return -1;
					}

					// to get argument #12
					sciErr = getVarAddressFromPosition(pvApiCtx, 12, &piAddr4);
					if(sciErr.iErr)
					{
						printError(&sciErr, 0);
						return 0;
					}
					intErr4 = getScalarDouble(pvApiCtx, piAddr4, &var2);
					if(intErr3)
					{
						Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 12);
						return -1;
					}

					// to get argument #13
					sciErr = getVarAddressFromPosition(pvApiCtx, 13, &piAddr5);
					if(sciErr.iErr)
					{
						printError(&sciErr, 0);
						return 0;
					}
					intErr5 = getScalarDouble(pvApiCtx, piAddr5, &var3);
					if(intErr5)
					{
						Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 13);
						return -1;
					}


					Ptr<MergeMertens> model = createMergeMertens(var1, var2, var3);
					model->process(images, ldr);

					ldr = ldr*255;
				        ldr.convertTo(ldr8bit, CV_8U);

					// to return the output HDR image
					string tempstring1 = type2str(ldr8bit.type());
					char *checker1;
					checker1 = (char *)malloc(tempstring1.size() + 1);
					memcpy(checker1, tempstring1.c_str(), tempstring1.size() + 1);
					returnImage(checker1, ldr8bit, 1);
					free(checker1);
					AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;

					// to return the output LDR image
					string tempstring2 = type2str(ldr8bit.type());
					char *checker2;
					checker2 = (char *)malloc(tempstring2.size() + 1);
					memcpy(checker2, tempstring2.c_str(), tempstring2.size() + 1);
					returnImage(checker2, ldr8bit, 2);
					free(checker2);
					AssignOutputVariable(pvApiCtx, 2) = nbInputArgument(pvApiCtx) + 2;

				}
				else
				{
					Scierror(999, "Invalid input for typeOfMethod. \n", 1);
				}
			}
			else if(num == 5)
			{
				retrieveImage(img1, 3);
				retrieveImage(img2, 4);
				retrieveImage(img3, 5);
				retrieveImage(img4, 6);
				retrieveImage(img5, 7);

				img1.convertTo(img1, CV_8U);
				img2.convertTo(img2, CV_8U);
				img3.convertTo(img3, CV_8U);
				img4.convertTo(img4, CV_8U);
				img5.convertTo(img5, CV_8U);

				images.push_back(img1);
				images.push_back(img2);
				images.push_back(img3);
				images.push_back(img4);
				images.push_back(img5);

				// to get exposure #1
				sciErr = getVarAddressFromPosition(pvApiCtx, 8, &piAddrExp1);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErrExp1 = getScalarDouble(pvApiCtx, piAddrExp1, &ex1);
				if(intErrExp1)
				{
					Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 8);
					return -1;
				}

				// to get exposure #2
				sciErr = getVarAddressFromPosition(pvApiCtx, 9, &piAddrExp2);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErrExp2 = getScalarDouble(pvApiCtx, piAddrExp2, &ex2);
				if(intErrExp2)
				{
					Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 9);
					return -1;
				}		

				// to get exposure #3
				sciErr = getVarAddressFromPosition(pvApiCtx, 10, &piAddrExp3);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErrExp3 = getScalarDouble(pvApiCtx, piAddrExp3, &ex3);
				if(intErrExp3)
				{
					Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 10);
					return -1;
				}

				// to get exposure #4
				sciErr = getVarAddressFromPosition(pvApiCtx, 11, &piAddrExp4);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErrExp4 = getScalarDouble(pvApiCtx, piAddrExp4, &ex4);
				if(intErrExp4)
				{
					Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 11);
					return -1;
				}

				// to get exposure #5
				sciErr = getVarAddressFromPosition(pvApiCtx, 12, &piAddrExp5);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErrExp5 = getScalarDouble(pvApiCtx, piAddrExp5, &ex5);
				if(intErrExp5)
				{
					Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 12);
					return -1;
				}


				times.push_back((float)ex1);
			        times.push_back((float)ex2);
			        times.push_back((float)ex3);
				times.push_back((float)ex4);
				times.push_back((float)ex5);

				if(typeOfMethod == 1)
				{
					// Merge Robertson object

					// to get argument #9
					sciErr = getVarAddressFromPosition(pvApiCtx, 13, &piAddr3);
					if(sciErr.iErr)
					{
						printError(&sciErr, 0);
						return 0;
					}
					intErr3 = getScalarDouble(pvApiCtx, piAddr3, &var1);
					if(intErr3)
					{
						Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 13);
						return -1;
					}

					// to get argument #10
					sciErr = getVarAddressFromPosition(pvApiCtx, 14, &piAddr4);
					if(sciErr.iErr)
					{
						printError(&sciErr, 0);
						return 0;
					}
					intErr4 = getScalarDouble(pvApiCtx, piAddr4, &var2);
					if(intErr3)
					{
						Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 14);
						return -1;
					}

					Ptr<CalibrateRobertson> calibrate = createCalibrateRobertson(var1, var2);
					calibrate->process(images, response, times);

					Ptr<MergeRobertson> model = createMergeRobertson();
					model->process(images, hdr, times, response);

					Ptr<Tonemap> tonemap = createTonemap();
					tonemap->process(hdr, ldr);

					hdr = hdr * 255;
				        hdr.convertTo(hdr8bit, CV_8U);

					ldr = ldr*255;
				        ldr.convertTo(ldr8bit, CV_8U);

					// to return the output HDR image
					string tempstring1 = type2str(hdr8bit.type());
					char *checker1;
					checker1 = (char *)malloc(tempstring1.size() + 1);
					memcpy(checker1, tempstring1.c_str(), tempstring1.size() + 1);
					returnImage(checker1, hdr8bit, 1);
					free(checker1);
					AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;

					// to return the output LDR image
					string tempstring2 = type2str(ldr8bit.type());
					char *checker2;
					checker2 = (char *)malloc(tempstring2.size() + 1);
					memcpy(checker2, tempstring2.c_str(), tempstring2.size() + 1);
					returnImage(checker2, ldr8bit, 2);
					free(checker2);
					AssignOutputVariable(pvApiCtx, 2) = nbInputArgument(pvApiCtx) + 2;


				}
				else if(typeOfMethod == 2)
				{
					// Merge Devevec object

					// to get argument #9
					sciErr = getVarAddressFromPosition(pvApiCtx, 13, &piAddr3);
					if(sciErr.iErr)
					{
						printError(&sciErr, 0);
						return 0;
					}
					intErr3 = getScalarDouble(pvApiCtx, piAddr3, &var1);
					if(intErr3)
					{
						Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 13);
						return -1;
					}

					// to get argument #10
					sciErr = getVarAddressFromPosition(pvApiCtx, 14, &piAddr4);
					if(sciErr.iErr)
					{
						printError(&sciErr, 0);
						return 0;
					}
					intErr4 = getScalarDouble(pvApiCtx, piAddr4, &var2);
					if(intErr3)
					{
						Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 14);
						return -1;
					}

					// to get argument #11
					sciErr = getVarAddressFromPosition(pvApiCtx, 15, &piAddr6);
					if(sciErr.iErr)
					{
						printError(&sciErr, 0);
						return 0;
					}
					intErr6 = getScalarBoolean(pvApiCtx, piAddr6, &var4);
					if(intErr6)
					{
						Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 15);
						return -1;
					}


					Ptr<CalibrateDebevec> calibrate	= createCalibrateDebevec(var1, var2, var4);
					calibrate->process(images, response, times);

					Ptr<MergeRobertson> model = createMergeRobertson();
					model->process(images, hdr, times, response);

					Ptr<Tonemap> tonemap = createTonemap();
					tonemap->process(hdr, ldr);

					hdr = hdr * 255;
				        hdr.convertTo(hdr8bit, CV_8U);

					ldr = ldr*255;
				        ldr.convertTo(ldr8bit, CV_8U);

					// to return the output HDR image
					string tempstring1 = type2str(hdr8bit.type());
					char *checker1;
					checker1 = (char *)malloc(tempstring1.size() + 1);
					memcpy(checker1, tempstring1.c_str(), tempstring1.size() + 1);
					returnImage(checker1, hdr8bit, 1);
					free(checker1);
					AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;

					// to return the output LDR image
					string tempstring2 = type2str(ldr8bit.type());
					char *checker2;
					checker2 = (char *)malloc(tempstring2.size() + 1);
					memcpy(checker2, tempstring2.c_str(), tempstring2.size() + 1);
					returnImage(checker2, ldr8bit, 2);
					free(checker2);
					AssignOutputVariable(pvApiCtx, 2) = nbInputArgument(pvApiCtx) + 2;

				}
				else if(typeOfMethod == 3)
				{
					// Merge Mertens object
					
					// to get argument #9
					sciErr = getVarAddressFromPosition(pvApiCtx, 13, &piAddr3);
					if(sciErr.iErr)
					{
						printError(&sciErr, 0);
						return 0;
					}
					intErr3 = getScalarDouble(pvApiCtx, piAddr3, &var1);
					if(intErr3)
					{
						Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 13);
						return -1;
					}

					// to get argument #10
					sciErr = getVarAddressFromPosition(pvApiCtx, 14, &piAddr4);
					if(sciErr.iErr)
					{
						printError(&sciErr, 0);
						return 0;
					}
					intErr4 = getScalarDouble(pvApiCtx, piAddr4, &var2);
					if(intErr3)
					{
						Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 14);
						return -1;
					}

					// to get argument #11
					sciErr = getVarAddressFromPosition(pvApiCtx, 15, &piAddr5);
					if(sciErr.iErr)
					{
						printError(&sciErr, 0);
						return 0;
					}
					intErr5 = getScalarDouble(pvApiCtx, piAddr5, &var3);
					if(intErr5)
					{
						Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 15);
						return -1;
					}


					Ptr<MergeMertens> model = createMergeMertens(var1, var2, var3);
					model->process(images, ldr);

					ldr = ldr*255;
				        ldr.convertTo(ldr8bit, CV_8U);

					// to return the output HDR image
					string tempstring1 = type2str(ldr8bit.type());
					char *checker1;
					checker1 = (char *)malloc(tempstring1.size() + 1);
					memcpy(checker1, tempstring1.c_str(), tempstring1.size() + 1);
					returnImage(checker1, ldr8bit, 1);
					free(checker1);
					AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;

					// to return the output LDR image
					string tempstring2 = type2str(ldr8bit.type());
					char *checker2;
					checker2 = (char *)malloc(tempstring2.size() + 1);
					memcpy(checker2, tempstring2.c_str(), tempstring2.size() + 1);
					returnImage(checker2, ldr8bit, 2);
					free(checker2);
					AssignOutputVariable(pvApiCtx, 2) = nbInputArgument(pvApiCtx) + 2;

				}
				else
				{
					Scierror(999, "Invalid input for typeOfMethod. \n", 1);
				}			
			}
			else if(num == 6)
			{
				retrieveImage(img1, 3);
				retrieveImage(img2, 4);
				retrieveImage(img3, 5);
				retrieveImage(img4, 6);
				retrieveImage(img5, 7);
				retrieveImage(img6, 8);

				img1.convertTo(img1, CV_8U);
				img2.convertTo(img2, CV_8U);
				img3.convertTo(img3, CV_8U);
				img4.convertTo(img4, CV_8U);
				img5.convertTo(img5, CV_8U);
				img6.convertTo(img6, CV_8U);

				images.push_back(img1);
				images.push_back(img2);
				images.push_back(img3);
				images.push_back(img4);
				images.push_back(img5);
				images.push_back(img6);

				// to get exposure #1
				sciErr = getVarAddressFromPosition(pvApiCtx, 9, &piAddrExp1);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErrExp1 = getScalarDouble(pvApiCtx, piAddrExp1, &ex1);
				if(intErrExp1)
				{
					Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 9);
					return -1;
				}

				// to get exposure #2
				sciErr = getVarAddressFromPosition(pvApiCtx, 10, &piAddrExp2);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErrExp2 = getScalarDouble(pvApiCtx, piAddrExp2, &ex2);
				if(intErrExp2)
				{
					Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 10);
					return -1;
				}		

				// to get exposure #3
				sciErr = getVarAddressFromPosition(pvApiCtx, 11, &piAddrExp3);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErrExp3 = getScalarDouble(pvApiCtx, piAddrExp3, &ex3);
				if(intErrExp3)
				{
					Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 11);
					return -1;
				}

				// to get exposure #4
				sciErr = getVarAddressFromPosition(pvApiCtx, 12, &piAddrExp4);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErrExp4 = getScalarDouble(pvApiCtx, piAddrExp4, &ex4);
				if(intErrExp4)
				{
					Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 12);
					return -1;
				}

				// to get exposure #5
				sciErr = getVarAddressFromPosition(pvApiCtx, 13, &piAddrExp5);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErrExp5 = getScalarDouble(pvApiCtx, piAddrExp5, &ex5);
				if(intErrExp5)
				{
					Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 13);
					return -1;
				}
			
				// to get exposure #6
				sciErr = getVarAddressFromPosition(pvApiCtx, 14, &piAddrExp6);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				intErrExp6 = getScalarDouble(pvApiCtx, piAddrExp6, &ex6);
				if(intErrExp6)
				{
					Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 14);
					return -1;
				}



				times.push_back((float)ex1);
			        times.push_back((float)ex2);
			        times.push_back((float)ex3);
				times.push_back((float)ex4);
				times.push_back((float)ex5);
				times.push_back((float)ex6);

				if(typeOfMethod == 1)
				{
					// Merge Robertson object

					// to get argument #9
					sciErr = getVarAddressFromPosition(pvApiCtx, 15, &piAddr3);
					if(sciErr.iErr)
					{
						printError(&sciErr, 0);
						return 0;
					}
					intErr3 = getScalarDouble(pvApiCtx, piAddr3, &var1);
					if(intErr3)
					{
						Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 15);
						return -1;
					}

					// to get argument #10
					sciErr = getVarAddressFromPosition(pvApiCtx, 16, &piAddr4);
					if(sciErr.iErr)
					{
						printError(&sciErr, 0);
						return 0;
					}
					intErr4 = getScalarDouble(pvApiCtx, piAddr4, &var2);
					if(intErr3)
					{
						Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 16);
						return -1;
					}

					Ptr<CalibrateRobertson> calibrate = createCalibrateRobertson(var1, var2);
					calibrate->process(images, response, times);

					Ptr<MergeRobertson> model = createMergeRobertson();
					model->process(images, hdr, times, response);

					Ptr<Tonemap> tonemap = createTonemap();
					tonemap->process(hdr, ldr);

					hdr = hdr * 255;
				        hdr.convertTo(hdr8bit, CV_8U);

					ldr = ldr*255;
				        ldr.convertTo(ldr8bit, CV_8U);

					// to return the output HDR image
					string tempstring1 = type2str(hdr8bit.type());
					char *checker1;
					checker1 = (char *)malloc(tempstring1.size() + 1);
					memcpy(checker1, tempstring1.c_str(), tempstring1.size() + 1);
					returnImage(checker1, hdr8bit, 1);
					free(checker1);
					AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;

					// to return the output LDR image
					string tempstring2 = type2str(ldr8bit.type());
					char *checker2;
					checker2 = (char *)malloc(tempstring2.size() + 1);
					memcpy(checker2, tempstring2.c_str(), tempstring2.size() + 1);
					returnImage(checker2, ldr8bit, 2);
					free(checker2);
					AssignOutputVariable(pvApiCtx, 2) = nbInputArgument(pvApiCtx) + 2;



				}
				else if(typeOfMethod == 2)
				{
					// Merge Devevec object

					// to get argument #9
					sciErr = getVarAddressFromPosition(pvApiCtx, 15, &piAddr3);
					if(sciErr.iErr)
					{
						printError(&sciErr, 0);
						return 0;
					}
					intErr3 = getScalarDouble(pvApiCtx, piAddr3, &var1);
					if(intErr3)
					{
						Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 15);
						return -1;
					}

					// to get argument #10
					sciErr = getVarAddressFromPosition(pvApiCtx, 16, &piAddr4);
					if(sciErr.iErr)
					{
						printError(&sciErr, 0);
						return 0;
					}
					intErr4 = getScalarDouble(pvApiCtx, piAddr4, &var2);
					if(intErr3)
					{
						Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 16);
						return -1;
					}

					// to get argument #11
					sciErr = getVarAddressFromPosition(pvApiCtx, 17, &piAddr6);
					if(sciErr.iErr)
					{
						printError(&sciErr, 0);
						return 0;
					}
					intErr6 = getScalarBoolean(pvApiCtx, piAddr6, &var4);
					if(intErr6)
					{
						Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 17);
						return -1;
					}


					Ptr<CalibrateDebevec> calibrate	= createCalibrateDebevec(var1, var2, var4);
					calibrate->process(images, response, times);

					Ptr<MergeRobertson> model = createMergeRobertson();
					model->process(images, hdr, times, response);

					Ptr<Tonemap> tonemap = createTonemap();
					tonemap->process(hdr, ldr);

					hdr = hdr * 255;
				        hdr.convertTo(hdr8bit, CV_8U);

					ldr = ldr*255;
				        ldr.convertTo(ldr8bit, CV_8U);

					// to return the output HDR image
					string tempstring1 = type2str(hdr8bit.type());
					char *checker1;
					checker1 = (char *)malloc(tempstring1.size() + 1);
					memcpy(checker1, tempstring1.c_str(), tempstring1.size() + 1);
					returnImage(checker1, hdr8bit, 1);
					free(checker1);
					AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;

					// to return the output LDR image
					string tempstring2 = type2str(ldr8bit.type());
					char *checker2;
					checker2 = (char *)malloc(tempstring2.size() + 1);
					memcpy(checker2, tempstring2.c_str(), tempstring2.size() + 1);
					returnImage(checker2, ldr8bit, 2);
					free(checker2);
					AssignOutputVariable(pvApiCtx, 2) = nbInputArgument(pvApiCtx) + 2;
				
				}
				else if(typeOfMethod == 3)
				{
					// Merge Mertens object
					
					// to get argument #9
					sciErr = getVarAddressFromPosition(pvApiCtx, 15, &piAddr3);
					if(sciErr.iErr)
					{
						printError(&sciErr, 0);
						return 0;
					}
					intErr3 = getScalarDouble(pvApiCtx, piAddr3, &var1);
					if(intErr3)
					{
						return intErr3;
					}

					// to get argument #10
					sciErr = getVarAddressFromPosition(pvApiCtx, 16, &piAddr4);
					if(sciErr.iErr)
					{
						printError(&sciErr, 0);
						return 0;
					}
					intErr4 = getScalarDouble(pvApiCtx, piAddr4, &var2);
					if(intErr3)
					{
						return intErr3;
					}

					// to get argument #11
					sciErr = getVarAddressFromPosition(pvApiCtx, 17, &piAddr5);
					if(sciErr.iErr)
					{
						printError(&sciErr, 0);
						return 0;
					}
					intErr5 = getScalarDouble(pvApiCtx, piAddr5, &var3);
					if(intErr5)
					{
						return intErr5;
					}


					Ptr<MergeMertens> model = createMergeMertens(var1, var2, var3);
					model->process(images, ldr);

					ldr = ldr*255;
				        ldr.convertTo(ldr8bit, CV_8U);

					// to return the output HDR image
					string tempstring1 = type2str(ldr8bit.type());
					char *checker1;
					checker1 = (char *)malloc(tempstring1.size() + 1);
					memcpy(checker1, tempstring1.c_str(), tempstring1.size() + 1);
					returnImage(checker1, ldr8bit, 1);
					free(checker1);
					AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;

					// to return the output LDR image
					string tempstring2 = type2str(ldr8bit.type());
					char *checker2;
					checker2 = (char *)malloc(tempstring2.size() + 1);
					memcpy(checker2, tempstring2.c_str(), tempstring2.size() + 1);
					returnImage(checker2, ldr8bit, 2);
					free(checker2);
					AssignOutputVariable(pvApiCtx, 2) = nbInputArgument(pvApiCtx) + 2;

					
				}
				else
				{
					Scierror(999, "Invalid input for typeOfMethod. \n", 1);
				}			
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
	
		
			
