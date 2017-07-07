/***************************************************
Author : Shreyash Sharma

TO DO:
1) nearest neighbor interpolation
2) upright parameter implementation
 ***************************************************/

#include <numeric>
#include <string.h>
#include <math.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/objdetect/objdetect.hpp>
#include <bits/stdc++.h>
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
//#include "../common.cpp"
        
	inline void ELBP(const Mat& src, Mat& dst, int radius, int neighbors) 
	{
		neighbors = max(min(neighbors,31),1); // set bounds...
		// Note: alternatively you can switch to the new OpenCV Mat_
		// type system to define an unsigned int matrix... I am probably
		// mistaken here, but I didn't see an unsigned int representation
		// in OpenCV's classic typesystem...
		dst = Mat::zeros(src.rows-2*radius, src.cols-2*radius, CV_8UC1);
		for(int n=0; n<neighbors; n++) 
		{
			// sample points
			float x = static_cast<float>(radius) * cos(2.0*M_PI*n/static_cast<float>(neighbors));
			float y = static_cast<float>(radius) * -sin(2.0*M_PI*n/static_cast<float>(neighbors));
			// relative indices
			int fx = static_cast<int>(floor(x));
			int fy = static_cast<int>(floor(y));
			int cx = static_cast<int>(ceil(x));
			int cy = static_cast<int>(ceil(y));
			// fractional part
			float ty = y - fy;
			float tx = x - fx;
			// set interpolation weights
			float w1 = (1 - tx) * (1 - ty);
			float w2 =      tx  * (1 - ty);
			float w3 = (1 - tx) *      ty;
			float w4 =      tx  *      ty;
			// iterate through your data
			for(int i=radius; i < src.rows-radius;i++) 
			{
				for(int j=radius;j < src.cols-radius;j++) 
				{
					float t = w1*src.at<float>(i+fy,j+fx) + w2*src.at<float>(i+fy,j+cx) + w3*src.at<float>(i+cy,j+fx) + w4*src.at<float>(i+cy,j+cx);
					// we are dealing with floating point precision, so add some little tolerance
					dst.at<unsigned int>(i-radius,j-radius) += ((t > src.at<float>(i,j)) && (abs(t-src.at<float>(i,j)) > std::numeric_limits<float>::epsilon())) << n;
				}
			}
		}
	}

	/*void ELBP(const Mat& src, Mat& dst, int radius, int neighbors)
	{
		switch(src.type()) {
			case CV_8SC1: ELBP_<char>(src, dst, radius, neighbors); break;
			case CV_8UC1: ELBP_<unsigned char>(src, dst, radius, neighbors); break;
			case CV_16SC1: ELBP_<short>(src, dst, radius, neighbors); break;
			case CV_16UC1: ELBP_<unsigned short>(src, dst, radius, neighbors); break;
			case CV_32SC1: ELBP_<int>(src, dst, radius, neighbors); break;
			case CV_32FC1: ELBP_<float>(src, dst, radius, neighbors); break;
			case CV_64FC1: ELBP_<double>(src, dst, radius, neighbors); break;
		}
	}*/

	static Mat histc_(const Mat& src, int minVal=0, int maxVal=255, bool normed=false)
	{
		Mat result;
		// Establish the number of bins.
		int histSize = maxVal-minVal+1;
		// Set the ranges.
		float range[] = { static_cast<float>(minVal), static_cast<float>(maxVal+1) };
		const float* histRange = { range };
		// calc histogram
		calcHist(&src, 1, 0, Mat(), result, 1, &histSize, &histRange, true, false);
		// normalize
		if(normed) {
			result /= (int)src.total();
		}
		return result.reshape(1,1);
	}

	static Mat histc(InputArray _src, int minVal, int maxVal, bool normed)
	{
		Mat src = _src.getMat();
		switch (src.type()) {
			case CV_8SC1:
				return histc_(Mat_<float>(src), minVal, maxVal, normed);
				break;
			case CV_8UC1:
				return histc_(src, minVal, maxVal, normed);
				break;
			case CV_16SC1:
				return histc_(Mat_<float>(src), minVal, maxVal, normed);
				break;
			case CV_16UC1:
				return histc_(src, minVal, maxVal, normed);
				break;
			case CV_32SC1:
				return histc_(Mat_<float>(src), minVal, maxVal, normed);
				break;
			case CV_32FC1:
				return histc_(src, minVal, maxVal, normed);
				break;
		}
		return Mat();
	}


	static Mat spatial_histogram(InputArray _src, int numPatterns, int grid_x, int grid_y, bool normed)
	{        //sciprint("hello ji1");
		 Mat src = _src.getMat();
		// calculate LBP patch size
		//int width = src.cols/grid_x;
		//int height = src.rows/grid_y;
		//sciprint("hello ji2");
		int width = src.cols/grid_x;
		int height = src.rows/grid_y;
                //sciprint("hello ji3");
		// allocate memory for the spatial histogram
		//sciprint("hello ji");
		Mat result = Mat::zeros(grid_x * grid_y, numPatterns, CV_32FC1);
		//sciprint("hua kya?");
		// return matrix with zeros if no data was given
		if(src.empty())
			return result.reshape(1,1);
		// initial result_row
		int resultRowIdx = 0;
		// iterate through grid
		for(int i = 0; i < grid_y; i++) {
		                //sciprint("hello ji11");
			for(int j = 0; j < grid_x; j++) {
				//sciprint("hello ji22");
				if((i+1)*height<src.rows && (j+1)*width<src.cols)
				{
				Mat src_cell = Mat(src, Range(i*height,(i+1)*height), Range(j*width,(j+1)*width));
				Mat cell_hist = histc(src_cell, 0, (numPatterns-1), normed);
				
		
				
				// copy to the result matrix
				
				Mat result_row = result.row(resultRowIdx);
				cell_hist.reshape(1,1).convertTo(result_row, CV_32FC1);
				// increase row count in result matrix
				resultRowIdx++;
			        }
			}
		}
		//sciprint("hello ji33");
		// return result as reshaped feature vector
		return result.reshape(1,1);
	}

	int opencv_extractLBPFeatures(char *fname, unsigned long fname_len)
	{
		//Error management variable
		SciErr sciErr;

		//Variable declaration
		int iRows = 0;
		int iCols = 0;
		int intErr = 0;
		int nbInputArguments = 0;
		int *piLen = NULL;
		int *piAddr = NULL;
		bool *providedArgs = NULL; 
		bool linearInterpolation = 1;
		bool normalization = 1;
		char *currentArg = NULL; 
		char **pstData  = NULL;
		double radius = 1;
		double upright = 1;
		double neighbors = 1;
		double *cellSize = NULL;
		double *outputHist = NULL;
		Mat image;
		Mat grayscaleImage;
		Mat lbpImage;
		Mat spatialHistogram;

		//Check input output arguments
		CheckInputArgument(pvApiCtx, 1, 13);
		CheckOutputArgument(pvApiCtx, 1, 1);

		nbInputArguments = *getNbInputArgument(pvApiCtx);

		if((nbInputArguments-1)%2!=0)
		{
			Scierror(999, "Incorrect number of arguments provided. Please check the documentation for more information.\n"); 
			return 0; 
		}

		providedArgs = (bool*)malloc(sizeof(bool) * 6);
		for(int i=0;i<6;i++)
			providedArgs[i] = 0;

		//getting input arguments
		
		retrieveImage(image, 1);
		//sciprint("%d\n%d\n", image.rows,image.cols);
		 
		
		cellSize = (double*)malloc(sizeof(double) * 2);

		for(int iter=2;iter<=nbInputArguments;iter++)
		{
			// Getting address of next argument
			sciErr = getVarAddressFromPosition(pvApiCtx, iter, &piAddr); 
			if (sciErr.iErr)
			{
				printError(&sciErr, 0); 
				return 0; 
			}

			// Extracting name of next argument takes three calls to getMatrixOfString
			sciErr = getMatrixOfString(pvApiCtx, piAddr, &iRows, &iCols, NULL, NULL); 
			if (sciErr.iErr)
			{
				printError(&sciErr, 0); 
				return 0; 
			}

			piLen = (int*) malloc(sizeof(int) * iRows * iCols); 

			sciErr = getMatrixOfString(pvApiCtx,  piAddr,  &iRows,  &iCols,  piLen,  NULL); 
			if (sciErr.iErr)
			{
				printError(&sciErr, 0); 
				return 0; 
			}

			pstData = (char**) malloc(sizeof(char*) * iRows * iCols); 
			for(int iterPstData = 0; iterPstData < iRows * iCols; iterPstData++)
			{
				pstData[iterPstData] = (char*) malloc(sizeof(char) * piLen[iterPstData] + 1); 
			}

			sciErr = getMatrixOfString(pvApiCtx, piAddr, &iRows, &iCols, piLen, pstData); 
			if (sciErr.iErr)
			{
				printError(&sciErr, 0); 
				return 0; 
			}

			currentArg = pstData[0];
			// providedArgs[] makes sure that no optional argument is provided more than once

			if(strcmp(currentArg, "NumNeighbors")==0)
			{
				if(!providedArgs[0])
				{
					sciErr = getVarAddressFromPosition(pvApiCtx, ++iter, &piAddr); 
					if (sciErr.iErr)
					{
						printError(&sciErr, 0); 
						return 0; 
					}

					intErr = getScalarDouble(pvApiCtx, piAddr, &neighbors); 
					if(intErr)
					{
						return intErr;  
					}
					// Checking if values are in proper range. Same for all optional arguments
					if(neighbors < 1 || neighbors > 24)
					{
						Scierror(999, "Error: Please provide proper value for \"%s\". Permissible range is [1, 24].\n", currentArg); 
						return 0; 
					}	
					providedArgs[0] = 1;

				}
				else if(providedArgs[0])
				{
					Scierror(999, "Please provide optional arguments only once.\n"); 
					return 0; 
				}
			}
			else if(strcmp(currentArg, "Radius")==0)
			{
				if(!providedArgs[1])
				{
					sciErr = getVarAddressFromPosition(pvApiCtx, ++iter, &piAddr); 
					if (sciErr.iErr)
					{
						printError(&sciErr, 0); 
						return 0; 
					}

					intErr = getScalarDouble(pvApiCtx, piAddr, &radius); 
					if(intErr)
					{
						return intErr;  
					}
					// Checking if values are in proper range. Same for all optional arguments
					if(radius < 1 || radius > 5)
					{
						Scierror(999, "Error: Please provide proper value for \"%s\". Permissible range is [1, 5].\n", currentArg); 
						return 0; 
					}
					providedArgs[1] = 1;
				}
				else if(providedArgs[1])
				{
					Scierror(999, "Please provide optional arguments only once.\n"); 
					return 0; 
				}
			}
			else if(strcmp(currentArg, "Upright")==0)
			{
				if(!providedArgs[2])
				{
					sciErr = getVarAddressFromPosition(pvApiCtx, ++iter, &piAddr); 
					if (sciErr.iErr)
					{
						printError(&sciErr, 0); 
						return 0; 
					}

					intErr = getScalarDouble(pvApiCtx, piAddr, &upright); 
					if(intErr)
					{
						return intErr;  
					}
					// Checking if values are in proper range. Same for all optional arguments
					if(upright!=0.0 && upright!=1.0)
					{
						Scierror(999, "Error: Please provide proper value for \"%s\". Permissible values are 0 or 1.\n", currentArg); 
						return 0; 
					}
					providedArgs[2] = 1;
				}
				else if(providedArgs[2])
				{
					Scierror(999, "Please provide optional arguments only once.\n"); 
					return 0; 
				}
			}
			else if(strcmp(currentArg, "Interpolation")==0)
			{
				if(!providedArgs[3])
				{
					sciErr = getVarAddressFromPosition(pvApiCtx, ++iter, &piAddr); 
					if (sciErr.iErr)
					{
						printError(&sciErr, 0); 
						return 0; 
					}

					sciErr = getMatrixOfString(pvApiCtx, piAddr, &iRows, &iCols, NULL, NULL); 
					if (sciErr.iErr)
					{
						printError(&sciErr, 0); 
						return 0; 
					}

					piLen = (int*) malloc(sizeof(int) * iRows * iCols); 

					sciErr = getMatrixOfString(pvApiCtx,  piAddr,  &iRows,  &iCols,  piLen,  NULL); 
					if (sciErr.iErr)
					{
						printError(&sciErr, 0); 
						return 0; 
					}

					pstData = (char**) malloc(sizeof(char*) * iRows * iCols); 
					for(int iterPstData = 0; iterPstData < iRows * iCols; iterPstData++)
					{
						pstData[iterPstData] = (char*) malloc(sizeof(char) * piLen[iterPstData] + 1); 
					}

					sciErr = getMatrixOfString(pvApiCtx, piAddr, &iRows, &iCols, piLen, pstData); 
					if (sciErr.iErr)
					{
						printError(&sciErr, 0); 
						return 0; 
					}

					if(strcmp(pstData[0],"Mean square error")==0)
						linearInterpolation=1;
					else if(strcmp(pstData[0], "Nearest")==0)
						linearInterpolation=0;
					else
					{
						Scierror(999, "Error: Please provide proper value for \"%s\". Permissible values are Linear or Nearest.\n", currentArg); 
						return 0; 
					}
					providedArgs[3]=1;
				}
				else if(providedArgs[3])
				{
					Scierror(999, "Please provide optional arguments only once.\n"); 
					return 0; 
				}	
			}
			else if(strcmp(currentArg, "CellSize")==0)
			{
				if(!providedArgs[4])
				{
					sciErr = getVarAddressFromPosition(pvApiCtx, ++iter, &piAddr); 
					if (sciErr.iErr)
					{
						printError(&sciErr, 0); 
						return 0; 
					}
					if(!isDoubleType(pvApiCtx, piAddr) || isVarComplex(pvApiCtx, piAddr))
					{
						Scierror(999, "%s: Wrong type for input argument #%d: A real matrix expected.\n", fname, iter); 
						return 0; 
					}
					sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &iRows, &iCols, &cellSize); 
					if(sciErr.iErr)
					{
						printError(&sciErr, 0); 
						return 0; 
					}
					if(iRows!=1 || iCols!=2)
					{
						Scierror(999, "Incorrect dimension of matrix for argument CellSize.\n"); 
						return 0; 
					}
					providedArgs[4]=1;
				}
				else if(providedArgs[4])
				{
					Scierror(999, "Please provide optional arguments only once.\n"); 
					return 0; 
				}
			}
			else if(strcmp(currentArg, "Normalization")==0)
			{
				if(!providedArgs[5])
				{
					sciErr = getVarAddressFromPosition(pvApiCtx, ++iter, &piAddr); 
					if (sciErr.iErr)
					{
						printError(&sciErr, 0); 
						return 0; 
					}

					sciErr = getMatrixOfString(pvApiCtx, piAddr, &iRows, &iCols, NULL, NULL); 
					if (sciErr.iErr)
					{
						printError(&sciErr, 0); 
						return 0; 
					}

					piLen = (int*) malloc(sizeof(int) * iRows * iCols); 

					sciErr = getMatrixOfString(pvApiCtx,  piAddr,  &iRows,  &iCols,  piLen,  NULL); 
					if (sciErr.iErr)
					{
						printError(&sciErr, 0); 
						return 0; 
					}

					pstData = (char**) malloc(sizeof(char*) * iRows * iCols); 
					for(int iterPstData = 0; iterPstData < iRows * iCols; iterPstData++)
					{
						pstData[iterPstData] = (char*) malloc(sizeof(char) * piLen[iterPstData] + 1); 
					}

					sciErr = getMatrixOfString(pvApiCtx, piAddr, &iRows, &iCols, piLen, pstData); 
					if (sciErr.iErr)
					{
						printError(&sciErr, 0); 
						return 0; 
					}

					if(strcmp(pstData[0],"L2")==0)
						normalization=1;
					else if(strcmp(pstData[0], "None")==0)
						normalization=0;
					else
					{
						Scierror(999, "Error: Please provide proper value for \"%s\". Permissible values are L2 or None.\n", currentArg); 
						return 0; 
					}
					providedArgs[5]=1;
				}
				else if(providedArgs[5])
				{
					Scierror(999, "Please provide optional arguments only once.\n"); 
					return 0; 
				}
			}
		}
		//convert RGB image to gray
		try
		
		{
		//resize(image, image, Size(64,64) ); 
		image.convertTo(image,CV_8UC1);
		//sciprint("%d\n%d\n", image.rows,image.cols);
		//cvtColor(image, grayscaleImage, CV_RGB2GRAY);
		ELBP(image, lbpImage, radius, neighbors);
		//image.release();
                //sciprint("%d\n%d\n", lbpImage.rows,lbpImage.cols);
		}
        	catch (cv::Exception & e) 
                {

                    Scierror(999,e.what());
                    return 0;

                }

		
		if(!providedArgs[4])
		{
			cellSize[0] = lbpImage.rows;
			cellSize[1] = lbpImage.cols;
		}

		
		//calculate the spatial histogram
		
		//try
		//{
		spatialHistogram = spatial_histogram(lbpImage, static_cast<int>(pow(2.0, static_cast<double>(neighbors))), int(cellSize[0]), int(cellSize[1]), normalization);
		//lbpImage.release();
		sciprint("%d\n%d\n", spatialHistogram.rows,spatialHistogram.cols);
		//sciprint("guru");
		outputHist = (double*)malloc(sizeof(double) * spatialHistogram.cols);
		//sciprint("guru1");
		for(int i=0;i<spatialHistogram.cols;i++)
		    {	outputHist[i] = spatialHistogram.at<double>(0,i);	
                        	//sciprint("%f\n",outputHist[i]);
                    }
                //sciprint("guru3");
                //}
                	
               /* catch(cv::Exception &e)
                {
                  const char *err = e.what();
                  sciprint("%s",err);
                }*/
		
		//sciprint("guru4");
		
		
		//spatialHistogram.release();
		
		//create output argument
		sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx)+1, 1, spatialHistogram.cols, outputHist); 
		
        if(sciErr.iErr)
        {
            printError(&sciErr, 0); 
            return 0; 
        }
          
		//return output Arguments
		AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
		ReturnArguments(pvApiCtx);
		//free(outputHist);
		return 0;
	}
	/* ==================================================================== */
}
