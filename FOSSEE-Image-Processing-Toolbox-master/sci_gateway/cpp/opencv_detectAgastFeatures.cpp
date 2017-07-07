/*************************************************************************************************************************************************************************************************
*	Author: Shreyash Sharma																					 *
*	Code: detectAgastFeatures.cpp																				 *
*	Function Format: detectAgastFeatures( Image, Optional Arguments... )															 *
* 		Requirements: 		Image should be a grayscale image															 *
*		Optional Arguments:	1. 'MinContrast'	Value: Range 0 < x < 1														 *
*					2. 'MinQuality'		Value: Range 0 < x < 1														 *
*					3. 'ROI'		Value: [ x_cordinate, y_cordinate, width, height ]										 *
*************************************************************************************************************************************************************************************************/


#include <iostream>
#include <numeric>
#include <vector>
#include <string>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
//#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/nonfree/nonfree.hpp"

using namespace cv;
using namespace std;

extern "C"
{
	#include "api_scilab.h"
	#include "Scierror.h"
	#include "BOOL.h"
	#include <localization.h>
	#include "sciprint.h"
	//#include "../common.h"

	// Function using OpenCV to detect FAST Features
	int opencv_detectAgastFeatures(char *fname, unsigned long fname_len)
	{
		int *piAddr = NULL;

		//Various Declarations
		SciErr sciErr;			//Error Variable
		int sca_read;			//Error Variable

		int *imgAddr = NULL;		//Address of the Image List Argument
		int *piAddrChild = NULL;	//Address of the Child of the Image Matrix List
		int no_item_list;		//Number of items in the list

		int iPrec = 0;			//Precesision of the image in the lsit
		int iRows=0, iCols=0;		//Rows & Columns in the image

		int inp_params;			//Number of Input Arguments
		int *piAddr1 = NULL;		//Address of Argument Head
		int *piAddr2 = NULL;		//Address of Value_1 of Argument
		//int *piAddr3 = NULL;		//Address of Value_2 of Argument
		//int *piAddr4 = NULL;		//Address of Value_3 of Argument
		//int *piAddr5 = NULL;		//Address of Value_4 of Argument
		int Rows,Cols;			//Rows & Columns of String Matrix
		int *len = NULL;		//Length of each String Element
		char **arg;			//String Argument
		double *roi_arg = NULL;		//ROI Argument Matrix Address
		int rows,cols;			//Rows and Columns of ROI value matrix

		//int *point = NULL;		//Points returned by the FAST Algorithm

		double minq  =0.1;                //Variables for Contrast/Quality Argument
		double minc  =0.2;
		double type = 2;
					
		double x = 0, y = 0, width, height;	//Variables for ROI Argument

		double thresh;			//Effective Threshold
		double thresh_c = 0;		//Threshold due to arguments
		double thresh_q = 0;		//Threshold due to arguments

		int count_q = 0;		//Count of Argument Quality Call
		int count_c = 0;		//Count of Argument Contrast Call
		int count_r = 0;
		int count_t = 0;		//Count of Argument ROI call

		vector<KeyPoint> keypoints;	//Keypoint Vector to store the points returned by FAST Algorithm

		int i,j;			//Iterators

		double count = 0;			//Count of number of Features Detected
		double *metric = NULL;

		Mat image;			//Image Container
		Mat cropped;			//Image Container

		//Checks on Input and Output Arguments
		CheckInputArgument(pvApiCtx, 1, 9);
		CheckOutputArgument(pvApiCtx, 1, 3);


		//Image Retrieval
		//Get Address of the Image List
		sciErr = getVarAddressFromPosition(pvApiCtx, 1, &imgAddr);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
		//Get Number of items in the list
		sciErr = getListItemNumber(pvApiCtx, imgAddr, &no_item_list);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
		//Check if it is a Grayscal image by checking whether 1 item is present in the list
		if( no_item_list == 1)
		{
			//Address of the Grayscale Matrix
			sciErr = getListItemAddress(pvApiCtx, imgAddr, 1, &piAddrChild);
    			if(sciErr.iErr)
    			{
        			printError(&sciErr, 0);
        			return 0;
    			}
			//Check whether it is an integer Matrix or not
			if(isIntegerType(pvApiCtx, piAddrChild))
    			{
        			//If integer matrix getting precision
        			sciErr = getMatrixOfIntegerPrecision(pvApiCtx, piAddrChild, &iPrec);
        			if(sciErr.iErr)
        			{
            			printError(&sciErr, 0);
            			return 0;
        			}
				//Based on precision Read the matrix
        			switch(iPrec)
				{
				case SCI_UINT8: //for unsigned integer 8
            				{
					unsigned char *pstDatagray = NULL;
               				sciErr = getMatrixOfUnsignedInteger8InList(pvApiCtx, imgAddr, 1, &iRows, &iCols, &pstDatagray);
               				if(sciErr.iErr)
               				{
                       				printError(&sciErr, 0);
                       				return 0;
               				}
					image = Mat(iRows,iCols,CV_8UC1);
					for(i=0;i<iRows;i++)
                       				for(j=0;j<iCols;j++)
                       					image.at<uchar>(i,j)=pstDatagray[i+iRows*j];
					break;
					}
            			case SCI_UINT16: //for unsigned integer 16
            				{
                    			short unsigned int *pstDatagray = NULL;
                    			sciErr = getMatrixOfUnsignedInteger16InList(pvApiCtx, imgAddr, 1, &iRows, &iCols, &pstDatagray);
                    			if(sciErr.iErr)
                    			{
                        			printError(&sciErr, 0);
                        			return 0;
                    			}
                               		image = Mat(iRows,iCols,CV_16UC1);
                    			for(i=0;i<iRows;i++)
                        			for(j=0;j<iCols;j++)
                       					image.at<ushort>(i,j)=pstDatagray[i+iRows*j];
                			break;
            				}
            			case SCI_INT32: //for unsigned integer 32
            				{
                    			int *pstDatagray = NULL;
                    			sciErr = getMatrixOfInteger32InList(pvApiCtx, imgAddr, 1, &iRows, &iCols, &pstDatagray);
                    			if(sciErr.iErr)
                    			{
                        			printError(&sciErr, 0);
                        			return 0;
                    			}
		                    	image = Mat(iRows,iCols,CV_32SC1);
               				for(i=0;i<iRows;i++)
                       				for(j=0;j<iCols;j++)
                       					image.at<int>(i,j)=pstDatagray[i+iRows*j];
       	 				break;
       					}
				}
			}
			else
                	{
				//If not Integer then must be double
                        	double *pstDatagray = NULL;
                        	sciErr = getVarAddressFromPosition(pvApiCtx, 1, &imgAddr);
                        	if(sciErr.iErr)
                        	{
                              		printError(&sciErr, 0);
                               		return 0;
                       		}
                       		sciErr = getMatrixOfDoubleInList(pvApiCtx, imgAddr, 1, &iRows, &iCols, &pstDatagray);
                       		if(sciErr.iErr)
                       		{
                               		printError(&sciErr, 0);
                               		return 0;
                       		}
                       		image = Mat(iRows,iCols,CV_64FC1);
                       		for(i=0;i<iRows;i++)
                               		for(j=0;j<iCols;j++)
                                       		image.at<double>(i,j)=pstDatagray[i+iRows*j];
               		}
		}
		//If not single element in list than it is not a grayscale image and hence error
		else
		{
			Scierror(999,"Expecting a single Matrix in grayscale format\n");
			return 0;
		}

		//Get the Number of Arguments
		inp_params = *getNbInputArgument(pvApiCtx);

		//Based on Number of Arguments Evaluate each Argument
		for(i=2; i<inp_params; i++)
		{
			//Reading the Name of the Argument
			sciErr = getVarAddressFromPosition(pvApiCtx, i, &piAddr1);
			if(sciErr.iErr)
			{
				printError(&sciErr, 0);
				return 0;
			}
			//Check for Argument type
			if( !isStringType(pvApiCtx, piAddr1))
			{
				Scierror(999, "%s: Wrong type of argument #%d. A string is expected.\n", fname, 1);
				return 0;
			}
			//Matrix of Stings
			sciErr = getMatrixOfString(pvApiCtx, piAddr1, &Rows, &Cols, NULL, NULL);
			if(sciErr.iErr)
			{
				printError(&sciErr, 0);
				return 0;
			}
			len = (int*)malloc(sizeof(int) * Rows * Cols);
			//second call to retrieve the length of the string
			sciErr = getMatrixOfString(pvApiCtx, piAddr1, &Rows, &Cols, len, NULL);
			if(sciErr.iErr)
			{
				printError(&sciErr, 0);
				free(len);
				return 0;
			}
			arg = (char**)malloc(sizeof(char*) * Rows * Cols);
			for(int j=0;j< Rows * Cols; j++)
			{
				arg[j] = (char*)malloc(sizeof(char) * (len[j] + 1));
			}

			//third call to retrieve data
			sciErr = getMatrixOfString(pvApiCtx, piAddr1, &Rows, &Cols, len, arg);
			if(sciErr.iErr)
			{
				printError(&sciErr, 0);
				free(len);
				free(arg);
				return 0;
			}

			//Evaluating the Read Arguments Name
			//MinContrast Argument
			if(strcmp(arg[0],"MinContrast") == 0)
			{
				if(count_c == 0)
				{
				sciErr = getVarAddressFromPosition(pvApiCtx, i+1, &piAddr2);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				//Checking the type of value
				if( !isDoubleType(pvApiCtx, piAddr2))
				{
					Scierror(999,"Not a valid type of value.\n");
					return 0;
				}
				//Reading the Value of the argument
				sca_read = getScalarDouble(pvApiCtx, piAddr2, &minc);
				if(sca_read)
				{
					Scierror(999,"Not a valid value.\n");
					return 0;
				}
				//Check for valid range
				if(minc<0 && minc>=1)
				{
					Scierror(999,"Not a valid range.Should be between 0 and 1.\n");
					return 0;
				}
				//Threshold Calculation
				else thresh_c = 256*minc;
				i++;							//Incrementing iterator to count for the value argument read
				count_c += 1;						//Indicating MinContrast has been called once
				}
				else
				{
					Scierror(999,"Specified MinContrast twice. Check Arguments\n");
					return 0;
				}
			}
			//MinQuality Argument
			else if(strcmp(arg[0],"MinQuality") == 0)
			{
				if(count_q == 0)
				{
				sciErr = getVarAddressFromPosition(pvApiCtx, i+1, &piAddr2);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				//Checking the type of Value
				if( !isDoubleType(pvApiCtx, piAddr2))
				{
					Scierror(999,"Not a valid type of value.\n");
					return 0;
				}
				//Reading the Value of argument
				sca_read = getScalarDouble(pvApiCtx, piAddr2, &minq);
				if(sca_read)
				{
					Scierror(999,"Not a valid type of value.\n");
					return 0;
				}
				//Cheack for valid range
				if(minq<0 && minq>=1)
				{
					Scierror(999,"Not a valid range.Should be between 0 and 1.\n");
					return 0;
				}
				//Threshold Calculation
				else thresh_q = 240*minq;
				i++;							//Incrementing iterator to count for the value argument read
				count_q += 1;						//Indicating MinQuality has been called once
				}
				else
				{
					Scierror(999,"Specified MinQuality twice. Check Arguments\n");
					return 0;
				}
			}
			
			//MinQuality Argument
			else if(strcmp(arg[0],"Type") == 0)
			{
				if(count_t == 0)
				{
				sciErr = getVarAddressFromPosition(pvApiCtx, i+1, &piAddr2);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				//Checking the type of Value
				if( !isDoubleType(pvApiCtx, piAddr2))
				{
					Scierror(999,"Not a valid type of value.\n");
					return 0;
				}
				//Reading the Value of argument
				sca_read = getScalarDouble(pvApiCtx, piAddr2, &type);
				if(sca_read)
				{
					Scierror(999,"Not a valid type of value.\n");
					return 0;
				}
				/*//Cheack for valid range
				if(arg_val<0 && arg_val>=1)
				{
					Scierror(999,"Not a valid range.Should be between 0 and 1.\n");
					return 0;
				}
				//Threshold Calculation
				else thresh_q = 240*arg_val;*/
				i++;							//Incrementing iterator to count for the value argument read
				count_t += 1;						//Indicating MinQuality has been called once
				}
				else
				{
					Scierror(999,"Specified MinQuality twice. Check Arguments\n");
					return 0;
				}
			}
			//ROI Argument
			else if(strcmp(arg[0],"ROI") == 0)
			{
				if(count_r == 0)
				{
				sciErr = getVarAddressFromPosition(pvApiCtx, i+1, &piAddr2);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				//Checking the type of argument
				if( !isDoubleType(pvApiCtx, piAddr2))
				{
					Scierror(999,"Not a valid type of value.\n");
					return 0;
				}
				//Reading the Value of the argument
				sciErr = getMatrixOfDouble(pvApiCtx, piAddr2, &rows, &cols, &roi_arg);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				//Assigning the elements of the list to their proper function
				x = roi_arg[0];
				y = roi_arg[1];
				width = roi_arg[2];
				height = roi_arg[3];


				//Code for reading the value associated with ROI individually rather than in a list form

				/*sca_read = getScalarDouble(pvApiCtx, piAddr2, &x);
				if(sca_read)
				{
					Scierror(999,"Not a valid type of value.\n");
					return 0;
				}
				sciErr = getVarAddressFromPosition(pvApiCtx, i+2, &piAddr3);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				sca_read = getScalarDouble(pvApiCtx, piAddr3, &y);
				if(sca_read)
				{
					Scierror(999,"Not a valid type of value.\n");
					return 0;
				}
				sciErr = getVarAddressFromPosition(pvApiCtx, i+3, &piAddr4);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				sca_read = getScalarDouble(pvApiCtx, piAddr4, &width);
				if(sca_read)
				{
					Scierror(999,"Not a valid type of value.\n");
					return 0;
				}
				sciErr = getVarAddressFromPosition(pvApiCtx, i+4, &piAddr5);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				sca_read = getScalarDouble(pvApiCtx, piAddr5, &height);
				if(sca_read)
				{
					Scierror(999,"Not a valid type of value.\n");
					return 0;
				}*/
                                 
                                      
				//Checking for valid crop area
				if(x>=image.cols || y>=image.rows || x<0 || y<0)
   			 	{
        				sciprint("Invalid x or y value\n");
        				return 0;
    				}
    				if(width<=0 || height<=0 || x+width > image.cols || y+height > image.rows)
    				{
        				sciprint("Invalid width or height value\n");
        				return 0;
    				}
				//Croppint the image
				Rect myROI(x, y, width, height);
				Mat croppedRef(image, myROI);
				croppedRef.copyTo(cropped);
				i++;						//Incrementing iterator to count for the value argument read
				count_r += 1;						//Indicating ROI has been called once
				}
				else
				{
					Scierror(999,"Specified ROC twice. Check Arguments\n");
					return 0;
				}
			}
			else
			{
				Scierror(999,"Invalid Argument\n");
				return 0;
			}

		}

		//Calculating the effective threshold by selecting the maximum threshold
		if(thresh_c > 0 && thresh_c >= thresh_q)
			thresh = thresh_c;
		else if(thresh_q > 0 && thresh_q > thresh_c)
			thresh = thresh_q;
		else thresh = 256*0.2;
                
                int t = int(thresh);
		Ptr<AgastFeatureDetector> fd = AgastFeatureDetector::create(t, true);
		image.convertTo(image, CV_8U);
		if(count_r != 0)
		{	
			try
			{
				fd -> detect(cropped, keypoints);
			}
			catch(Exception &e)
			{
				const char *err = e.what();
				Scierror(999, "%s", err);
				return 0;
			}
		}
		else
		{	
			try
			{	
				int t = int(thresh);
				fd -> detect(image, keypoints);
			}
			catch(Exception &e)
			{
				const char *err = e.what();
				Scierror(999, "%s", err);
				return 0;
			}
		}

		count = keypoints.size();

		metric = (double*)malloc(sizeof(double)*keypoints.size()*1);
		for( i=0; i<keypoints.size(); i++)
		{
			metric[i] = keypoints[i].response;
		}

		//Returning the obtained point co-ordinates
		//Checking what type of Input matrix image was
		//If input matrix was integer
		if(isIntegerType(pvApiCtx, piAddrChild))
		{
			//Based on Precision of the Image given in input
			switch(iPrec)
                                {
				case SCI_UINT8: {
							//Output point Matrix
							unsigned char *point = (unsigned char *)malloc(keypoints.size()*2*sizeof(char));
							for(i=0;i!=keypoints.size();i++)
         				      		{
                        					point[i] = keypoints[i].pt.x + x;
                        					point[keypoints.size()+i] = keypoints[i].pt.y + y;
               						}
							sciErr = createMatrixOfUnsignedInteger8(pvApiCtx, nbInputArgument(pvApiCtx) + 1, int(keypoints.size()), 2, point);
							if(sciErr.iErr)
							{
								printError(&sciErr, 0);
								return 0;
							}
							free(point);
							break;
						}
				case SCI_UINT16:{
							//Output point Matrix
							unsigned short *point = (unsigned short *)malloc(keypoints.size()*2*sizeof(short));
							for(i=0;i!=keypoints.size();i++)
                					{
                        					point[i] = keypoints[i].pt.x + x;
                        					point[keypoints.size()+i] = keypoints[i].pt.y + y;
                					}
							sciErr = createMatrixOfUnsignedInteger16(pvApiCtx, nbInputArgument(pvApiCtx) + 1, int(keypoints.size()), 2, point);
							if(sciErr.iErr)
							{
								printError(&sciErr, 0);
								return 0;
							}
							free(point);
							break;
						}
				case SCI_INT32: {
							//Output Point Matrix
							int *point = (int *)malloc(keypoints.size()*2*sizeof(int));
							for(i=0;i!=keypoints.size();i++)
                					{
                        					point[i] = keypoints[i].pt.x + x;
                        					point[keypoints.size()+i] = keypoints[i].pt.y + y;
                					}
							sciErr = createMatrixOfInteger32(pvApiCtx, nbInputArgument(pvApiCtx) + 1, int(keypoints.size()), 2, point);
							if(sciErr.iErr)
							{
								printError(&sciErr, 0);
								return 0;
							}
							free(point);
							break;
						}
				}
		}
		//If not Integer than must be double input image
		else
		{
			//Ouput point matrix
			double *point = (double *)malloc(keypoints.size()*2*sizeof(double));
			for(i=0;i!=keypoints.size();i++)
                	{
                        	point[i] = keypoints[i].pt.x + x;
                    	    	point[keypoints.size()+i] = keypoints[i].pt.y + y;
                	}
			sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 1, keypoints.size(), 2, point);
			if(sciErr.iErr)
			{
				printError(&sciErr, 0);
				return 0;
			}
			free(point);
		}

		sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 2, 1, 1, &count);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}

		sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 3, keypoints.size(), 1, metric);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}

		//Returning the Output Matrix
		AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
		AssignOutputVariable(pvApiCtx, 2) = nbInputArgument(pvApiCtx) + 2;
		AssignOutputVariable(pvApiCtx, 3) = nbInputArgument(pvApiCtx) + 3;

		ReturnArguments(pvApiCtx);
		return 0;
	}
}
