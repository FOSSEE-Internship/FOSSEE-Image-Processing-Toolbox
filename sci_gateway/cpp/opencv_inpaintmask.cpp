/*
This is the .cpp gateway file for the 'iminpaint' scilab function.
It includes the OpenCV function void cv::inpaint ( InputArray src,InputArray inpaintMask,OutputArray dst,double inpaintRadius,int flags)
The function reconstructs the selected image area from the pixel near the area boundary. The function may be used to remove dust and scratches from a scanned photo, or to remove undesirable objects from still images.

Author: Shubham Lohakare, NITK Surathkal
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

  	int opencv_inpaintmask(char *fname, unsigned long fname_len)
  	{	
		SciErr sciErr;
		int i;
	
		Mat img1,img;
		Mat inpainted;
		/*int *piAddr2 = NULL;//Variables for 3rd argument
		int intErr2 = 0;
		double radius = 5;

		int *piAddr3 = NULL;//Variables for 4th argument
		int intErr3 = 0;
		double methodNum=0;
		*/
		CheckInputArgument(pvApiCtx, 1, 1);//Checking the number of input arguments
		CheckOutputArgument(pvApiCtx, 1, 1);//Checking the number of output arguments
		
		//To get the argument #2
		/*sciErr = getVarAddressFromPosition(pvApiCtx, 2, &piAddr2);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
		intErr2 = getScalarDouble(pvApiCtx, piAddr2, &radius);
		if(intErr2)
		{
			return intErr2;
		}

		//To get the argument #3
		sciErr = getVarAddressFromPosition(pvApiCtx, 3, &piAddr3);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
		intErr3 = getScalarDouble(pvApiCtx,piAddr3, &methodNum);
		if(intErr3)
		{
			return intErr3;
		}
		*/
		try
		{	
			retrieveImage(img1,1);//Retrieving the input image
			img1.convertTo(img1,CV_8U);//Converting the image to 8Bit
			
			cvtColor(img1,img,CV_BGR2GRAY);
			threshold(img,img,30,255,THRESH_BINARY_INV);
			//retrieveImage(img2,2);//Retrieving the mask for the image
			//img2.convertTo(img2,CV_8U);//Converting the mask to 8Bit
			//cvtColor(img2,img,CV_BGR2GRAY);//Converting the mask to a grayscale image
			
			    int largest_area=0;
   				int largest_contour_index=0;
    Rect bounding_rect;	
    vector<vector <Point> > contours; // Vector for storing contour
    vector<Vec4i> hierarchy;
    findContours( img, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
    // iterate through each contour.
    for( int i = 0; i< contours.size(); i++ )
    {

    //  Find the area of contour
    double a=contourArea( contours[i],false); 
    if(a>largest_area){
        largest_area=a;//cout << i<<" area  "<<a<<endl;
        // Store the index of largest contour
        largest_contour_index=i;               
        // Find the bounding rectangle for biggest contour
        bounding_rect=boundingRect(contours[i]);
    }
}

Scalar color(255,255,255);  // color of the contour in the
//Draw the contour and rectangle
drawContours( img1, contours,largest_contour_index, color, CV_FILLED,8,hierarchy);
rectangle(img1, bounding_rect,  Scalar(0,255,0),2, 8,0);


inpaint(img1, img, inpainted, 3, INPAINT_NS);


			/*if(methodNum==1)
				inpaint(img1,img,outputImage,radius,INPAINT_NS);//The inpaint method follows NS algorithm
		
			else if(methodNum==2)
				inpaint(img1,img,outputImage,radius,INPAINT_TELEA);//The inpaint method follows TELEA algorithm
*/
			string tempstring = type2str(inpainted.type());//Returning the output image to Scilab console
            		char* checker = (char *)malloc(tempstring.size() + 1);
            		memcpy(checker, tempstring.c_str(), tempstring.size() + 1);
            		returnImage(checker,inpainted,1);
            		AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
            		ReturnArguments(pvApiCtx);
		}
		catch(Exception &e)
		{
			const char* err=e.what();
			sciprint("%s",err);
		}
	}
}
