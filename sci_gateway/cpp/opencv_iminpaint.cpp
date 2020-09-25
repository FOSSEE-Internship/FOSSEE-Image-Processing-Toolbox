/*
Author : Shubham Lohakare, NITK Surathkal

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

  	int opencv_iminpaint(char *fname, unsigned long fname_len)
  	{	
		//Declaring Variables
		SciErr sciErr;
	
		Mat img1,img2,outputImage;
		
		int *piAddr3 = NULL;
		int intErr3 = 0;
		double radius = 5;

		int *piAddr4 = NULL;
		int intErr4 = 0;
		double methodNum=0;
		int *addvartwo=NULL;

		CheckInputArgument(pvApiCtx, 4, 4);//Checking the input arguments
		CheckOutputArgument(pvApiCtx, 1, 1);//Checking the output arguments
		
	
		//Getting argument #3
		sciErr = getVarAddressFromPosition(pvApiCtx, 3, &piAddr3);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
		intErr3 = getScalarDouble(pvApiCtx, piAddr3, &radius);
		if(intErr3)
		{
			return intErr3;
		}
		//Getting argument #4
		sciErr = getVarAddressFromPosition(pvApiCtx, 4, &piAddr4);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
		intErr4 = getScalarDouble(pvApiCtx,piAddr4, &methodNum);
		if(intErr4)
		{
			return intErr4;
		}
		try
		{
			retrieveImage(img1,1);//Retrieving the source image
			img1.convertTo(img1,CV_8U);//Converting the image to 8-bit image
		
			
		
			retrieveImage(img2,2);//Retrieving the mask
			img2.convertTo(img2,CV_8U);//Converting the mask
		
			if(methodNum==1)
				inpaint(img1,img2,outputImage,radius,INPAINT_NS);//Calling the opencv inpaint function with method inpaint_ns
			else if(methodNum==2)
				inpaint(img1,img2,outputImage,radius,INPAINT_TELEA);//Calling the opencv inpaint function with method inpaint_telea

			}
		catch(Exception &e)
		{
			const char* err=e.what();
			sciprint("%s",err);
		}
		//Assigning the output to be displayed on scilab console
		string tempstring = type2str(outputImage.type());
          	char* checker = (char *)malloc(tempstring.size() + 1);
            	memcpy(checker, tempstring.c_str(), tempstring.size() + 1);
            	returnImage(checker,outputImage,1);
            	AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
            	ReturnArguments(pvApiCtx);
	}
}
