/*
This is a .cpp gateway file for the 'imdecolor' scilab function.
It uses the OpenCV function : cv::decolor (InputArray src, OutputArray grayscale, OutputArray color_boost)
It takes an image as input and gives 2 output images. One is a grayscale image and the other a color boosted image.

Author : Shubham Lohakare, NITK Surathkal
	 Ashish Manatosh Barik, NIT Rourkela
*/
#include <numeric>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
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

	  int opencv_imdecolor(char *fname, unsigned long fname_len)
	  {	SciErr sciErr;	
		int i;

		
		
         	CheckInputArgument(pvApiCtx, 1, 1);//Check The number of input arguments
    		CheckOutputArgument(pvApiCtx, 2, 2) ;//Check The number of output arguments

		Mat src, dst,colout;//Declaring variables

		try{
			retrieveImage(src,1);//Getting the input image
			src.convertTo(src,CV_8U);//Converting the image to 8bit
	
			decolor(src,dst,colout);//OpenCV function. Gives 2 outputs. In dst a decolored image is stored and in colout a color boosted image is stored.
		}
		catch(Exception &e)
		{
			const char* err=e.what();
			sciprint("%s",err);
		}
	
		string tempstring1 = type2str(dst.type());//For returning the first image as output
            	char* checker1;
		checker1 = (char *)malloc(tempstring1.size() + 1);
            	memcpy(checker1, tempstring1.c_str(), tempstring1.size() + 1);
            	returnImage(checker1,dst,1);
	    	free(checker1);
            	AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
	
		string tempstring2 = type2str(colout.type());//For returning the second image as output
            	char* checker2;
		checker2 = (char *)malloc(tempstring2.size() + 1);
            	memcpy(checker2, tempstring2.c_str(), tempstring2.size() + 1);
            	returnImage(checker2,colout,2);
		free(checker2);
        	AssignOutputVariable(pvApiCtx, 2) = nbInputArgument(pvApiCtx) + 2;
    
        	ReturnArguments(pvApiCtx);
            
		return 0; 
	
  }

}
