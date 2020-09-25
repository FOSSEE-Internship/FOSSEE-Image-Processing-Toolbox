//histeq function is for grayscale image
/* ==================================================================== */
/* Authors :Priyanka Hiranandani NIT Surat, Shubham Lohakare NITK Surathkal                               */
/* ==================================================================== */
/* Syntax : return_image=histeq(rgbimage);
/* ==================================================================== */
#include <numeric>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
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
	#include<string.h>
	#include "../common.h"
	int opencv_histeq(char *fname, unsigned long fname_len)
	{

    		// Error management variable
        	SciErr sciErr;
    	
    		//checking input argument
         	CheckInputArgument(pvApiCtx,1,1);
    		//checking output argument
         	CheckOutputArgument(pvApiCtx,1,1) ;
    		//for first argument 
            	Mat img,p,dst;

            	retrieveImage(img,1);//Retrieving the input image
		try{	    	
			img.convertTo(img,CV_8U);//Converting the image to 8-bit image
	    		cvtColor(img,img,CV_BGR2GRAY);//Converting to grayscale
            	 
            		//open cv function to equalize the histogram
            		equalizeHist(img,dst);
		   }
		catch(Exception& e)
		{
			const char* err=e.what();
			printf("%s",err);
		}	 
            
		string tempstring = type2str(dst.type());
            	char* checker = (char *)malloc(tempstring.size() + 1);
            	memcpy(checker, tempstring.c_str(), tempstring.size() + 1);
            	returnImage(checker,dst,1);
            	AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
            	ReturnArguments(pvApiCtx);
            
    } 
}
/* ==================================================================== */

