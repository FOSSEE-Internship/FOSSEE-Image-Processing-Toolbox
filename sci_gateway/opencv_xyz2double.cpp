// Authors
// Ashish Manatosh Barik, NIT Rourkela	
#include <numeric>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
#include<stdio.h>
#include <sciprint.h>
#include<string.h>
#include<math.h>

using namespace cv;
using namespace std;

extern "C"
{
	#include "api_scilab.h"
  	#include "Scierror.h"
  	#include "BOOL.h"
  	#include <localization.h>
	#include "../common.h"

	int opencv_xyz2double(char *fname, unsigned long fname_len)	
	{
 		// Error management variable
        	SciErr sciErr;
		
		// to read input data as a color space
		Mat img;

		// to get the input array as a list of rgb values
		retrieveImage(img, 1);

		if(img.cols > 3 || img.cols < 3)
		{
			Scierror(999, "inpput should be a M by 3 or M by N by 3 \n", 1);
			return -1;
		}

		int type = img.type();
		if(type != 2)
		{
			// if input is not uint16, no conversion takes places.
			
			// to return the output 
			string tempstring1 = type2str(img.type());
			char *checker1;
			checker1 = (char *)malloc(tempstring1.size() + 1);
			memcpy(checker1, tempstring1.c_str(), tempstring1.size() + 1);
			returnImage(checker1, img, 1);
			free(checker1);
			AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;

			ReturnArguments(pvApiCtx);

			return 0;
		}

		// output
		Mat image;

		try
		{
			// OpenCV functionalities

			// conversion to double precision with proper scaling
			img.convertTo(image, CV_64FC3, 1.0/32768);
		}
		catch(Exception& e)
		{
			const char* err=e.what();
			Scierror(999, "%s", err);
		}	

		// to return the output 
		string tempstring1 = type2str(image.type());
		char *checker1;
		checker1 = (char *)malloc(tempstring1.size() + 1);
		memcpy(checker1, tempstring1.c_str(), tempstring1.size() + 1);
		returnImage(checker1, image, 1);
		free(checker1);
		AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;

		ReturnArguments(pvApiCtx);

		return 0;
	}
}
	
