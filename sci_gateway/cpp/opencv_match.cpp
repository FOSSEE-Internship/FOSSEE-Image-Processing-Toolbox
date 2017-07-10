/********************************************************
Author: Manoj Sree Harsha 
********************************************************/
#include <numeric>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/stitching/detail/matchers.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <iostream>

using namespace cv;
using namespace std;
using namespace cv::xfeatures2d;
extern "C"
{
  #include "api_scilab.h"
  #include "Scierror.h"
  #include "BOOL.h"
  #include <localization.h>
  #include "sciprint.h"
  #include "../common.h"

  int opencv_match(char *fname, unsigned long fname_len)
  {			

      // Error management variables	
		SciErr sciErr;

      //variable declarations
		int *piAddr1  = NULL;
    int *piAddr2  = NULL;
    double *result=NULL;

      //Checking number of input and output arguments (enviromnet variable, min arguments, max arguments)
   	  	CheckInputArgument(pvApiCtx, 2, 2);
      	CheckOutputArgument(pvApiCtx, 1, 1);

     	Mat image1;
	    retrieveImage(image1, 1);
     	Mat image2;
    	retrieveImage(image2, 2);

      //Application Code
    	try
    	{
    	  Ptr<SURF> surf = SURF::create(5000);
    		vector<KeyPoint> keypoints1, keypoints2;
    		Mat descriptors1, descriptors2;
    		Mat img1,img2;
    		image1.convertTo(img1,CV_8U);
    		image2.convertTo(img2,CV_8U);
		  	surf->detectAndCompute(img1, Mat(), keypoints1, descriptors1);
    		surf->detectAndCompute(img2, Mat(), keypoints2, descriptors2);
    		BFMatcher matcher(surf->defaultNorm());
    		cv::detail::MatchesInfo m1; 
    		matcher.match(descriptors1, descriptors2, m1.matches);
    		Mat img_matches;
    		drawMatches(img1, keypoints1, img2, keypoints2,m1.matches, img_matches);
  			string tempstring = type2str(img_matches.type());
  			char *checker;
  			checker = (char *)malloc(tempstring.size() + 1);
  			memcpy(checker, tempstring.c_str(), tempstring.size() + 1);
  			returnImage(checker, img_matches, 1);
  			free(checker);

        //Assigning output variables
  			AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
        
        //Returning the Output Variables as arguments to the Scilab environment
  			ReturnArguments(pvApiCtx);	
	    }
	   catch(Exception& e)
		{
  			const char* err=e.what();
  		  	Scierror(999,("%s",err));
		} 		

  }
}
