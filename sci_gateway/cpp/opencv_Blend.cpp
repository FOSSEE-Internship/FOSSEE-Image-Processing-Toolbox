//Author-M Avinash Reddy

#include <numeric>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/stitching/detail/blenders.hpp"
#include "opencv2/imgproc/imgproc.hpp"
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

int opencv_Blend(char *fname,unsigned long fname_len){
	SciErr sciErr;
	int intErr=0;
	int *piaddrvar1=NULL;
	int *piaddrvar2=NULL;
	int *piaddrvar3=NULL;
	int *piaddrvar4=NULL;
	double w1=0,w2=0;
	CheckInputArgument(pvApiCtx, 4, 4); // Checking the number of input and output arguments
    CheckOutputArgument(pvApiCtx, 1, 1);

    try{
		Mat image1,image2,image1s,image2s,I;
    	retrieveImage(image1, 1);
    	retrieveImage(image2, 2);

	    sciErr = getVarAddressFromPosition(pvApiCtx, 3, &piaddrvar3);
	    if (sciErr.iErr)
	    {
	        printError(&sciErr, 0);
	        return 0;
	    }
	    if ( !isDoubleType(pvApiCtx, piaddrvar3))
	    {
	        Scierror(999, _("%s: Wrong type for input argument #%d: A scalar expected.\n"), fname, 3);
	        return 0;
	    }

	    intErr = getScalarDouble(pvApiCtx, piaddrvar3,&w1);
	    if (intErr)
	    {
	        
	        return intErr;
	    }    

	    sciErr = getVarAddressFromPosition(pvApiCtx, 4, &piaddrvar4);
	    if (sciErr.iErr)
	    {
	        printError(&sciErr, 0);
	        return 0;
	    }
	    if ( !isDoubleType(pvApiCtx, piaddrvar4))
	    {
	        Scierror(999, _("%s: Wrong type for input argument #%d: A scalar expected.\n"), fname, 3);
	        return 0;
	    }

	    intErr = getScalarDouble(pvApiCtx, piaddrvar4,&w2);
	    if (intErr)
	    {
	        
	        return intErr;
	    }	    
	    if(w1<0 || w1>100){
			Scierror(999, _("%s:Invalid input argument #%d: Weight should be between 0 and 100\n"), fname,3);
	        return 0;    	
	    }
	    if(w2<0 || w2>100){
			Scierror(999, _("%s:Invalid input argument #%d: Weight should be between 0 and 100\n"), fname,4);
	        return 0;    	
	    }

	    resize(image2,image2,image1.size());  


		image1.convertTo(image1s, CV_16S);
		image2.convertTo(image2s, CV_16S);

		Mat mask1(image1s.size(), CV_8U);
		mask1(Rect(0, 0, (mask1.cols*w1)/100, mask1.rows)).setTo(255);
		mask1(Rect((mask1.cols*w1)/100, 0, (mask1.cols*(100-w1))/100, mask1.rows)).setTo(0);

		Mat mask2(image2s.size(), CV_8U);
		mask2(Rect(0, 0, (mask2.cols*(100-w2))/100, mask2.rows)).setTo(0);
		mask2(Rect((mask2.cols*(100-w2))/100, 0, (mask2.cols*w2)/100, mask2.rows)).setTo(255);

	    cv::detail::FeatherBlender blender;
	    blender.prepare(Rect(0, 0, max(image1s.cols, image2s.cols), max(image1s.rows, image2s.rows)));
	    blender.feed(image1s, mask1, Point(0,0));
	    blender.feed(image2s, mask2, Point(0,0));

	    Mat result_s, result_mask;
	    blender.blend(result_s, result_mask);	

	    Mat result; 
	    result_s.convertTo(result, CV_8U);


        string tempstring = type2str(result.type());
        char *checker;
        checker = (char *)malloc(tempstring.size() + 1);
        memcpy(checker, tempstring.c_str(), tempstring.size() + 1);
        returnImage(checker,result,1);
        free(checker);	
     }

	catch(cv::Exception& e){
	 	const char* err=e.what();
	 	Scierror(999,("%s",err));
	 } 

    AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
    //Returning the Output Variables as arguments to the Scilab environment
    ReturnArguments(pvApiCtx);
	return 0;
}

}