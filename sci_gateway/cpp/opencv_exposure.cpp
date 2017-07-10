/********************************************************
Author: Manoj Sree Harsha 
********************************************************/
#include <numeric>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/stitching/detail/exposure_compensate.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/utility.hpp"
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

  int opencv_exposure(char *fname,unsigned long fname_len)
  {   

   // Error management variables
  	SciErr sciErr;
  	int intErr=0;

    //variable declarations
    int *piaddrvar1=NULL;
    int *piaddrvar2=NULL;
    double index=0;
    double choice=0;
    int i,num;

    //Checking number of input and output arguments (enviromnet variable, min arguments, max arguments)
  	CheckInputArgument(pvApiCtx, 4, 8);
    CheckOutputArgument(pvApiCtx, 1, 1);
   
    //number of input arguments
    num=*getNbInputArgument(pvApiCtx);

    //for argument #1
    sciErr = getVarAddressFromPosition(pvApiCtx, 1, &piaddrvar1);
    if (sciErr.iErr)
    {
      printError(&sciErr, 0);
      return 0;
    }
    if ( !isDoubleType(pvApiCtx, piaddrvar1))
    {
      Scierror(999, _("%s: Wrong type for input argument #%d: A scalar expected.\n"), fname, 1);
      return 0;
    }
    intErr = getScalarDouble(pvApiCtx, piaddrvar1,&choice);
    if (intErr)
    {
      return intErr;
    }

    //for argument #2
    sciErr = getVarAddressFromPosition(pvApiCtx, 2, &piaddrvar2);
    if (sciErr.iErr)
    {
      printError(&sciErr, 0);
      return 0;
    }
    if ( !isDoubleType(pvApiCtx, piaddrvar2))
    {
      Scierror(999, _("%s: Wrong type for input argument #%d: A scalar expected.\n"), fname,2);
      return 0;
    }
    intErr = getScalarDouble(pvApiCtx, piaddrvar2,&index);
    if (intErr)
    {
      return intErr;
    }

    //for images
    vector<UMat> images;
    vector<UMat> masks;
    vector<Point> points;
    for(i=3;i<=num;i++)
    {
      Mat image1;
      retrieveImage(image1, i);
      image1.convertTo(image1,CV_8U);
      Mat mask1(image1.size(), CV_8U);
      mask1(Rect(0, 0, mask1.cols, mask1.rows)).setTo(255);
      UMat img1 = image1.getUMat( ACCESS_READ );
      UMat m1 = mask1.getUMat( ACCESS_READ );
      images.push_back(img1);
      masks.push_back(m1);
      points.push_back(Point(0,0));
    }

    //Application Code
    try
    {
      if(choice==1)
      {
          Ptr<detail::ExposureCompensator> compensator = detail::ExposureCompensator::createDefault(detail::ExposureCompensator::GAIN);
          compensator->feed(points,images,masks);
          compensator->apply(index-1,points[index-1],images[index-1],masks[index-1]);
      }
      else if(choice==2)
      {
          Ptr<detail::ExposureCompensator> compensator = detail::ExposureCompensator::createDefault(detail::ExposureCompensator::GAIN_BLOCKS);
          compensator->feed(points,images,masks);
          compensator->apply(index-1,points[index-1],images[index-1],masks[index-1]);
      }
      else if(choice==3)
      {
          Ptr<detail::ExposureCompensator> compensator = detail::ExposureCompensator::createDefault(detail::ExposureCompensator::NO);
          compensator->feed(points,images,masks);
          compensator->apply(index-1,points[index-1],images[index-1],masks[index-1]);
      }
      else
      {
        Scierror(999,("%s: Wrong value for input argument #%d: A value b/w [1,3] is expected.\n"), fname,2);
        return 0;
      }

      //returning final image for which exposure is compensated
      Mat result=images[index-1].getMat(ACCESS_RW);
    	string tempstring = type2str(result.type());
      char *checker;
      checker = (char *)malloc(tempstring.size() + 1);
      memcpy(checker, tempstring.c_str(), tempstring.size() + 1);
      returnImage(checker,result,1);
      free(checker);
    }
	catch(cv::Exception& e)
    {
	 	const char* err=e.what();
	 	Scierror(999,("%s",err));
	}

    //Assigning output variables
    AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
    
    //Returning the Output Variables as arguments to the Scilab environment
    ReturnArguments(pvApiCtx);
  	return 0;

    }
}
