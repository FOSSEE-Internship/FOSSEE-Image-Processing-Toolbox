/********************************************************
Author: Manoj Sree Harsha
********************************************************/
#include <numeric>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/stitching/detail/blenders.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/stitching/detail/blenders.hpp>
#include <opencv2/stitching/detail/camera.hpp>
#include <opencv2/stitching/detail/exposure_compensate.hpp>
#include <opencv2/stitching/detail/matchers.hpp>
#include <opencv2/stitching/detail/motion_estimators.hpp>
#include <opencv2/stitching/detail/seam_finders.hpp>
#include <opencv2/stitching/detail/util.hpp>
#include <opencv2/stitching/detail/warpers.hpp>
#include <opencv2/stitching/warpers.hpp>
#include <iostream>
using namespace cv;
using namespace cv::detail;
using namespace std;
extern "C"
{
    #include "api_scilab.h"
    #include "Scierror.h"
    #include "BOOL.h"
    #include <localization.h>
    #include "sciprint.h"
    #include "../common.h"

    int opencv_rotestimate(char *fname,unsigned long fname_len)
    {

       // Error management variables
       SciErr sciErr;
       int intErr = 0;

       //variable declarations
       int i,k,j,num;
       double scale = 1;
       double index=0;
       int *piaddrvar=NULL;
       double *result=NULL;
       float conf_thresh = 1.f;
       float match_conf = 0.3f;


       //Checking number of input and output arguments (enviromnet variable, min arguments, max arguments)
       CheckInputArgument(pvApiCtx,3,7);
       CheckOutputArgument(pvApiCtx,1,1);

       //number of input arguments
       num = *getNbInputArgument(pvApiCtx);
       num = num-1;

       //argument #1
       sciErr = getVarAddressFromPosition(pvApiCtx,1,&piaddrvar);
       if (sciErr.iErr)
       {
           printError(&sciErr, 0);
           return 0;
       }
       if ( !isDoubleType(pvApiCtx, piaddrvar))
       {
           Scierror(999, _("%s: Wrong type for input argument #%d: A scalar expected.\n"), fname,1);
           return 0;
       }

       intErr = getScalarDouble(pvApiCtx, piaddrvar,&index);
       if (intErr)
       {
           return intErr;
       }

       if(index<1 || index>num){
           Scierror(999, _("%s: Invalid input argument #%d. The value is expected to be between 1 and %d\n"), fname,1,num);
           return 0;
       }

       //Application Code
       try
       {
          //PHASE-1 REGISTRATION
          //Feature matching
          vector<Size> full_img_sizes(num);
          vector<Mat> images(num);
          vector<Mat> imagescopy(num);
          Ptr<FeaturesFinder> finder;
          finder = makePtr<SurfFeaturesFinder>();
          vector<ImageFeatures> features(num);
          for(i=1;i<=num;i++)
          { 
             Mat image1,img;
             retrieveImage(image1, i+1);
             image1.convertTo(image1,CV_8U);
             resize(image1, img, Size(), scale, scale);
             images[i-1]=img.clone();
             imagescopy[i-1]=img.clone();

             full_img_sizes.push_back(image1.size());
             (*finder)(img, features[i-1]);
             features[i-1].img_idx = i-1;
             img.release();
          }
          finder->collectGarbage();
          vector<MatchesInfo> pairwise_matches;
          BestOf2NearestMatcher matcher(false, match_conf);
          matcher(features, pairwise_matches);
          matcher.collectGarbage();
          vector<int> indices = leaveBiggestComponent(features, pairwise_matches, conf_thresh);
          vector<Mat> img_subset;
          vector<Size> full_img_sizes_subset;
          for (i = 0; i < indices.size(); ++i)
          {
             img_subset.push_back(images[indices[i]]);
             full_img_sizes_subset.push_back(full_img_sizes[indices[i]]);
          }
          images = img_subset;
          full_img_sizes = full_img_sizes_subset;

          //PHASE-2 CALIBRARION
          //Estimating Camera Parameters
          HomographyBasedEstimator estimator;
          vector<CameraParams> cameras;
          if (!estimator(features, pairwise_matches, cameras))
          {
             sciprint("Homography estimation failed.\n");
             return 0;
          }
          for (i = 0; i < cameras.size(); ++i)
          {
             Mat R;
             cameras[i].R.convertTo(R, CV_32F);
             cameras[i].R = R;
          }
          Ptr<BundleAdjusterBase> adjuster;
          adjuster = makePtr<BundleAdjusterRay>();
          adjuster->setConfThresh(conf_thresh);
          if (!(*adjuster)(features, pairwise_matches, cameras)) 
          {
             sciprint("Camera parameters adjusting failed.\n");
             return 0;
          }
          result=(double*)malloc(sizeof(double)*3*3);
          for(i=0;i<3;i++)
          {
             for(j=0;j<3;j++)
             {
               result[i+3*j]=cameras[index-1].K().at<double>(i,j);
             }
          }

          //creating the output matrix
          sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 1,3,3,result); 
         	if (sciErr.iErr)
          {
         	    printError(&sciErr, 0);
         	    return sciErr.iErr;
         	}
          //Assigning output variables
          AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;

          //Returning the Output Variables as arguments to the Scilab environment
          ReturnArguments(pvApiCtx);

       }
       catch(cv::Exception& e)
       {
         const char* err=e.what();
     	 	 sciprint("%s",err);
       }
       return 0;
    }
}
