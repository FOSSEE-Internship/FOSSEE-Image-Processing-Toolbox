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

    int opencv_focals(char *fname,unsigned long fname_len)
    {
       // Error management variables
	     SciErr sciErr;
       int intErr = 0;

       //variable declarations
       int i,k,j,num;
       double scale = 1;
       double *result=NULL;
       float conf_thresh = 1.f;
       float match_conf = 0.3f;

       //Checking number of input and output arguments (enviromnet variable, min arguments, max arguments)
       CheckInputArgument(pvApiCtx,2,6);
       CheckOutputArgument(pvApiCtx,1,1);

       //number of input arguments
       num=*getNbInputArgument(pvApiCtx);

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
           Mat image1,img;
           for(i=0;i<num;i++)
           {

            retrieveImage(image1, i+1);
            image1.convertTo(image1,CV_8U);
            resize(image1, img, Size(), scale, scale);
            images[i]=img.clone();
            imagescopy[i]=img.clone();
            full_img_sizes.push_back(image1.size());
            (*finder)(img, features[i]);
            features[i].img_idx = i;
           }
           finder->collectGarbage();
           image1.release();
           img.release();
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
           result=(double*)malloc(sizeof(double)*1*num);
           for(i=0;i<num;i++)
           {
               result[i]=cameras[i].focal;
           }
           //creating the output matrix
           sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 1,1,num,result); 
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
     	 Scierror(999,("%s",err));
       }
       return 0;
    }
}
