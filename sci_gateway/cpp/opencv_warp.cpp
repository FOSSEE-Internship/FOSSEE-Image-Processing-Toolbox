//Author-Avinash Reddy

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
#include <string>
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

    int opencv_warp(char *fname,unsigned long fname_len)
    {
	     SciErr sciErr;
       int intErr = 0;
       int *piAddr = NULL;
       int *piAddVar1 = NULL;
       int *piAddVar2 = NULL;
       int *piAddVar3 = NULL;
       int iLen        = 0;
       //variable info
       int iRows       = 0;
       int iCols       = 0;
       int piRows      = 0;
       int piCols      = 0;
       int *piLen      = NULL;
       char **pstData  = NULL;
       int **pstData1  = NULL;

       int num_images = 2;

       //Check input and output arguments
       CheckInputArgument(pvApiCtx,num_images,num_images);
       CheckOutputArgument(pvApiCtx,num_images,num_images);

       try
       {
         vector<Mat> images(num_images);
         for(int i=0;i<num_images;i++)
         {
           retrieveImage(images[i],1+i);
         }

         //Find features
         double fx = 1;
         double fy = 1;
         Ptr<FeaturesFinder> finder;
         finder = makePtr<OrbFeaturesFinder>();
         vector<ImageFeatures> features(num_images);
         for(int i = 0; i < num_images; i++)
         {
           images[i].convertTo(images[i],CV_8UC3);
           resize(images[i],images[i],Size(),fx,fy);
           (*finder)(images[i],features[i]);
           features[i].img_idx = i;
         }
         finder->collectGarbage();

         //Pair_wise matching
         float match_conf = 0.3f;
         vector<MatchesInfo> pairwise_matches;
         BestOf2NearestMatcher matcher(false,match_conf);
         matcher(features,pairwise_matches);
         matcher.collectGarbage();

         //Estimate camera parameters
         HomographyBasedEstimator estimator;
         vector<CameraParams> cameras;
         if(!estimator(features,pairwise_matches,cameras))
         {
           sciprint("Estimator Failed");
           return -1;
         }

         size_t cameras_size = cameras.size();

         for(size_t i=0;i<cameras_size;i++)
         {
           Mat R;
           cameras[i].R.convertTo(R, CV_32F);
           cameras[i].R = R;
         }

         //Refine camera parameters
         float conf_thresh = 1.f;
         Ptr<BundleAdjusterBase> adjuster;
         adjuster = makePtr<BundleAdjusterRay>();
         adjuster->setConfThresh(conf_thresh);
         if (!(*adjuster)(features, pairwise_matches, cameras))
         {
           sciprint("Adjuster Failed");
           return -1;
         }

         //Median focal
         vector<double> focals;
         for(size_t i=0;i<cameras_size;i++)
         {
           focals.push_back(cameras[i].focal);
         }
         sort(focals.begin(),focals.end());
         float median_focal;
         int focals_size = focals.size();
         if(focals_size%2)
         {
           median_focal = (float)(focals[focals_size/2]);
         }
         else
         {
           median_focal = (float)((focals[focals_size/2]+focals[focals_size/2 - 1]) * 0.5f);
         }

         //Wave correction
         vector<Mat> rmats;
         WaveCorrectKind wave_correct_type = WAVE_CORRECT_HORIZ;
         for (size_t i = 0; i < cameras.size(); ++i)
         {
           rmats.push_back(cameras[i].R.clone());
         }

         waveCorrect(rmats, wave_correct_type);
         for (size_t i = 0; i < cameras.size(); ++i)
         {
           cameras[i].R = rmats[i];
         }

         //Warping
         vector<UMat> masks(num_images);
         vector<Point> corners(num_images);
         vector<Size> sizes(num_images);
         vector<UMat> masks_warped(num_images);
         vector<UMat> images_warped(num_images);
         for(int i=0;i<num_images;i++)
         {
           masks[i].create(images[i].size(),CV_8U);
           masks[i].setTo(255);
         }

         Ptr<WarperCreator> warper_creator;
         warper_creator = makePtr<cv::SphericalWarper>();
         Ptr<RotationWarper> warper = warper_creator->create((float)(median_focal));
         for(int i=0;i<num_images;i++)
         {
           Mat_<float> K;
           cameras[i].K().convertTo(K, CV_32F);
           corners[i] = warper->warp(images[i], K, cameras[i].R, INTER_LINEAR, BORDER_REFLECT, images_warped[i]);
           sizes[i] = images_warped[i].size();
           warper->warp(masks[i], K, cameras[i].R, INTER_NEAREST, BORDER_CONSTANT, masks_warped[i]);
         }

         vector<UMat> images_warped_f(num_images);
         for (int i=0;i<num_images;i++)
         {
           Mat final_img;
           images_warped[i].convertTo(images_warped_f[i], CV_32F);
           masks_warped[i].copyTo(final_img);
           string tempstring = type2str(final_img.type());
           char *checker;
           checker = (char *)malloc(tempstring.size() + 1);
           memcpy(checker, tempstring.c_str(), tempstring.size() + 1);
           returnImage(checker,final_img,1 + i);
           AssignOutputVariable(pvApiCtx, 1 + i) = nbInputArgument(pvApiCtx) + 1 + i;
           free(checker);
         }

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
