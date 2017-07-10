/********************************************************
Author: Avinash Reddy , Manoj Sree Harsha , Ebey Abraham
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
using namespace std;
using namespace cv;
using namespace cv::detail;
extern "C"
{
  #include "api_scilab.h"
  #include "Scierror.h"
  #include "BOOL.h"
  #include <localization.h>
  #include "sciprint.h"
  #include "../common.h"

int opencv_panorama(char *fname,unsigned long fname_len)
{

      // Error management variables
		SciErr sciErr;

      //Checking number of input and output arguments (enviromnet variable, min arguments, max arguments)
		CheckInputArgument(pvApiCtx, 2, 6);
   		CheckOutputArgument(pvApiCtx, 1, 1);

      //variable declarations
   		int num,i;
    	double scale = 1;
      	bool do_wave_correct = true;
      	int expos_comp_type = ExposureCompensator::GAIN_BLOCKS;
      	string seam_find_type = "gc_color";
      	float blend_strength = 5;
      	float match_conf = 0.3f;
      	float conf_thresh = 1.f;
      	WaveCorrectKind wave_correct_type = WAVE_CORRECT_HORIZ;
      	int blend_type = Blender::MULTI_BAND;

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

        if (!estimator(features, pairwise_matches, cameras)){
          Scierror(999,("Homography estimation failed.\n"));
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
        if (!(*adjuster)(features, pairwise_matches, cameras)) {
          Scierror(1000,("Camera parameters adjusting failed.\n"));
          return 0;
        }


        vector<double> focals;
        for ( i = 0; i < cameras.size(); ++i)
        { 
          focals.push_back(cameras[i].focal);
        }

        sort(focals.begin(), focals.end());
        float warped_image_scale;
        if (focals.size() % 2 == 1){
          warped_image_scale = static_cast<float>(focals[focals.size() /2]);
        }
        else{
          warped_image_scale = static_cast<float>(focals[focals.size() /2 - 1] + focals[focals.size() / 2]) * 0.5f;
        }


        if (do_wave_correct)
        {
        vector<Mat> rmats;
        for (size_t i = 0; i < cameras.size(); ++i)
          rmats.push_back(cameras[i].R.clone());
        waveCorrect(rmats, wave_correct_type);
        for (size_t i = 0; i < cameras.size(); ++i)
          cameras[i].R = rmats[i];
        }

        //PHASE-3 COMPOSITION
        //Warping
        vector<Point> corners(num);
        vector<UMat> masks_warped(num);
        vector<UMat> images_warped(num);
        vector<Size> sizes(num);
        vector<UMat> masks(num);

        for ( i = 0; i < num; ++i)
        {
          masks[i].create(images[i].size(), CV_8U);
          masks[i].setTo(Scalar::all(255));
        }

        Ptr<WarperCreator> warper_creator;
        warper_creator = makePtr<cv::SphericalWarper>();
        if (!warper_creator){ 
          Scierror(1001,("Can't create the following warper")); return 0; 
        }

        Ptr<RotationWarper> warper = warper_creator->create(static_cast<float>(warped_image_scale * scale));
        for (i = 0; i < num; ++i)
        {
        Mat_<float> K;
        cameras[i].K().convertTo(K, CV_32F);
        float swa = (float)scale;
        K(0,0) *= swa; K(0,2) *= swa;
        K(1,1) *= swa; K(1,2) *= swa;
        corners[i] = warper->warp(images[i], K, cameras[i].R, INTER_LINEAR, BORDER_REFLECT, images_warped[i]);
        sizes[i] = images_warped[i].size();
        warper->warp(masks[i], K, cameras[i].R, INTER_NEAREST, BORDER_CONSTANT, masks_warped[i]);
        }

        vector<UMat> images_warped_f(num);
        for (i = 0; i < num; ++i)
        images_warped[i].convertTo(images_warped_f[i], CV_32F);

        //Exposure Compensation
        Ptr<ExposureCompensator> compensator = ExposureCompensator::createDefault(expos_comp_type);
        compensator->feed(corners, images_warped, masks_warped);
      
        //Seam Finding 
        Ptr<SeamFinder> seam_finder;
        seam_finder = makePtr<GraphCutSeamFinder>(GraphCutSeamFinderBase::COST_COLOR);

        seam_finder->find(images_warped_f, corners, masks_warped);

        
        images_warped.clear();
        images_warped_f.clear();
        masks.clear();

        //Blending 
        Ptr<Blender> blender = Blender::createDefault(blend_type, false);
        Size dst_sz = resultRoi(corners, sizes).size();
        float blend_width = sqrt(static_cast<float>(dst_sz.area())) *blend_strength / 100.f;

        if (blend_width < 1.f){
          blender = Blender::createDefault(Blender::NO, false);
        }

        else{
          MultiBandBlender* mb = dynamic_cast<MultiBandBlender*>(blender.get());
          mb->setNumBands(static_cast<int>(ceil(log(blend_width)/log(2.)) - 1.));
        }

        blender->prepare(corners, sizes);

        Mat img_warped, img_warped_s,full_img;
        Mat dilated_mask, seam_mask, mask, mask_warped;

        for (int img_idx = 0; img_idx < num; ++img_idx)
        {
          //  Read image and resize it if necessary
          full_img = imagescopy[img_idx];
          if (abs(scale - 1) > 1e-1)
          resize(full_img, img, Size(), scale, scale);
          else
          img = full_img;
          full_img.release();
          Size img_size = img.size();
          Mat K;
          cameras[img_idx].K().convertTo(K, CV_32F);
          //  Warp the current image

          warper->warp(img, K, cameras[img_idx].R, INTER_LINEAR, BORDER_REFLECT,img_warped);

          // Warp the current image mask
          mask.create(img_size, CV_8U);
          mask.setTo(Scalar::all(255));
          warper->warp(mask, K, cameras[img_idx].R, INTER_NEAREST,BORDER_CONSTANT, mask_warped);
          //  Compensate exposure error step
          compensator->apply(img_idx, corners[img_idx], img_warped, mask_warped);
          img_warped.convertTo(img_warped_s, CV_16S);
          img_warped.release();
          img.release();
          mask.release();
          dilate(masks_warped[img_idx], dilated_mask, Mat());
          resize(dilated_mask, seam_mask, mask_warped.size());
          mask_warped = seam_mask & mask_warped;
          //  Blending images step
          blender->feed(img_warped_s, mask_warped, corners[img_idx]);
        }

        Mat result, result_mask;
        blender->blend(result, result_mask);


        //returning panoramic Image
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
