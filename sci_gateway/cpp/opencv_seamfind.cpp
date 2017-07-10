//Author- M Avinash Reddy

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

int opencv_seamfind(char *fname,unsigned long fname_len){

		  SciErr sciErr;
		  CheckInputArgument(pvApiCtx, 3, 7);
   		CheckOutputArgument(pvApiCtx, 1, 1);
      int *piaddrvar=NULL;
      double choice=0;
   		int num,i,j,intErr;
      double scale = 1;
      bool do_wave_correct = true;
      int expos_comp_type = ExposureCompensator::GAIN_BLOCKS;
      string seam_find_type = "gc_color";
      float blend_strength = 5;
      float match_conf = 0.3f;
      float conf_thresh = 1.f;
      WaveCorrectKind wave_correct_type = WAVE_CORRECT_HORIZ;



      num=*getNbInputArgument(pvApiCtx);
      num=num-1;
      sciErr = getVarAddressFromPosition(pvApiCtx, 1, &piaddrvar);
      if (sciErr.iErr)
      {
          printError(&sciErr, 0);
          return 0;
      }
      if ( !isDoubleType(pvApiCtx, piaddrvar))
      {
          Scierror(999, _("%s: Wrong type for input argument #%d: A scalar expected.\n"), fname, 1);
          return 0;
      }

      intErr = getScalarDouble(pvApiCtx, piaddrvar,&choice);
      if (intErr)
      {

          return intErr;
      }
      if(choice<1 || choice>num){
            Scierror(999, _("%s:Invalid input argument #%d: Choice has to be between 1 and %d\n"), fname,1,num);
            return 0;
      }
      try{
      vector<Size> full_img_sizes(num);
   		vector<Mat> images(num);
      vector<Mat> imagescopy(num);

      Ptr<FeaturesFinder> finder;
      finder = makePtr<SurfFeaturesFinder>();

      vector<ImageFeatures> features(num);
      Mat image1,img;
   		for(i=0;i<num;i++){

			  retrieveImage(image1, i+2);

        image1.convertTo(image1,CV_8U);
        //sciprint("%d",image1.type());
        resize(image1, img, Size(), scale, scale);
			  images[i]=img.clone();
        imagescopy[i]=img.clone();

        full_img_sizes.push_back(image1.size());
        (*finder)(img, features[i]);
        features[i].img_idx = i;
        //cout << "Features in image #" << i+1 << " are : " <<features[i].keypoints.size() << endl;
    	}


      //imwrite("/home/avinashr175/Desktop/output.jpg",images[0]);
      finder->collectGarbage();
      image1.release();
      img.release();


      vector<MatchesInfo> pairwise_matches;
      BestOf2NearestMatcher matcher(false, match_conf);
      matcher(features, pairwise_matches);
      matcher.collectGarbage();


      vector<int> indices = leaveBiggestComponent(features, pairwise_matches, conf_thresh);
      //cout<<indices.size()<<endl;
      vector<Mat> img_subset;
      //vector<String> img_names_subset;
      vector<Size> full_img_sizes_subset;
      for (i = 0; i < indices.size(); ++i)
      {
      //img_names_subset.push_back(img_names[indices[i]]);
      img_subset.push_back(images[indices[i]]);
      full_img_sizes_subset.push_back(full_img_sizes[indices[i]]);
      }

      images = img_subset;
      full_img_sizes = full_img_sizes_subset;


      HomographyBasedEstimator estimator;
      vector<CameraParams> cameras;

      if (!estimator(features, pairwise_matches, cameras)){
        sciprint("Homography estimation failed.\n");
        return 0;
      }
      for (i = 0; i < cameras.size(); ++i)
      {
      Mat R;
      cameras[i].R.convertTo(R, CV_32F);
      cameras[i].R = R;
      //cout << "Initial intrinsic #" << indices[i]+1 << ":\n" <<cameras[i].K() << endl;
      }

      Ptr<BundleAdjusterBase> adjuster;
      adjuster = makePtr<BundleAdjusterRay>();

      adjuster->setConfThresh(conf_thresh);
      if (!(*adjuster)(features, pairwise_matches, cameras)) {
        sciprint("Camera parameters adjusting failed.\n");
        return 0;
      }


      vector<double> focals;
      for ( i = 0; i < cameras.size(); ++i)
      {
        //cout << "Camera #" << indices[i]+1 << ":\n" << cameras[i].K()<< endl;
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
      if (!warper_creator){ sciprint("Can't create the following warper"); return 0; }

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


      Ptr<ExposureCompensator> compensator = ExposureCompensator::createDefault(expos_comp_type);
      compensator->feed(corners, images_warped, masks_warped);
      //cout<<corners<<endl;

      Ptr<SeamFinder> seam_finder;
      seam_finder = makePtr<GraphCutSeamFinder>(GraphCutSeamFinderBase::COST_COLOR);

      seam_finder->find(images_warped_f, corners, masks_warped);

			Mat result=masks_warped[choice-1].getMat('ACCESS_RW');


      for(i=0;i<result.rows;i++){
        for(j=0;j<result.cols;j++){

          if(result.at<uchar>(i,j)==0){

            images[choice-1].at<Vec3b>(i,j)[0]=0;
            images[choice-1].at<Vec3b>(i,j)[1]=0;
            images[choice-1].at<Vec3b>(i,j)[2]=0;


          }

         

          

        }
      }


        	string tempstring = type2str(images[choice-1].type());
        	char *checker;
        	checker = (char *)malloc(tempstring.size() + 1);
        	memcpy(checker, tempstring.c_str(), tempstring.size() + 1);
        	returnImage(checker,images[choice-1],1);
        	free(checker);
        }

	catch(cv::Exception& e){
	 	const char* err=e.what();
	 	sciprint("%s",err);
	 }

    AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;;
    //Returning the Output Variables as arguments to the Scilab environment
    ReturnArguments(pvApiCtx);
	return 0;

    }
}
