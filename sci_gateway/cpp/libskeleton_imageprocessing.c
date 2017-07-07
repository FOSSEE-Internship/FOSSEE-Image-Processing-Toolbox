#ifdef __cplusplus
extern "C" {
#endif
#include <mex.h> 
#include <sci_gateway.h>
#include <api_scilab.h>
#include <MALLOC.h>
static int direct_gateway(char *fname,void F(void)) { F();return 0;};
extern Gatefunc opencv_dctmtx;
extern Gatefunc opencv_imgaussfilt3;
extern Gatefunc opencv_imguidedfilter;
extern Gatefunc opencv_imhmax;
extern Gatefunc opencv_impixel;
extern Gatefunc opencv_imread;
extern Gatefunc opencv_lab2uint16;
extern Gatefunc opencv_rgb2ntsc;
extern Gatefunc opencv_imhistmatch;
extern Gatefunc opencv_imrotate;
extern Gatefunc opencv_mean1;
extern Gatefunc opencv_imageSet;
extern Gatefunc opencv_bagOfFeatures;
extern Gatefunc opencv_trainANNMLPClassifier;
extern Gatefunc opencv_predictANN;
extern Gatefunc opencv_getParamsANN;
extern Gatefunc opencv_getParamsdtree;
extern Gatefunc opencv_getParamsBoost;
extern Gatefunc opencv_predictdtree;
extern Gatefunc opencv_predictBoost;
extern Gatefunc opencv_traindtreeClassifier;
extern Gatefunc opencv_trainBoostClassifier;
extern Gatefunc opencv_extractHOGFeatures;
extern Gatefunc opencv_extractLBPFeatures;
extern Gatefunc opencv_detectSURFFeatures;
extern Gatefunc opencv_detectBRISKFeatures;
extern Gatefunc opencv_extractFeatures;
extern Gatefunc opencv_detectAgastFeatures;
extern Gatefunc opencv_detectAkazeFeatures;
extern Gatefunc opencv_matchFeatures;
extern Gatefunc opencv_drawMatch;
extern Gatefunc opencv_drawKeypoints;
extern Gatefunc opencv_distortPoints;
extern Gatefunc opencv_rectifyStereoImages;
extern Gatefunc opencv_projectPoints;
extern Gatefunc opencv_undistortImage;
extern Gatefunc opencv_estnewcam;
extern Gatefunc opencv_genCheckerboardPoints;
extern Gatefunc opencv_detectCheckerboardCorner;
extern Gatefunc opencv_Calibrate;
extern Gatefunc opencv_imsharpen;
extern Gatefunc opencv_line;
extern Gatefunc opencv_threshold;
extern Gatefunc opencv_sobel;
extern Gatefunc opencv_puttext;
extern Gatefunc opencv_trainLRClassifier;
extern Gatefunc opencv_imfuse;
extern Gatefunc opencv_isEpipoleInImage;
extern Gatefunc opencv_epipolarlines;
extern Gatefunc opencv_GeometricShearer;
extern Gatefunc PCL_pcread;
extern Gatefunc pcl_pointCloud;
extern Gatefunc opencv_multithresh;
extern Gatefunc opencv_disparity;
static GenericTable Tab[]={
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_dctmtx,"raw_dctmtx"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_imgaussfilt3,"raw_imgaussfilt3"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_imguidedfilter,"raw_imguidedfilter"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_imhmax,"raw_imhmax"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_impixel,"raw_impixel"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_imread,"raw_imread"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_lab2uint16,"raw_lab2uint16"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_rgb2ntsc,"raw_rgb2ntsc"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_imhistmatch,"raw_imhistmatch"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_imrotate,"raw_imrotate"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_mean1,"raw_mean1"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_imageSet,"raw_imageSet"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_bagOfFeatures,"raw_bagOfFeatures"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_trainANNMLPClassifier,"raw_trainANNMLPClassifier"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_predictANN,"raw_predictANN"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_getParamsANN,"raw_getParamsANN"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_getParamsdtree,"raw_getParamsdtree"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_getParamsBoost,"raw_getParamsBoost"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_predictdtree,"raw_predictdtree"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_predictBoost,"raw_predictBoost"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_traindtreeClassifier,"raw_traindtreeClassifier"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_trainBoostClassifier,"raw_trainBoostClassifier"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_extractHOGFeatures,"raw_extractHOGFeatures"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_extractLBPFeatures,"raw_extractLBPFeatures"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_detectSURFFeatures,"ocv_detectSURFFeatures"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_detectBRISKFeatures,"ocv_detectBRISKFeatures"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_extractFeatures,"raw_extractFeatures"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_detectAgastFeatures,"raw_detectAgastFeatures"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_detectAkazeFeatures,"ocv_detectAkazeFeatures"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_matchFeatures,"raw_matchFeatures"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_drawMatch,"raw_drawMatch"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_drawKeypoints,"raw_drawKeypoints"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_distortPoints,"raw_distortPoints"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_rectifyStereoImages,"raw_rectifyStereoImages"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_projectPoints,"raw_projectPoints"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_undistortImage,"raw_undistortImage"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_estnewcam,"raw_estnewcam"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_genCheckerboardPoints,"raw_genCheckerboardPoints"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_detectCheckerboardCorner,"raw_detectCheckerboardCorner"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_Calibrate,"raw_Calibrate"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_imsharpen,"raw_imsharpen"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_line,"raw_line"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_threshold,"raw_threshold"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_sobel,"raw_sobel"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_puttext,"raw_puttext"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_trainLRClassifier,"raw_trainLRClassifier"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_imfuse,"raw_imfuse"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_isEpipoleInImage,"raw_isEpipoleInImage"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_epipolarlines,"raw_epipolarlines"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_GeometricShearer,"raw_GeometricShearer"},
  {(Myinterfun)sci_gateway_without_putlhsvar,PCL_pcread,"raw_pcread"},
  {(Myinterfun)sci_gateway_without_putlhsvar,pcl_pointCloud,"raw_pointCloud"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_multithresh,"raw_multithresh"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_disparity,"raw_disparity"},
};
 
int C2F(libskeleton_imageprocessing)()
{
  Rhs = Max(0, Rhs);
  if (*(Tab[Fin-1].f) != NULL) 
  {
     if(pvApiCtx == NULL)
     {
       pvApiCtx = (StrCtx*)MALLOC(sizeof(StrCtx));
     }
     pvApiCtx->pstName = (char*)Tab[Fin-1].name;
    (*(Tab[Fin-1].f))(Tab[Fin-1].name,Tab[Fin-1].F);
  }
  return 0;
}
#ifdef __cplusplus
}
#endif
