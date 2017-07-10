#ifdef __cplusplus
extern "C" {
#endif
#include <mex.h> 
#include <sci_gateway.h>
#include <api_scilab.h>
#include <MALLOC.h>
static int direct_gateway(char *fname,void F(void)) { F();return 0;};
extern Gatefunc opencv_DCT;
extern Gatefunc opencv_FFT;
extern Gatefunc opencv_match;
extern Gatefunc opencv_cameraMatrix;
extern Gatefunc PCL_radsearch;
extern Gatefunc opencv_cornerEigenValsAndVecs;
extern Gatefunc opencv_cornerHarris;
extern Gatefunc opencv_cornerMinEigenVal;
extern Gatefunc opencv_imattributes;
extern Gatefunc opencv_imcrop;
extern Gatefunc opencv_imread;
extern Gatefunc opencv_bboxOverlapRatio;
extern Gatefunc opencv_convolver;
extern Gatefunc opencv_rotationVectorToMatrix;
extern Gatefunc opencv_affine2d;
extern Gatefunc opencv_radius;
extern Gatefunc opencv_panorama;
extern Gatefunc opencv_exposure;
extern Gatefunc opencv_rotestimate;
extern Gatefunc opencv_focals;
extern Gatefunc opencv_opticalflow;
static GenericTable Tab[]={
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_DCT,"raw_DCT"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_FFT,"raw_FFT"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_match,"raw_match"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_cameraMatrix,"raw_cameraMatrix"},
  {(Myinterfun)sci_gateway_without_putlhsvar,PCL_radsearch,"raw_radsearch"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_cornerEigenValsAndVecs,"raw_cornerEigenValsAndVecs"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_cornerHarris,"raw_cornerHarris"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_cornerMinEigenVal,"raw_cornerMinEigenVal"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_imattributes,"raw_imattributes"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_imcrop,"raw_imcrop"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_imread,"raw_imread"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_bboxOverlapRatio,"raw_bboxOverlapRatio"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_convolver,"raw_convolver"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_rotationVectorToMatrix,"raw_rotationVectorToMatrix"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_affine2d,"raw_affine2d"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_radius,"raw_radius"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_panorama,"raw_panorama"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_exposure,"raw_exposure"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_rotestimate,"raw_rotestimate"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_focals,"raw_focals"},
  {(Myinterfun)sci_gateway_without_putlhsvar,opencv_opticalflow,"raw_opticalflow"},
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
