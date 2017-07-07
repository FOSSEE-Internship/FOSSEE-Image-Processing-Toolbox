/********************************************************
Function   :applycform
Syntax     :B=applycform(image,string)
*string    : 'xyz2lab'   'lab2xyz'   'srgb2xyz' 'xyz2uvl'  
             'xyz2srgb'  'srgb2lab'  'lab2srgb' 'uvl2xyz'  
Author     : Tess  Zacharias,Gursimar Singh 
********************************************************/

#include <numeric>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
#include "string.h"
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
 
  int opencv_applycform(char *fname, unsigned long fname_len)
  {
     // Error management variable
    SciErr sciErr;
    //variable info
    int* piAddr2=NULL;
    Mat img;
    char* pstData = NULL;
    int iRet    = 0;
    CheckInputArgument(pvApiCtx,2,2);
    CheckOutputArgument(pvApiCtx,1,1);
    retrieveImage(img,1);
    try
    {
        img.convertTo(img,CV_8U);
    }
    catch(cv::Exception&e)
    {
        const char* err=e.what();
        Scierror(999,e.what());
    }
    
    sciErr = getVarAddressFromPosition(pvApiCtx, 2,&piAddr2);

  	if(sciErr.iErr)
  	{
  		printError(&sciErr, 0);
  		return 0;
  	}

  	if(isStringType(pvApiCtx, piAddr2))
  	{
  		if(isScalar(pvApiCtx, piAddr2))
  		{
  			
          iRet = getAllocatedSingleString(pvApiCtx, piAddr2, &pstData);
      }
    }
       
    
    Mat image; 
    //XYZ2LAb
    if(strcasecmp(pstData,"xyz2lab")==0)
     {
        cvtColor(img,image,cv::COLOR_XYZ2RGB);
        cvtColor(image,image,cv::COLOR_RGB2Lab);   
     }
     //UVL2XYZ
     else if(strcasecmp(pstData,"uvl2xyz")==0)
     {
    
       cvtColor(img,image,cv::COLOR_YUV2RGB);
       cvtColor(image,image,cv::COLOR_RGB2XYZ);   

     }
     //XYZ2uvl
     else if(strcasecmp(pstData,"xyz2uvl")==0)
     {
        
      cvtColor(img,image,cv::COLOR_XYZ2RGB);
      cvtColor(image,image,cv::COLOR_RGB2YUV);   

     }
     //Lab2XYZ
    else if(strcasecmp(pstData,"lab2xyz")==0)
     {
       cvtColor(img,image,cv::COLOR_Lab2RGB);
       cvtColor(image,image,cv::COLOR_RGB2XYZ);                        
     } 
     //srgb2XYZ
     else if(strcasecmp(pstData,"srgb2xyz")==0)
     {
        cvtColor(img,image,cv::COLOR_RGB2XYZ);   
     }
     //XYZ2SRGB 
     else if(strcasecmp(pstData,"xyz2srgb")==0)
     {
         cvtColor(img,image,cv::COLOR_XYZ2RGB);
     }
     //SRGB2Lab 
     else if(strcasecmp(pstData,"srgb2lab")==0)
     {
         cvtColor(img,image,cv::COLOR_RGB2Lab);
     }
     //Lab2srgb
     else if(strcasecmp(pstData,"lab2srgb")==0)
     {
         cvtColor(img,image,cv::COLOR_Lab2RGB);
     }
    else
     {
        sciprint("Expected input argument 'xyz2lab'   'lab2xyz'   'srgb2xyz' 'xyz2srgb'  'srgb2lab'  'lab2srgb' 'xyz2uvl' 'uvl2xyz' ");
        return 0;
     }  

    
    int temp = nbInputArgument(pvApiCtx) + 1;
    string tempstring = type2str(image.type());
    char *checker;
    checker = (char *)malloc(tempstring.size() + 1);
    memcpy(checker, tempstring.c_str(), tempstring.size() + 1);
    returnImage(checker, image, 1);
    free(checker);
    
    AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
    ReturnArguments(pvApiCtx);            
    return 0;       
  }
}
