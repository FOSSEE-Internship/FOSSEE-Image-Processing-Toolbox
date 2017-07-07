/********************************************************
Author: Suraj Prakash,Rohan Gurve
[return_image,x,y] = templateMatcher(InputArray image, InputArray template_image)
********************************************************/

#include <numeric>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

extern "C"{
  #include "api_scilab.h"
  #include "Scierror.h"
  #include "BOOL.h"
  #include <localization.h>
  #include <sciprint.h>
  #include "../common.h"
  
  int opencv_templateMatcher(char *fname, unsigned long fname_len){

    /// Error management variable
    SciErr sciErr;
    
    /// Variables
    int i, j;
    int error;
    int iRows = 0;
    int iCols = 0;
    int *piLen = NULL;
    int *piAddr = NULL;

     double minVal; double maxVal; Point minLoc; Point maxLoc;
     Point matchLoc;

    /// checking input argument
    CheckInputArgument(pvApiCtx, 2, 2);
    CheckOutputArgument(pvApiCtx, 3, 3);

    Mat image, template_image;
    retrieveImage(image, 1);
    retrieveImage(template_image, 2);

    
    
    if(image.channels()!=1){
      Scierror(999,"argument %d requires a single channel image\n",1);
      return 0;
    }
    
    if(template_image.channels()!=1){
      Scierror(999,"argument %d requires a single channel image\n",2);
      return 0;
    }
    
    if(template_image.rows > image.rows or template_image.cols > image.cols){
      Scierror(999, "%s: Dimensions of template image is greater then image\n");
      return 0;
    }

    //converting image to 8 bit
    image.convertTo(image,CV_8UC1);
     
    //converting template_image to type of image
    template_image.convertTo(template_image,CV_8UC1);
    
    
    /// generated image is (W - w + 1) * (H - h + 1) and it must be 32 bit floating number with single channel

    Mat new_image(image.rows, image.cols,CV_32FC1); 

     
        try
        {
            
             matchTemplate(image, template_image, new_image, CV_TM_SQDIFF);
             normalize( new_image, new_image, 0, 1, NORM_MINMAX, -1, Mat() );
             /// Localizing the best match with minMaxLoc
             minMaxLoc( new_image, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
             /// For SQDIFF the best matches are lower values
             matchLoc = minLoc;
      
        
        }
        catch(Exception& e)
        {
        const char* err = e.what();
        Scierror(999, "%s",err);
        return 0;
              
        }
         

		              
    
    
    string tempstring = type2str(new_image.type());
    char *checker;
    checker = (char *)malloc(tempstring.size() + 1);
    memcpy(checker, tempstring.c_str(), tempstring.size() + 1);
    returnImage(checker, new_image, 1);
    free(checker);

     error=createScalarDouble(pvApiCtx,nbInputArgument(pvApiCtx)+2,matchLoc.x);
     if(error!=0)
     {
       sciprint("error occurred");
       return 0;    
     }  
    error=createScalarDouble(pvApiCtx,nbInputArgument(pvApiCtx)+3,matchLoc.y);
     if(error!=0)
     {
       sciprint("error occurred");
       return 0;    
     }
    
    
    
    //Assigning the list as the Output Variable
    AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
    AssignOutputVariable(pvApiCtx, 2) = nbInputArgument(pvApiCtx) + 2;
    AssignOutputVariable(pvApiCtx, 3) = nbInputArgument(pvApiCtx) + 3;

    
    //Returning the Output Variables as arguments to the Scilab environment
    ReturnArguments(pvApiCtx);
    return 0;

  }
/* ==================================================================== */
}
