/********************************************************
Author: Suraj Prakash 
return_image = IDCT(source_image)
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
  #include "../common.h"
  
  int opencv_IDCT(char *fname, unsigned long fname_len){

    /// Error management variable
    SciErr sciErr;
    
    /// Variables
    int i, j;

    int flags = 0;

    /// checking input argument
    CheckInputArgument(pvApiCtx, 1, 1);
    CheckOutputArgument(pvApiCtx, 1, 1);


    Mat image;
    retrieveImage(image, 1);
    
    if(image.channels() != 1){
    	Scierror(999, "%s: Image should be single channel\n");
    	return 0;
    }
    /// even sized rows and columns required
    if(image.rows%2 != 0 or image.cols%2 != 0){
      Scierror(999, "%s: Either of rows size or column size is odd\n");
      return 0;
    }
    
    /// transformed_idct stores the transformed image
    Mat transformed_idct(image.rows, image.cols, image.type());
    ///calling idct///
    idct(image, transformed_idct, flags);
    //output//
    string tempstring = type2str(transformed_idct.type());
    char *checker;
    checker = (char *)malloc(tempstring.size() + 1);
    memcpy(checker, tempstring.c_str(), tempstring.size() + 1);
    returnImage(checker, transformed_idct, 1);
    free(checker);

    //Assigning the list as the Output Variable
    AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;

    //Returning the Output Variables as arguments to the Scilab environment
    ReturnArguments(pvApiCtx);
    return 0;

  }
/* ==================================================================== */
}

