/********************************************************
Author: Diwakar
return_image = IFFT(source_image)
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
  
  int opencv_IFFT(char *fname, unsigned long fname_len)
  {

    /// Error management variable
    SciErr sciErr;  
    /// Variables
    int i, j;
    /// checking input argument
    CheckInputArgument(pvApiCtx, 1, 1);
    CheckOutputArgument(pvApiCtx, 1, 1);
    Mat img;
    //Retriving image
    retrieveImage(img, 1);

    if(img.channels()>2)
    {
        Scierror(999, "Input should be single or double channel\n"); 
        return 0; 
    }

    Mat planes[] = {Mat_<float>(img), Mat::zeros(img.size(), CV_32F)};
    Mat complexI;

    merge(planes, 2, complexI);
    
    dft(complexI, complexI);
    Mat invDFT, invDFTcvt;
    idft(complexI, invDFT, DFT_SCALE | DFT_REAL_OUTPUT );
    invDFT.convertTo(invDFTcvt, CV_8U); 
  
    string tempstring = type2str(invDFTcvt.type());
    char *checker;
    checker = (char *)malloc(tempstring.size() + 1);
    memcpy(checker, tempstring.c_str(), tempstring.size() + 1);
    returnImage(checker,invDFTcvt, 1);
    free(checker);

    //Assigning the list as the Output Variable
    AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;

    //Returning the Output Variables as arguments to the Scilab environment
    ReturnArguments(pvApiCtx);
    return 0;

  }
/* ==================================================================== */
}

