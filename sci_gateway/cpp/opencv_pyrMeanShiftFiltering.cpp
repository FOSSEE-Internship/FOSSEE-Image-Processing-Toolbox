/* ==================================================================== */
/* Author :Priyanka Hiranandani NIT Surat                               */
/* ==================================================================== */
/* Syntax :return_image=pyrMeanShiftFiltering(InputArray src, double sp, double sr); */
/* ==================================================================== */
#include <numeric>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <sciprint.h>
using namespace cv;
using namespace std;
extern "C"
{
  #include "api_scilab.h"
  #include "Scierror.h"
  #include "BOOL.h"
  #include <localization.h>
  #include "../common.h"

  int pyrMeanShiftFiltering(char *fname, unsigned long fname_len)
    {
    // Error management variable
        SciErr sciErr;
    //variable info
        int* piAddr2 = NULL;
        int* piAddr3 = NULL;
        int* piAddr4 = NULL;
        int nb;
        double sp;
        double sr;
        double maxValue=1;
        int error;
        TermCriteria termcrit=TermCriteria( TermCriteria::MAX_ITER+TermCriteria::EPS,5,1) ;
        nb=*getNbInputArgument(pvApiCtx);
        // checking input argument 
        CheckInputArgument(pvApiCtx,3,4);
        //checking output argument
        CheckOutputArgument(pvApiCtx, 1, 1);
        //for first argument 
        Mat img;
        retrieveImage(img,1);
        try
        {
            img.convertTo(img,CV_8UC3);
        }
        catch(cv::Exception&e)
        {
            const char* err=e.what();
            Scierror(999,e.what());
        }
    //second argument
        sciErr = getVarAddressFromPosition(pvApiCtx,2,&piAddr2);
        if (sciErr.iErr)
        {
        printError(&sciErr, 0);
        return 0;
        }
        error=getScalarDouble(pvApiCtx,piAddr2,&sp); 
        if(error!=0)
          {
                  sciprint("error in retrieving second argument");
        }  
  //third argument
        sciErr = getVarAddressFromPosition(pvApiCtx,3,&piAddr3);
        if (sciErr.iErr)
        {
        printError(&sciErr, 0);
        return 0;
        }
        error=getScalarDouble(pvApiCtx,piAddr3,&sr); 
        if(error!=0)
        {
          sciprint("error in retrieving second argument");
        }  
        if (nb>3)
        {
          sciErr = getVarAddressFromPosition(pvApiCtx,4,&piAddr4);
          if (sciErr.iErr)
          {
            printError(&sciErr, 0);
            return 0;
          }
          error=getScalarDouble(pvApiCtx,piAddr4,&maxValue); 
          if(error!=0)
          {
            sciprint("error in retrieving second argument");
          } 
        }
       //creating variable of type Mat
      Mat dst;
       //open cv function 
      try
      {
        pyrMeanShiftFiltering(img,dst, sp,sr,int(maxValue),termcrit);
      }
      catch(cv::Exception&e)
      {
        const char* err=e.what();
        Scierror(999,e.what());
      }
      string tempstring = type2str(dst.type());
      char* checker = (char *)malloc(tempstring.size() + 1);
      memcpy(checker, tempstring.c_str(), tempstring.size() + 1);
      returnImage(checker,dst,1);
      AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
      ReturnArguments(pvApiCtx);
      return 0;
   
    }
}
