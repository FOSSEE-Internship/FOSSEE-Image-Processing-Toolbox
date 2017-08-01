/***************************************************
Author : Rohan Gurve
****************************************************
   dst = convert(src,rtype,alpha,beta)
***************************************************/

#include <numeric>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
using namespace cv;
using namespace std;
extern "C"
{
  #include "api_scilab.h"
  #include "Scierror.h"
  #include "BOOL.h"
  #include <localization.h>
  #include <sciprint.h>
  #include "../common.h"
  
  int opencv_convert(char *fname, unsigned long fname_len)
  {

    SciErr sciErr;
    int intErr=0;
    int iRows=0,iCols=0;
    int *piAddr2  = NULL;
    int *piAddr3  = NULL;
    int *piAddr4  = NULL;
    int *piLen = NULL;
    char **rtype = NULL;
    double alpha,beta;
    int i;
    int type;
    
    //checking input argument
    CheckInputArgument(pvApiCtx, 4, 4);
    CheckOutputArgument(pvApiCtx, 1, 1) ;

    Mat src;
    retrieveImage(src,1);

    
    //for type
    sciErr = getVarAddressFromPosition(pvApiCtx,2, &piAddr2);
    if (sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }
    sciErr = getMatrixOfString(pvApiCtx, piAddr2, &iRows, &iCols, NULL, NULL);
    if(sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }
    piLen = (int*)malloc(sizeof(int) * iRows * iCols);
    //second call to retrieve length of each string
    sciErr = getMatrixOfString(pvApiCtx, piAddr2, &iRows, &iCols, piLen, NULL);
    if(sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }
    rtype = (char**)malloc(sizeof(char*) * iRows * iCols);
    for(i = 0 ; i < iRows * iCols ; i++)
    {
        rtype[i] = (char*)malloc(sizeof(char) * (piLen[i] + 1));//+ 1 for null termination
    }
    //third call to retrieve data
    sciErr = getMatrixOfString(pvApiCtx, piAddr2, &iRows, &iCols, piLen, rtype);
    if(sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }
    
    
    //for value of alpha
    sciErr = getVarAddressFromPosition(pvApiCtx,3,&piAddr3);
    if (sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }
    intErr = getScalarDouble(pvApiCtx, piAddr3, &alpha);
    if(intErr)
        return intErr;
  
    //for value of beta
    sciErr = getVarAddressFromPosition(pvApiCtx,4,&piAddr4);
    if (sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }
    
    intErr = getScalarDouble(pvApiCtx, piAddr4 ,&beta);
    if(intErr)
        return intErr;

    
    if(strcmp(rtype[0], "CV_8U") == 0)
        type = CV_8U;
    else if(strcmp(rtype[0], "CV_8S") == 0)
        type = CV_8S;
    else if(strcmp(rtype[0], "CV_16U") == 0)
        type = CV_16U;
    else if(strcmp(rtype[0], "CV_16S") == 0)
        type = CV_16S;
    else if(strcmp(rtype[0], "CV_32S") == 0)
        type = CV_32S;
    else if(strcmp(rtype[0], "CV_32F") == 0)
        type = CV_32F;    
    else if(strcmp(rtype[0], "CV_64F") == 0)
        type = CV_64F;
    //8bit unsigned int
    else if(strcmp(rtype[0], "CV_8UC1") == 0)
        type = CV_8UC1;
    else if(strcmp(rtype[0], "CV_8UC2") == 0)
        type = CV_8UC2;
    else if(strcmp(rtype[0], "CV_8UC3") == 0)
        type = CV_8UC3;
    else if(strcmp(rtype[0], "CV_8UC4") == 0)
        type = CV_8UC4;
    //8 bit signed int
    else if(strcmp(rtype[0], "CV_8SC1") == 0)
        type = CV_8SC1;
    else if(strcmp(rtype[0], "CV_8SC2") == 0)
        type = CV_8SC2;
    else if(strcmp(rtype[0], "CV_8SC3") == 0)
        type = CV_8SC3;
    else if(strcmp(rtype[0], "CV_8SC4") == 0)
        type = CV_8SC4;
    //16 bit unsigned int
    else if(strcmp(rtype[0], "CV_16UC1") == 0)
        type = CV_16UC1;
    else if(strcmp(rtype[0], "CV_16UC2") == 0)
        type = CV_16UC2;
    else if(strcmp(rtype[0], "CV_16UC3") == 0)
        type = CV_16UC3;
    else if(strcmp(rtype[0], "CV_16UC4") == 0)
        type = CV_16UC4;
    
    //16 bit signed int    
    else if(strcmp(rtype[0], "CV_16SC1") == 0)
        type = CV_16SC1;
    else if(strcmp(rtype[0], "CV_16SC2") == 0)
        type = CV_16SC2;
    else if(strcmp(rtype[0], "CV_16SC3") == 0)
        type = CV_16SC3;
    else if(strcmp(rtype[0], "CV_16SC4") == 0)
        type = CV_16SC4;
    
   //32 bit signed int    
    else if(strcmp(rtype[0], "CV_32SC1") == 0)
        type = CV_32SC1;
    else if(strcmp(rtype[0], "CV_32SC2") == 0)
        type = CV_32SC2;
    else if(strcmp(rtype[0], "CV_32SC3") == 0)
        type = CV_32SC3;
    else if(strcmp(rtype[0], "CV_32SC4") == 0)
        type = CV_32SC4;
 
    //64 bit float    
   else if(strcmp(rtype[0], "CV_64FC1") == 0)
        type = CV_64FC1;
    else if(strcmp(rtype[0], "CV_64FC2") == 0)
        type = CV_64FC2;
    else if(strcmp(rtype[0], "CV_64FC3") == 0)
        type = CV_64FC3;
    else if(strcmp(rtype[0], "CV_64FC4") == 0)
        type = CV_64FC4;
    
    else
      {
      Scierror(999, "%s: Invalid image type\n");
      return 0;
      }   

    try
    {
        src.convertTo(src,type,alpha,beta);
    }
    catch(Exception& e)
    {
        const char* err = e.what();
        Scierror(999, "%s",err);
        return 0;  
    }

    
    string tempstring = type2str(src.type());
    char *checker;
    checker = (char *)malloc(tempstring.size() + 1);
    memcpy(checker, tempstring.c_str(), tempstring.size() + 1);
    returnImage(checker,src,1);
    free(checker);

    //Assigning the list as the Output Variable
    AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
    //Returning the Output Variables as arguments to the Scilab environment
    ReturnArguments(pvApiCtx);
    return 0;

  }
/* ==================================================================== */
}
