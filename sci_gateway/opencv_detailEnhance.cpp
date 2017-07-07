/********************************************************
    Author: Rohan Gurve
********************************************************/

#include <numeric>
#include "opencv2/opencv.hpp"
#include "opencv2/photo.hpp"
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
  /*
   Calling Sequence
   src = imread("image-location-for-src");
   outputImage = detailEnhance(src,sigma_s,sigma_r)

  */

  int opencv_detailEnhance(char *fname, unsigned long fname_len)
  {

    SciErr sciErr;
    int intErr = 0;
    int iRows=0,iCols=0;
    int *piAddr = NULL;
    int *piAddrNew = NULL;
    int *piAddr2  = NULL;
    int *piAddr3  = NULL;
    int *piAddr4  = NULL;
    int *piAddr5  = NULL;
    int i,j,k;
    double sigma_s,sigma_r;
    double temp_flag;
    int flag;
    char * funcName= "detailEnhance";
    //checking input argument
    CheckInputArgument(pvApiCtx, 3, 3);
    //checking output argument
    CheckOutputArgument(pvApiCtx, 1, 1) ;

    Mat src;
    retrieveImage(src, 1);
 
    if(src.channels()!=3)
    {
        Scierror(999, "%s: input argument #%d requires a 3 channel image .\n",funcName,1);
        return -1;
    }
   
    //converting src to 8 bit 3 channel
    src.convertTo(src,CV_8UC3);
 
     //for value of sigma_s
    sciErr = getVarAddressFromPosition(pvApiCtx,2,&piAddr2);
    if (sciErr.iErr)
    {
        printError(&sciErr, 0);
        return -1;
    }
    
     
     if (isDoubleType(pvApiCtx, piAddr2) )
    {
        if(isScalar(pvApiCtx, piAddr2))
		{
	        	
		    intErr = getScalarDouble(pvApiCtx, piAddr2, &sigma_s);
            if(intErr)
            {
                return intErr;
            }

            if( (sigma_s <= 0) || (sigma_s  >= 200))
            {
                Scierror(999, "%s: value of input argument #%d should be between 0 & 200 .\n",funcName,2);
                return -1;

            }
       }
		else
		{
		    Scierror(999, "%s: Wrong type for input argument #%d: A scalar expected.\n", funcName, 2);
            return -1;

		} 
            
    }
    else
    {
        Scierror(999, "%s: Wrong type for input argument #%d: A scalar float expected.\n", funcName,2);
        return -1;

    }
    

    //for value of sigma_r
    sciErr = getVarAddressFromPosition(pvApiCtx,3,&piAddr3);
    if (sciErr.iErr)
    {
        printError(&sciErr, 0);
        return -1;
    }
    if ( isDoubleType(pvApiCtx, piAddr3) )
    {
        if(isScalar(pvApiCtx, piAddr3))
		{
	        	
		    intErr = getScalarDouble(pvApiCtx, piAddr3, &sigma_r);
            if(intErr)
            {
                return intErr;
            }

            if( (sigma_r <= 0) || (sigma_r  >= 1))
            {
                Scierror(999, "%s: value of input argument #%d should be between 0 & 1 !\n",funcName,3);
                return -1;

            }
       }
		else
		{
		    Scierror(999, "%s: Wrong type for input argument #%d: A scalar expected.\n", funcName, 3);
            return -1;

		} 
            
    }
    else
    {
        Scierror(999, "%s: Wrong type for input argument #%d: A scalar float expected.\n", funcName,3);
        return -1;

    }
       
    
    
    
    

    Mat output_image(src.rows,src.cols,CV_8UC3);

    try
    {
        detailEnhance(src,output_image,sigma_s,sigma_r);
    }
    catch(Exception& e)
    {
        const char* err = e.what();
        Scierror(999, "%s",err);
        return -1;
        
    }
    string tempstring = type2str(output_image.type());
    char *checker;
    checker = (char *)malloc(tempstring.size() + 1);
    memcpy(checker, tempstring.c_str(), tempstring.size() + 1);
    returnImage(checker,output_image,1);
    free(checker);

    //Assigning the list as the Output Variable
    AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
    //Returning the Output Variables as arguments to the Scilab environment
    ReturnArguments(pvApiCtx);
    return 0;

  }
/* ==================================================================== */
}
