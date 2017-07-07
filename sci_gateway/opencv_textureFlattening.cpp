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

/* Calling Sequence
   
   src = imread("image-location-for-src");
   mask = roiFreeHand(src); //we can also use other function to create the mask
   outputImage = textureFlattening(src,mask,low_threshold,high_threshold, kernel_size); 
*/

  int opencv_textureFlattening(char *fname, unsigned long fname_len)
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
    double * pointMatrix =NULL;
    int m1,n1;
    int kernel_size;
    
    //for point
    int x,y;
    double low_threshold,high_threshold,temp_kernel_size;
    char * funcName= "textureFlattening";
    //checking input argument
    CheckInputArgument(pvApiCtx,5,5);
    //checking output argument
    CheckOutputArgument(pvApiCtx, 1, 1) ;

     Mat src,mask,dst;
     //retrieveing src 
     retrieveImage(src, 1);
    
     if(src.channels()!=3)
    {
        Scierror(999, "%s: input argument #%d requires a 3 channel image.\n",funcName,1);
        return -1;
    }
   
    //converting src to 8 bit 3 channel
    src.convertTo(src,CV_8UC3);

    //retrieveing mask 
    retrieveImage(mask, 2);
    
     //size of mask should be same as that of src
    if( (src.rows!=mask.rows) || (src.cols!=mask.cols) )
    {
        Scierror(999, "%s: in input argument #%d - size of mask is not equal to size of source image ! Same size required !\n",funcName,2);
        return -1;
    
    }
   
    if(mask.channels()==1)
    {
        //convert to 8 bit 1 channel
            mask.convertTo(mask,CV_8UC1);
    }
    else if(mask.channels()==3)
    {
        //convert to 8 bit 3 channel
        mask.convertTo(mask,CV_8UC3);

    }
    else
    {
       Scierror(999, "%s: in input argument #%d requires either a 1 or 3 channel image !\n",funcName,2);
        return -1;
    
    }
    
         
     // for low threshold
    
    sciErr = getVarAddressFromPosition(pvApiCtx, 3, &piAddr3);
    if (sciErr.iErr)
    {
        printError(&sciErr, 0);
        return -1;
    }


    
    if (isDoubleType(pvApiCtx, piAddr3) )
    {
        if(isScalar(pvApiCtx, piAddr3))
		{
	        	
		    intErr = getScalarDouble(pvApiCtx, piAddr3, &low_threshold);
            if(intErr)
            {
                return intErr;
            }
            
            if( (low_threshold < 0) || (low_threshold  > 100))
            {
                Scierror(999, "%s: Out of range! value of input argument #%d should be from 0 to 100 .\n",funcName,3);
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
        Scierror(999, "%s: Wrong type for input argument #%d: A scalar double expected.\n", funcName,3);
        return -1;

    }
    
    
    
     
    //for high threshold
    sciErr = getVarAddressFromPosition(pvApiCtx, 4, &piAddr4);
    if (sciErr.iErr)
    {
        printError(&sciErr, 0);
        return -1;
    }


         
    if (isDoubleType(pvApiCtx, piAddr4) )
    {
        if(isScalar(pvApiCtx, piAddr4))
		{
	        	
		    intErr = getScalarDouble(pvApiCtx, piAddr4, &high_threshold);
            if(intErr)
            {
                return intErr;
            }
            
            if( (high_threshold < 100 || high_threshold > 255) )
            {
                Scierror(999, "%s: Out of range! argument #%d should be greater than  100 and less than or equal to 255 .\n",funcName,4);
                return -1;

            }
       }
		else
		{
		    Scierror(999, "%s: Wrong type for input argument #%d: A scalar expected.\n", funcName, 4);
            return -1;

		} 
            
    }
    else
    {
        Scierror(999, "%s: Wrong type for input argument #%d: A scalar double expected.\n", funcName, 4);
        return -1;

    }
    
    //for kernel_size
    sciErr = getVarAddressFromPosition(pvApiCtx, 5, &piAddr5);
    if (sciErr.iErr)
    {
        printError(&sciErr, 0);
        return -1;
    }


         
    if (isDoubleType(pvApiCtx, piAddr5) )
    {
        if(isScalar(pvApiCtx, piAddr5))
		{
	        	
		    intErr = getScalarDouble(pvApiCtx, piAddr5, &temp_kernel_size);
            if(intErr)
            {
                return intErr;
            }
            kernel_size = int(temp_kernel_size);
            
            if(! (kernel_size == 3 || kernel_size == 5 || kernel_size == 7))
            {
                Scierror(999, "%s: For input argument #%d : size of kernel can only be 3, 5 or 7.\n",funcName,5);
                return -1;

            }
           
       }
		else
		{
		    Scierror(999, "%s: Wrong type for input argument #%d: A scalar expected.\n", funcName, 5);
            return -1;

		} 
            
    }
    else
    {
        Scierror(999, "%s: Wrong type for input argument #%d: A scalar int expected.\n", funcName, 5);
        return -1;

    }
    
    Mat output_image(src.rows,src.cols,CV_8UC3);
 
    try
    {
        textureFlattening(src,mask,output_image,low_threshold,high_threshold, kernel_size);
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
