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

 /*Calling Sequence
      
      src = imread("image-location-for-src");
      mask = roiFreeHand(src); //we can also use other function to create the mask
      outputImage = seamlessClone(src,dst,mask, pointMatirx, flag) 

 */ 
  
  
  int opencv_seamlessClone(char *fname, unsigned long fname_len)
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
    int flag;
    
    //for point
    int x,y;
    double alpha,beta,temp_flag;
    char * funcName= "seamlessClone";
    //checking input argument
    CheckInputArgument(pvApiCtx, 5, 5);
    //checking output argument
    CheckOutputArgument(pvApiCtx, 1, 1) ;

    Mat src,mask,dst;
    //retrieveing src 
    retrieveImage(src, 1);
    
     if(src.channels()!=3)
    {
        Scierror(999, "%s: input argument #%d requires a 3 channel image .\n",funcName,1);
        return 0;
    }
  
   
    //converting src to 8 bit 3 channel
    src.convertTo(src,CV_8UC3);

    //retrieveing dst
    retrieveImage(dst, 2);

     
     
     
     if(dst.channels()!=3)
    {
        Scierror(999, "%s: input argument #%d requires a 3 channel image .\n",funcName,2);
        return -1;
    }
    
    
    if(dst.rows < src.rows || dst.cols < src.cols)
    {
        Scierror(999, "%s: input argument #%d : size of destination image is less than size of source image!  Size of destination image should be atleast equal to the size of source image! \n",funcName,2);
        return -1;
    }
    
    
    
    
    //converting dst to 8 bit 3 channel
    dst.convertTo(dst,CV_8UC3);

    //retrieveing mask 
    retrieveImage(mask, 3);
    
     //size of mask should be same as that of src
    if( (src.rows!=mask.rows) || (src.cols!=mask.cols) )
    {
        Scierror(999, "%s: in input argument #%d - size of mask is not equal to size of source image ! Same size required !\n",funcName,3);
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
       Scierror(999, "%s: in input argument #%d requires either a 1 or 3 channel image !\n",funcName,3);
        return -1;
    
    }
        
    //for value of p
    sciErr = getVarAddressFromPosition(pvApiCtx, 4, &piAddr4);
    if (sciErr.iErr)
    {
        printError(&sciErr, 0);
        return -1;
    }

    /* Check that the 4th input argument is a real matrix (and not complex) */
    if ( !isDoubleType(pvApiCtx, piAddr4) ||  isVarComplex(pvApiCtx, piAddr4) )
    {
        
    
        Scierror(999, "%s: Wrong type for input argument #%d: A real matrix expected.\n",funcName, 4);
        return -1;
    }
    
    /* get matrix */
    sciErr = getMatrixOfDouble(pvApiCtx, piAddr4, &m1, &n1, &pointMatrix);
    if (sciErr.iErr)
    {
        sciprint("get matrix main");
        printError(&sciErr, 0);
        return -1;
    }
    
    if((m1!=1) || (n1!=2))
    {
       Scierror(999, "%s: a 1x2 real matrix expcted for argument #%d .\n",funcName,4);
       return -1;
    } 
     
     
    Point p(pointMatrix[0],pointMatrix[1]);
 
     
   
    //for flag
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
	        	
		    intErr = getScalarDouble(pvApiCtx, piAddr5, &temp_flag);
            if(intErr)
            {
                return intErr;
            }
            flag = int(temp_flag);
            
            if( (flag < 1) || (flag  > 3))
            {
                Scierror(999, "%s: value of input argument #%d should be 1, 2 or 3 .\n",funcName,5);
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
          switch(flag)
          {
            case 1:seamlessClone(src,dst,mask,	p, output_image,NORMAL_CLONE); 
                   break; 
            case 2:seamlessClone(src,dst,mask,	p, output_image,MIXED_CLONE); 
                   break; 
            case 3:seamlessClone(src,dst,mask,	p, output_image,MONOCHROME_TRANSFER); 
                   break;               
            
          
          }
          
          
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
