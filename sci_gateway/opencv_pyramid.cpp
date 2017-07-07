/********************************************************
Author: Diwakar Bhardwaj
*********************************************************
Usage :return_image=pyramid(input_image,'direction',level)
********************************************************/

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
  #include "sciprint.h"
  #include "../common.h"
  
  int opencv_pyramid(char *fname, unsigned long fname_len)
  {
    SciErr sciErr;
    int intErr = 0;
    int iRows=0,iCols=0;
    int *piAddr = NULL;
    int *piAddrNew = NULL;
    int *piAddr2  = NULL;
    int *piAddr3  = NULL;
    int *piLen = NULL;
    char **direction = NULL;
    int i,j,k=0,l=0;
    double alpha, beta,x,n;
    
     
    //checking input argument
    CheckInputArgument(pvApiCtx, 3, 3);
    CheckOutputArgument(pvApiCtx, 1, 1) ;
    Mat image,new_image;
    
    retrieveImage(image, 1);

    sciErr = getVarAddressFromPosition(pvApiCtx,2, &piAddr2);

    if (sciErr.iErr)
    {
        printError(&sciErr, 0);
        return -1;
    }
    sciErr = getMatrixOfString(pvApiCtx, piAddr2, &iRows, &iCols, NULL, NULL);
    if(sciErr.iErr)
    {
        printError(&sciErr, 0);
        return -1;
    }
    piLen = (int*)malloc(sizeof(int) * iRows * iCols);
    //second call to retrieve length of each string
    sciErr = getMatrixOfString(pvApiCtx, piAddr2, &iRows, &iCols, piLen, NULL);
    if(sciErr.iErr)
    {
        printError(&sciErr, 0);
        return -1;
    }
    direction = (char**)malloc(sizeof(char*) * iRows * iCols);
    for(i = 0 ; i < iRows * iCols ; i++)
    {
        direction[i] = (char*)malloc(sizeof(char) * (piLen[i] + 1));//+ 1 for null termination
    }
    //third call to retrieve data
    sciErr = getMatrixOfString(pvApiCtx, piAddr2, &iRows, &iCols, piLen, direction);
    if(sciErr.iErr)
    {
        printError(&sciErr, 0);
        return -1;
    }

     sciErr = getVarAddressFromPosition(pvApiCtx,3,&piAddr3);
    if (sciErr.iErr)
    {
        printError(&sciErr, 0);
        return -1;
    }
    intErr = getScalarDouble(pvApiCtx, piAddr3 ,&x);
    if(intErr)
      return intErr;
     n=x;
     k=image.rows;
     l=image.cols; 
     while(n--)
    {
     
     //if expand operation is performed
     if(strcmp(direction[0], "expand") == 0)
     {   
     
          try
            {
            pyrUp(image, new_image, Size( image.cols*2, image.rows*2 ));
          }
            catch(Exception& e)
            {
            const char* err = e.what();
            Scierror(999, "%s",err);
            return -1;
                  
            }  
     
          //Size size((k*2)-1,(l*2)-1); 
          //resize(image,new_image,size);
     }

     
     //if reduced operation is performed
     else if(strcmp(direction[0], "reduce") == 0)
    {  
          try
            {
             pyrDown( image, new_image, Size( image.cols/2, image.rows/2 ));
     
          }
            catch(Exception& e)
            {
            const char* err = e.what();
            Scierror(999, "%s",err);
            return -1;
                  
            }  
     }
     image=new_image;

    }
    
    
 

    int temp = nbInputArgument(pvApiCtx) + 1;
    string tempstring = type2str(new_image.type());
    char *checker;
    checker = (char *)malloc(tempstring.size() + 1);
    memcpy(checker, tempstring.c_str(), tempstring.size() + 1);
    returnImage(checker,new_image,1);
    free(checker); 

    //Assigning the list as the Output Variable
    AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
    //Returning the Output Variables as arguments to the Scilab environment
    ReturnArguments(pvApiCtx);
    return 0;

  }
/* ==================================================================== */
}


