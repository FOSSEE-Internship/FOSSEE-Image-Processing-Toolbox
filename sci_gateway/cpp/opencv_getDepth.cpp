/***************************************************
Author : Rohan Gurve
****************************************************
   depth = getDepth(src)
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
  
  int opencv_getDepth(char *fname, unsigned long fname_len)
  {

    SciErr sciErr;
    int intErr=0;
    
    char **postData = NULL;
    int i;
    int size;
    int iRows		= 1;
	int iCols		= 1;    
    char * depth;
    //checking input argument
    CheckInputArgument(pvApiCtx, 1, 1);
    CheckOutputArgument(pvApiCtx, 1, 1) ;

    Mat src;
    retrieveImage(src,1);
   
   postData= (char**)malloc(sizeof(char*) * (iRows*iCols));
     
     try
     {
         switch(src.depth())
         {
          case CV_8U : depth= "CV_8U"; 
                       postData[0] = depth;
                        break; 
         
         
          case CV_8S : depth = "CV_8S"; 
                         postData[0] = depth;
                         break;   
         
          case CV_16U : depth = "CV_16U"; 
                        postData[0] = depth;
                         break;
 
          case CV_16S : depth = "CV_16S"; 
                        postData[0] = depth;
                        
                       break;
              
          case CV_32S : depth = "CV_32S"; 
                        postData[0] = depth;
                         break;
      
          //scialb dosen't support CV_32F - it would be treated as CV_64F
           
          case CV_64F : depth = "CV_64F"; 
                       
                       postData[0] = depth;
                        break;  
        
         }
     }
    catch(Exception& e)
    {
        const char* err = e.what();
        Scierror(999, "%s",err);
        return 0;
          
    }


    /* Create the stringmatrix as return of the function */
    sciErr = createMatrixOfString(pvApiCtx, nbInputArgument(pvApiCtx) + 1, iRows,iCols, postData);
    free(postData); // Data have been copied into Scilab memory
    if (sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }


 
    //Assigning the list as the Output Variable
    AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
    //Returning the Output Variables as arguments to the Scilab environment
    ReturnArguments(pvApiCtx);
    return 0;

  }
/* ==================================================================== */
}
