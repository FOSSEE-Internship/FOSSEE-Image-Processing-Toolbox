/********************************************************
Author: Diwakar Bhardwaj
*********************************************************
Usage :return_image=GeometricShearer(input_image,'direction',value)
value is the amount want to shear the image in given direction 
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
  
  int opencv_GeometricShearer(char *fname, unsigned long fname_len)
  {
    SciErr sciErr;
    int intErr = 0;
    int iRows=0,iCols=0;
    int *piAddr = NULL;
    int *piAddrNew = NULL;
    int *piAddr2  = NULL;
    int *piAddr3  = NULL;
    int *piAddr4  = NULL;
    int *piLen = NULL;
    char **direction = NULL;
    int i,j,k;
    double alpha, beta,x,n,value;
    
    
     
    //checking input argument
    CheckInputArgument(pvApiCtx,3,3);
    CheckOutputArgument(pvApiCtx, 1, 1) ;
    Mat image,new_image;
    
    retrieveImage(image, 1);
    sciprint("%d\n%d\n",image.rows,image.cols);

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
        return 0;
    }

     sciErr = getVarAddressFromPosition(pvApiCtx,3,&piAddr3);
    if (sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }
    intErr = getScalarDouble(pvApiCtx, piAddr3 ,&value);
    if(intErr)
      return intErr;
    
    int r1, c1; // tranformed point
    int rows, cols; // original image rows and columns
    rows = image.rows;
    cols = image.cols;
    
    if(strcmp(direction[0], "Horizontal") == 0)
    {
          double Bx = value; 
          double By = 0; 

          int maxXOffset = abs(cols * Bx);
          int maxYOffset = abs(rows * By);

          Mat out = Mat::ones(image.rows+maxYOffset, image.cols+maxXOffset, image.type()); 
    
          sciprint("%d\n%d\n",out.rows,out.cols);
          for(int r = 0; r < out.rows; r++)
             {
                for(int c = 0; c < out.cols; c++)
                    {
                        r1 = r + c * By - maxYOffset;
                        c1 = r * Bx + c - maxXOffset;

                        if(r1 >= 0 && r1 <= rows && c1 >= 0 && c1 <= cols)
                          {
                              out.at<uchar>(r, c) = image.at<uchar>(r1, c1);
                           }

                    }
              }
              int temp = nbInputArgument(pvApiCtx) + 1;
              string tempstring = type2str(out.type());
              char *checker;
              checker = (char *)malloc(tempstring.size() + 1);
              memcpy(checker, tempstring.c_str(), tempstring.size() + 1);
              returnImage(checker,out,1);
              free(checker); 
     }
     else if(strcmp(direction[0], "Vertical") == 0)
    {            
                double Bx = 0; 
                double By = value; 

                int maxXOffset = abs(cols * Bx);
                int maxYOffset = abs(rows * By);

                Mat out = Mat::ones(image.rows+maxYOffset, image.cols+maxXOffset, image.type()); 

                for(int r = 0; r < out.rows; r++)
                     {
                       for(int c = 0; c < out.cols; c++)
                           {
                             r1 = r + c * By - maxYOffset;
                             c1 = r * Bx + c - maxXOffset;

                             if(r1 >= 0 && r1 <= rows && c1 >= 0 && c1 <= cols)
                                {
                                  out.at<uchar>(r, c) = image.at<uchar>(r1, c1);
                                }

                           }
                      }
                      int temp = nbInputArgument(pvApiCtx) + 1;
                      string tempstring = type2str(out.type());
                      char *checker;
                      checker = (char *)malloc(tempstring.size() + 1);
                      memcpy(checker, tempstring.c_str(), tempstring.size() + 1);
                      returnImage(checker,out,1);
                      free(checker); 

    }
    else
     {
      sciprint("Not valid 2nd argument");
     }
    

    

    //Assigning the list as the Output Variable
    AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
    //Returning the Output Variables as arguments to the Scilab environment
    ReturnArguments(pvApiCtx);
    return 0;

  }
/* ==================================================================== */
}
