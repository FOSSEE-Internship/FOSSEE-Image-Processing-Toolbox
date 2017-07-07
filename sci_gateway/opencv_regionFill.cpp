/********************************************************
Author: Vinay Bhat,Rohan Gurve
********************************************************
Usage: return_image = regionFill(input_image,inpaintRadius, column_list, row_list)
       return_image = regionFill(input_image,inpaintRadius, mask)
Example:
  im = regionFill(image, 3,[0 100 100 0], [0 0 100 100])
  im = regionFill(image, 1, mask)
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

  int opencv_regionFill(char *fname, unsigned long fname_len)
  {

    SciErr sciErr;
    int intErr = 0;
    int iRowsR=0,iColsR=0,iColsC=0,iRowsC=0;
    int *piAddr = NULL;
    int *piAddr2  = NULL;
    int *piAddr3  = NULL;
    int *piAddr4  = NULL;
    double *pstDataR = NULL;
    double *pstDataC = NULL;
    int i, number_of_points;
    int lineType = 8;
    double inpaintRadius;
    int iRet;
    
    //checking input argument
    CheckInputArgument(pvApiCtx, 3, 4);
    CheckOutputArgument(pvApiCtx, 1, 1);

    // Get the input image from the Scilab environment
    Mat image, mask, mask_orig, fin_image;
    retrieveImage(image, 1);

    //get the inPaint radius
     sciErr = getVarAddressFromPosition(pvApiCtx, 2, &piAddr2);
        if (sciErr.iErr)
        {
            printError(&sciErr, 0);
            return -1;
        }
            
    
    iRet = getScalarDouble(pvApiCtx, piAddr2, &inpaintRadius);
    if(iRet)
    {
        return iRet;
    }

    if (nbInputArgument(pvApiCtx) == 3)
    {   
        retrieveImage(mask_orig, 3);

         if(mask_orig.channels()==3)
         {
            try
            {
            mask_orig.convertTo(mask_orig,CV_8UC3);
            cvtColor(mask_orig, mask_orig, CV_BGR2GRAY);
            }   
            catch(Exception& e)
            {
            const char* err = e.what();
            Scierror(999, "%s",err);
            return -1;
                  
            }
                         
         
         }
        else
        {
            try
            {
                mask_orig.convertTo(mask_orig,CV_8UC1);
            }   
            catch(Exception& e)
            {
                const char* err = e.what();
                Scierror(999, "%s",err);
                return -1;
                  
            }
         }
          
          
          mask = mask_orig.clone();
        
    }
    else
    {
        // Get the address of 3rd argument, the column list
        sciErr = getVarAddressFromPosition(pvApiCtx, 3, &piAddr3);
        if (sciErr.iErr)
        {
            printError(&sciErr, 0);
            return -1;
        }

        // Get the address of the 4th argument, the row list
        sciErr = getVarAddressFromPosition(pvApiCtx, 4, &piAddr4);
        if (sciErr.iErr)
        {
            printError(&sciErr, 0);
            return -1;
        }

        // Get the column list in the form of a matrix 
        // No. of columns = No. of elements in the list
        // No. of rows = 1 
        sciErr = getMatrixOfDouble(pvApiCtx, piAddr3, &iRowsC, &iColsC, &pstDataC);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return -1;
        }

        if (iRowsC != 1)
        {
                   
            Scierror(999, "%s","Please enter a list of column coordinates.\n");
            return -1;
       
        }
        
        // Get the row list in the form of a matrix
        // No. of columns = No. of elements in the list
        // No. of rows = 1
        sciErr = getMatrixOfDouble(pvApiCtx, piAddr4, &iRowsR, &iColsR, &pstDataR);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return -1;
        }

        if (iRowsR != 1)
        {
            sciprint("Please enter a list of row coordinates.\n");
            return -1;
        }

        if (iColsC != iColsR)
        {
            sciprint("Please ensure number of elements in both row and column lists are equal.\n");
            return -1;
        }

        // Number of points is number of columns
        number_of_points = iColsC;

        // Create the n points which define
        // the polygon
        Point points[1][number_of_points];
        
        for (i = 0; i < number_of_points; i++)
        {
            if (pstDataR[i] < 0 || pstDataC[i] < 0)
            {
                sciprint("Coordinates cannot be negative.\n");
                return -1;
            }
            else
                points[0][i] = Point(pstDataR[i], pstDataC[i]);
        }

        const Point* ppt[1] = { points[0] };

        int npt[] = { number_of_points };

        // Create a new, black, blank image same size as of input
        mask_orig = Mat::zeros(image.size(),CV_8UC3 );
        
        // Call the fillPoly OpenCV function
        // Fill the blank image in the polygon specified
        // by the points
        
        
        
        try
        {   
            fillPoly(mask_orig, ppt, npt, 1, Scalar(255, 255, 255), lineType);
        }
        catch(Exception& e)
        {
            const char* err = e.what();
            Scierror(999, "%s",err);
            return -1;
                  
        }  
     
        try
        {
            mask_orig.convertTo(mask_orig,CV_8UC3);
            cvtColor(mask_orig, mask_orig, CV_BGR2GRAY);
            // Binary Threshold
            threshold(mask_orig,mask_orig, 0, 255, THRESH_BINARY);
     
        }   
        catch(Exception& e)
        {
            const char* err = e.what();
            Scierror(999, "%s",err);
            return -1;
              
        }
 
        mask = mask_orig.clone();
    
    }

    
        try
        {   
            image.convertTo(image,CV_8U);//convert image to 8 bit
            inpaint(image, mask, fin_image, inpaintRadius, INPAINT_TELEA);

        }
        catch(Exception& e)
        {
            const char* err = e.what();
            Scierror(999, "%s",err);
            return -1;
                  
        } 
    
    
    string tempstring = type2str(fin_image.type());
    char *checker;
    checker = (char *)malloc(tempstring.size() + 1);
    memcpy(checker, tempstring.c_str(), tempstring.size() + 1);
    returnImage(checker, fin_image, 1);
    free(checker);

    //Assigning the list as the Output Variable
    AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
    //Returning the Output Variables as arguments to the Scilab environment
    ReturnArguments(pvApiCtx);
    return 0;

  }
/* ==================================================================== */
}
