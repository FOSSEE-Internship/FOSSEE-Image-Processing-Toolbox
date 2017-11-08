/********************************************************
Author : Deepshikha
[key_value] : blobAnalysis(source_image)

// 1 :image
Optional arguments :-
// 2 : filterByArea 
// 3 : filterByThreshold
// 4 : filterByCircularity
// 5 : filterByConvexity
********************************************************/

#include <numeric>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/features2d/features2d.hpp"
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
  
  int opencv_blobAnalysis(char *fname, unsigned long fname_len)
  {

    // Error management variable
    SciErr sciErr;
    
    //Variables
    // int i, j;
    int i = 0;
    int intErr;
    int iRows1 = 0;
    int iCols1 = 0;
	
	  int iRows = 0;
    int iCols = 0;
    int *piLen = NULL;
    int *piAddr = NULL;
    char **pstData = NULL;
    char *currentArg = NULL;
    bool *providedArgs = NULL; 
    
    double *area = NULL;
    double *threshold = NULL; 
    double *circularity = NULL;
    double *convexity = NULL;
    double *ROImat = NULL;
    double *key_value = NULL;

    // checking input argument
    // 1 : image
    // 2 : filterByArea 
    // 3 : filterByThreshold
    // 4 : filterByCircularity
    // 5 : ROI
    // 6 : filterByConvexity
    CheckInputArgument(pvApiCtx, 1, 11);
    CheckOutputArgument(pvApiCtx, 1, 1);

    // retrieve image 
    Mat image;
    retrieveImage(image, 1);

    // For the optional arguments
    int nbInputArguments = *getNbInputArgument(pvApiCtx);
    //sciprint("%d\n",nbInputArguments);
    
    if((nbInputArguments%2) == 0)
    {
    	Scierror(999, "%d:  The number of arguments must be odd\n");
        return 0;
    }
    
    providedArgs = (bool*) malloc(sizeof(bool) * 5); 
    memset(providedArgs, 0, 5);

    for(int iter = 2; iter <= nbInputArguments; ++iter)
    {
        sciErr = getVarAddressFromPosition(pvApiCtx, iter, &piAddr); 
        if (sciErr.iErr)
        {
            printError(&sciErr, 0); 
            return 0; 
        }

        /// Three calls to getMatrixOfString
        //First Call
        sciErr = getMatrixOfString(pvApiCtx, piAddr, &iRows, &iCols, NULL, NULL); 
        if (sciErr.iErr)
        {
            printError(&sciErr, 0); 
            return 0; 
        }
        
        // Second call to get length of string
        piLen = (int*) malloc(sizeof(int) * iRows1 * iCols1); 
        sciErr = getMatrixOfString(pvApiCtx,  piAddr,  &iRows1,  &iCols1,  piLen,  NULL); 
        if (sciErr.iErr)
        {
            printError(&sciErr, 0); 
            return 0; 
        }
        
        // third call
        pstData = (char**) malloc(sizeof(char*) * iRows1 * iCols1); 
        for(int k = 0; k < iRows1 * iCols1; ++k)
        {
            pstData[k] = (char*) malloc(sizeof(char) * piLen[k] + 1); 
        }
        
        sciErr = getMatrixOfString(pvApiCtx, piAddr, &iRows1, &iCols1, piLen, pstData); 
        if (sciErr.iErr)
        {
            printError(&sciErr, 0); 
            return 0; 
        }
        currentArg = pstData[0];         
        
        // getting filterbyArea
        if(strcmp(currentArg, "filterByArea") == 0)
        {
            if(iter + 1 <= nbInputArguments and !providedArgs[0])
            {
                sciErr = getVarAddressFromPosition(pvApiCtx, ++iter, &piAddr); 
                if (sciErr.iErr)
                {
                    printError(&sciErr, 0); 
                    return 0; 
                }
                sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &iRows1, &iCols1, &area); 
                if(sciErr.iErr)
                {
                    printError(&sciErr, 0); 
                    return 0; 
                }
                if(iRows1 != 1 or iCols1 != 2)
                {
                    Scierror(999, "Incorrect dimension of matrix for argument .\n"); 
                    return 0; 
                }
                providedArgs[0] = 1; 
            }
            else if(providedArgs[0]) // Send an error message if an argument is provided more than once. Same for all optional arguments.
            {
                Scierror(999, "Please provide optional arguments only once.\n"); 
                return 0; 
            }
            else // Send an error message if name of argument is given but type is incorrect. Same for all optional arguments.
            {
                Scierror(999, "Incorrect number of arguments provided. Please check the documentation for more information.\n"); 
                return 0; 
            }
        }

            /// filterbyThreshold
            else if(strcmp(currentArg, "filterByThreshold") == 0)
            {
                if(iter+1 <= nbInputArguments && !providedArgs[1])
                {
                    sciErr = getVarAddressFromPosition(pvApiCtx, ++iter, &piAddr); 
                    if (sciErr.iErr)
                    {
                        printError(&sciErr, 0); 
                        return 0; 
                    }
                    sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &iRows1, &iCols1, &threshold); 
                    if(sciErr.iErr)
                    {
                        printError(&sciErr, 0); 
                        return 0; 
                    }
                    if(iRows1 != 1 or iCols1 != 2)
                    {
                        Scierror(999, "Incorrect dimension of matrix for argument .\n"); 
                        return 0; 
                    }
                    
                    // Checking if values are in proper range. Same for all optional arguments
                    providedArgs[1] = 1;
                }
                else if(providedArgs[1]) // Send an error message if an argument is provided more than once. Same for all optional arguments.
                {
                    Scierror(999, "Please provide optional arguments only once.\n"); 
                    return 0; 
                }
                else // Send an error message if name of argument is given but type is incorrect. Same for all optional arguments.
                {
                    Scierror(999, "Incorrect number of arguments provided. Please check the documentation for more information.\n"); 
                    return 0; 
                }
            }

            /// filterbycircularity
            else if(strcmp(currentArg, "filterByCircularity") == 0)
            {
                if(iter+1 <= nbInputArguments && !providedArgs[2])
                {
                    sciErr = getVarAddressFromPosition(pvApiCtx, ++iter, &piAddr); 
                    if (sciErr.iErr)
                    {
                        printError(&sciErr, 0); 
                        return 0; 
                    }
                    sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &iRows1, &iCols1, &circularity); 
                    if(sciErr.iErr)
                    {
                        printError(&sciErr, 0); 
                        return 0; 
                    }
                    if(iRows1 != 1 or iCols1 != 2)
                    {
                        Scierror(999, "Incorrect dimension of matrix for argument .\n"); 
                        return 0; 
                    }
                    
                    // Checking if values are in proper range. Same for all optional arguments
                    providedArgs[2] = 1;
                }
                else if(providedArgs[2]) // Send an error message if an argument is provided more than once. Same for all optional arguments.
                {
                    Scierror(999, "Please provide optional arguments only once.\n"); 
                    return 0; 
                }
                else // Send an error message if name of argument is given but type is incorrect. Same for all optional arguments.
                {
                    Scierror(999, "Incorrect number of arguments provided. Please check the documentation for more information.\n"); 
                    return 0; 
                }
            }

            else if(strcmp(currentArg,"ROI")==0)
            {
                // val_position=i+1;
                
                if(providedArgs[3] == 1)
                {
                    Scierror(999,"Do not enter the same parameter\n");
                    return 0;
                }
                
                sciErr = getVarAddressFromPosition(pvApiCtx, i + 1, &piAddr);
                if (sciErr.iErr)
                {
                    printError(&sciErr, 0);
                    return 0;
                }

                if(!isDoubleType(pvApiCtx, piAddr) || isVarComplex(pvApiCtx, piAddr))
                {
                    Scierror(999,"Enter a List of 4 arguments\n");
                    return 0;
                }

                sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &iRows, &iCols, &ROImat);  
                if(sciErr.iErr)
                {
                    printError(&sciErr, 0);
                    return 0;
                }

                if(iRows*iCols!=4)
                {   
                    Scierror(999,"Invalid Argument\n");
                    return 0;
                }

                if(ROImat[0]<0 || ROImat[1]<0 || ROImat[2]<0 || ROImat[3]<0)
                {   
                    Scierror(999,"Arguments cannot be negative\n");
                    return 0;
                }       

                if(ROImat[0]>image.cols || ROImat[1]>image.rows || ROImat[0]+ROImat[2]>image.cols
                    || ROImat[1]+ROImat[3]>image.rows) 
                {
                    Scierror(999,"Please make sure the arguments are within the image range\n");
                    return 0;
                }

                providedArgs[3] = 1;
            }

            /// filterbyConvexity
            else if(strcmp(currentArg, "filterByConvexity") == 0)
            {
                if(iter + 1 <= nbInputArguments and !providedArgs[4])
                {
                    sciErr = getVarAddressFromPosition(pvApiCtx, ++iter, &piAddr); 
                    if (sciErr.iErr)
                    {
                        printError(&sciErr, 0); 
                        return 0; 
                    }
                    sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &iRows1, &iCols1, &convexity); 
                    if(sciErr.iErr)
                    {
                        printError(&sciErr, 0); 
                        return 0; 
                    }
                    if(iRows1 != 1 or iCols1 != 2)
                    {
                        Scierror(999, "Incorrect dimension of matrix for argument .\n"); 
                        return 0; 
                    }

                    providedArgs[4] = 1; 
                }
                else if(providedArgs[4]) // Send an error message if an argument is provided more than once. Same for all optional arguments.
                {
                    Scierror(999, "Please provide optional arguments only once.\n"); 
                    return 0; 
                }
                else // Send an error message if name of argument is given but type is incorrect. Same for all optional arguments.
                {
                  Scierror(999, "Incorrect number of arguments provided. Please check the documentation for more information.\n"); 
                  return 0; 
                }
            }

            else
            {
                Scierror(9,"Invalid argument passed");
                return 0;
            }
        }   
        /// End of error check and input get;

        /// params for holding the parameters values;
        SimpleBlobDetector::Params params;
        
        /// if area is provided
        if(providedArgs[0] == 1)
        {
            params.filterByArea = true;
            params.minArea = area[0];
            params.maxArea = area[1];
        }

        /// if threshold is provided
        if(providedArgs[1] == 1)
        {
            sciprint("%d %d", int(threshold[0]), int(threshold[1]));
            params.minThreshold = int(threshold[0]);
            params.maxThreshold = int(threshold[1]);
        }

        /// if Circularity is provided
        if(providedArgs[2] == 1)
        {
            params.filterByCircularity = true;
            params.minCircularity = circularity[0];
            params.maxCircularity = circularity[1];
        }

        if(providedArgs[3]==0)
        {
            ROImat=(double *)malloc(sizeof(double)*1*4);
            ROImat[0]=0;
            ROImat[1]=0;
            ROImat[2]=image.cols;
            ROImat[3]=image.rows;       
        }

        Rect masker(ROImat[0], ROImat[1], ROImat[2], ROImat[3]);
        Mat mask(image.size(), CV_8UC1, Scalar::all(0));
        
        // Creating ROI
        try
        {
            mask(masker).setTo(Scalar::all(255));
        }
        catch(Exception &e)
        {
          const char *err = e.what();
          Scierror(999, "%s", err);
        }
        
        /// if convexity is provided
        if(providedArgs[4] == 1)
        {
            params.filterByConvexity = true;
            params.minConvexity = convexity[0];
            params.maxConvexity = convexity[1];
        }

        Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
        /// to store the keypoints of blobs detected
        vector<KeyPoint> keyPoints;
        Mat descriptors;
        
        try
        {
          image.convertTo(image, CV_8UC1);
          /// fuction to detect blob in the image
          detector->detect(image, keyPoints);
        }
        catch(Exception &e)
        {
          const char *err = e.what();
          Scierror(999, "%s", err);
          return 0;
        }

        key_value = (double*)malloc(sizeof(double) * (int)keyPoints.size() * 2);

        double *size_value = NULL;
        size_value = (double*)malloc(sizeof(double) * (int)keyPoints.size() * 1);

        int total_keypoints = (int)keyPoints.size();

        vector< KeyPoint >::iterator it;

        i = 0;
        for(it = keyPoints.begin(); it != keyPoints.end(); ++it)
        {
          KeyPoint temp = keyPoints[i];

          // x coordinate
          key_value[i + 0 * total_keypoints] = it->pt.x;

          // y coordinate
          key_value[i + 1 * total_keypoints] = it->pt.y;

          // size
          size_value[i + 0 * total_keypoints] = (int)it->size;

          //area
          //key_value[i + 3 * total_keypoints] = it->Area;
          i++;
        }

        // Creating output arguments
        sciErr = createList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, 2, &piAddr);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }

        sciErr = createMatrixOfDoubleInList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, piAddr, 1, total_keypoints, 2, key_value);
        if(sciErr.iErr)
        {
          printError(&sciErr, 0);
          return 0;
        }

        sciErr = createMatrixOfDoubleInList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, piAddr, 2, total_keypoints, 1, size_value);
        if(sciErr.iErr)
        {
          printError(&sciErr, 0);
          return 0;
        }

        // sciErr = createMatrixOfDoubleInList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, piAddr, 3, 1, dcols, _descriptors);
        // if(sciErr.iErr)
        // {
        //   printError(&sciErr, 0);
        //   return 0;
        // }

        //Assigning the list as the Output Variable
        AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;

        //Returning the Output Variables as arguments to the Scilab environment
        ReturnArguments(pvApiCtx);
        return 0;
    }
/* ==================================================================== */
}
