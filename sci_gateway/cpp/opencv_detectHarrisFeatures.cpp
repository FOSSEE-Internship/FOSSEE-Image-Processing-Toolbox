/***************************************************
Author : Siddhant Narang
***************************************************/
#include <numeric>
#include <string.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <math.h>

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

    /* Calling syntax: detectHarrisFeatures(I) or detectHarrisFeatures(I, Name, Value)
       which uses additional options specified by one or more Name, Value pair arguments.
       Arguments Allowed : MinQuality, FilterSize, ROI, SensitivityFactor
       MinQuality        : The minimum accepted quality of corners represents a fraction of the
                           maximum corner metric value in the image. Larger values can be used
                           to remove erroneous corners.
       FilterSize        : Gaussian filter dimension
       ROI               : Rectangular region for corner detection
       SensitivityFactor : A scalar value, K, where 0 < K < 0.25, specifying the sensitivity
                           factor used in the Harris detection algorithm.                          */

    int opencv_detectHarrisFeatures(char *fname, unsigned long fname_len)
    {
        // Error management variables
        SciErr sciErr;
        int intErr;

        //------Local variables------//
        double *location = NULL;
        double *metric = NULL;
        int *piAddr = NULL;
        int *piLen = NULL;
        int nbInputArguments;
        char **pstData = NULL;
        char *currentArg = NULL;
        bool *providedArgs = NULL;
        double *matrixOfRoi;  // ROI[xStart, yStart, width, height]
        int iRows, iCols;
        Mat sourceImage, dst, dstThresholded, ucharDstThresholded, extended;
        vector<vector<Point> > contours;
        double filterSize = 5, minQuality = 1, sensitivityFactor = 0.04, blockSize = 2, maxDst, localMax;
        double sensitivityFactor1 = 0.05;
        int blockSize1 = 2, filterSize1 = 3;
        bool *included = NULL;
        int pointCount = 0, localMaxPos;
        double tempForSwapping;
        int coordinateMin, coordinatePos;

        //------Check number of parameters------//
        CheckInputArgument(pvApiCtx, 1, 9);
        CheckOutputArgument(pvApiCtx, 1, 3);

        //------Get input arguments------//
        retrieveImage(sourceImage, 1);
        if(sourceImage.channels() != 1)
        {
            Scierror(999, "The input image is not a grayscale image.");
            return 0;
        }
        matrixOfRoi = (double*) malloc(sizeof(double) * 4);
        providedArgs = (bool*) malloc(sizeof(bool) * 4);
        memset(providedArgs, 0, 4);
        matrixOfRoi[0] = 0;
        matrixOfRoi[1] = 0;
        matrixOfRoi[2] = sourceImage.cols;
        matrixOfRoi[3] = sourceImage.rows;

        nbInputArguments = *getNbInputArgument(pvApiCtx);
        // The following loop is for checking if optional arguments have been provided
        for(int iter = 2; iter <= nbInputArguments; iter++)
        {
            // Getting address of next argument
            sciErr = getVarAddressFromPosition(pvApiCtx, iter, &piAddr);
            if (sciErr.iErr)
            {
                printError(&sciErr, 0);
                return 0;
            }

            // Extracting name of next argument takes three calls to getMatrixOfString
            sciErr = getMatrixOfString(pvApiCtx, piAddr, &iRows, &iCols, NULL, NULL);
            if (sciErr.iErr)
            {
                printError(&sciErr, 0);
                return 0;
            }

            piLen = (int*) malloc(sizeof(int) * iRows * iCols);

            sciErr = getMatrixOfString(pvApiCtx,  piAddr,  &iRows,  &iCols,  piLen,  NULL);
            if (sciErr.iErr)
            {
                printError(&sciErr, 0);
                return 0;
            }

            pstData = (char**) malloc(sizeof(char*) * iRows * iCols);
            for(int iterPstData = 0; iterPstData < iRows * iCols; iterPstData++)
            {
                pstData[iterPstData] = (char*) malloc(sizeof(char) * piLen[iterPstData] + 1);
            }

            sciErr = getMatrixOfString(pvApiCtx, piAddr, &iRows, &iCols, piLen, pstData);
            if (sciErr.iErr)
            {
                printError(&sciErr, 0);
                return 0;
            }

            currentArg = pstData[0];
            // providedArgs[] makes sure that no optional argument is provided more than once
            if(strcmp(currentArg, "MinQuality")==0)
            {
                if(iter+1<=nbInputArguments && !providedArgs[0])
                {
                    sciErr = getVarAddressFromPosition(pvApiCtx, ++iter, &piAddr);
                    if (sciErr.iErr)
                    {
                        printError(&sciErr, 0);
                        return 0;
                    }
                    intErr = getScalarDouble(pvApiCtx, piAddr, &minQuality);
                    if(intErr)
                    {
                        return intErr;
                    }
                    // Checking if values are in proper range. Same for all optional arguments
                    if(minQuality < 0 || minQuality > 1)
                    {
                        Scierror(999, "Error: Please provide proper value for \"%s\". Permissible range is [0, 1].\n", currentArg);
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
            else if(strcmp(currentArg, "FilterSize")==0)
            {
                if(iter+1 <= nbInputArguments  && !providedArgs[1])
                {
                    sciErr = getVarAddressFromPosition(pvApiCtx, ++iter, &piAddr);
                    if (sciErr.iErr)
                    {
                        printError(&sciErr, 0);
                        return 0;
                    }
                    intErr = getScalarDouble(pvApiCtx, piAddr, &filterSize);
                    if(intErr)
                    {
                        return intErr;
                    }
                    providedArgs[1] = 1;
                    if(filterSize!=1 && filterSize!=3 && filterSize!=5 && filterSize!=7)
                    {
                        Scierror(999, "Error: Please provide proper value for \"%s\". Permissible values are {1, 3, 5, 7}.\n", currentArg);
                        return 0;
                    }
                }
                else if(providedArgs[1])
                {
                    Scierror(999, "Please provide optional arguments only once.\n");
                    return 0;
                }
                else
                {
                    Scierror(999, "Incorrect number of arguments provided. Please check the documentation for more information.\n");
                    return 0;
                }
            }
            else if(strcmp(currentArg, "ROI")==0)
            {
                if(iter+1 <= nbInputArguments && !providedArgs[2])
                {
                    sciErr = getVarAddressFromPosition(pvApiCtx, ++iter, &piAddr);
                    if (sciErr.iErr)
                    {
                        printError(&sciErr, 0);
                        return 0;
                    }
                    if(!isDoubleType(pvApiCtx, piAddr) || isVarComplex(pvApiCtx, piAddr))
                    {
                        Scierror(999, "%s: Wrong type for input argument #%d: A real matrix expected.\n", fname, iter);
                        return 0;
                    }
                    sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &iRows, &iCols, &matrixOfRoi);
                    if(sciErr.iErr)
                    {
                        printError(&sciErr, 0);
                        return 0;
                    }
                    if(iRows!=1 || iCols!=4)
                    {
                        Scierror(999, "Incorrect dimension of matrix for argument ROI.\n");
                        return 0;
                    }
                    providedArgs[2] = 1;
                    if(matrixOfRoi[0] < 0 || matrixOfRoi[0] > sourceImage.cols || matrixOfRoi[1] < 0 ||
                        matrixOfRoi[1] > sourceImage.rows || matrixOfRoi[2] < 0 || matrixOfRoi[0] + matrixOfRoi[2] > sourceImage.cols
                        || matrixOfRoi[3] < 0 || matrixOfRoi[1] + matrixOfRoi[3] > sourceImage.rows)
                    {
                        Scierror(999, "Error: Please provide proper values for \"%s\".\n", currentArg);
                        return 0;
                    }
                }
                else if(providedArgs[2])
                {
                    Scierror(999, "Please provide optional arguments only once.\n");
                    return 0;
                }
                else
                {
                    Scierror(999, "Incorrect number of arguments provided. Please check the documentation for more information.\n");
                    return 0;
                }
            }
            else if(strcmp(currentArg, "SensitivityFactor")==0)
            {
                if(iter+1 <= nbInputArguments && !providedArgs[3])
                {
                    sciErr = getVarAddressFromPosition(pvApiCtx, ++iter, &piAddr);
                    if (sciErr.iErr)
                    {
                        printError(&sciErr, 0);
                        return 0;
                    }
                    intErr = getScalarDouble(pvApiCtx, piAddr, &sensitivityFactor);
                    if(intErr)
                    {
                        return intErr;
                    }
                    providedArgs[3] = 1;
                    if(sensitivityFactor < 0 || sensitivityFactor > 0.25)
                    {
                        Scierror(999, "Error: Please provide proper value for \"%s\". Permissible values are [0, 0.25].\n", currentArg);
                        return 0;
                    }
                }
                else if(providedArgs[3])
                {
                    Scierror(999, "Please provide optional arguments only once.\n");
                    return 0;
                }
                else
                {
                    Scierror(999, "Incorrect number of arguments provided. Please check the documentation for more information.\n");
                    return 0;
                }
            }
            else
            {
                Scierror(999, "Error: \"%s\" name for input argument is not valid.\n", currentArg);
                return 0;
            }
        }

        //------Actual processing------//
        dst =  Mat::zeros( sourceImage.size(), CV_32FC1 );
        try 
        {   
            sourceImage.convertTo(sourceImage, CV_8U);
            //(Rect(matrixOfRoi[0], matrixOfRoi[1], matrixOfRoi[2], matrixOfRoi[3])
            cornerHarris(sourceImage, dst, blockSize1, filterSize1, sensitivityFactor1, BORDER_DEFAULT);
        }
        catch(Exception &e)
        {
            const char *err = e.what();
            Scierror(999, "%s", err);
            return 0;
        }
        
        pointCount = dst.rows * dst.cols;
       
        Mat dst_norm, dst_norm_scaled;
        vector<int> loc;
        normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
        convertScaleAbs( dst_norm, dst_norm_scaled );

        for( int j = 0; j < dst_norm.rows; j++ )
        { 
            for( int i = 0; i < dst_norm.cols; i++ )
            {
                if( (int) dst_norm.at<float>(j,i) > 200 )
                {
                    loc.push_back(i);
                    loc.push_back(j);
                }
            }
        }

        int size1 = loc.size() / 2;
        location = (double*) malloc(sizeof(double) * size1 * 2);
        metric = (double*) malloc(sizeof(double) * size1);
        for(int i = 0; i < size1; i++)
        {
            location[i] = loc[i];
            location[i + size1] = loc[i + 1];
            metric[i] = dst_norm.at<float>(loc[i + 1],loc[i]);
        }
        //------Create output arguments------//
        sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx)+1, size1, 2, location);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx)+2, size1, 1, metric);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        sciErr = createMatrixOfInteger32(pvApiCtx, nbInputArgument(pvApiCtx)+3, 1, 1, &pointCount);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }

        //------Return Arguments------//
        AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx)+1;
        AssignOutputVariable(pvApiCtx, 2) = nbInputArgument(pvApiCtx)+2;
        AssignOutputVariable(pvApiCtx, 3) = nbInputArgument(pvApiCtx)+3;
        ReturnArguments(pvApiCtx);
        return 0;
    }
/* ==================================================================== */
}
