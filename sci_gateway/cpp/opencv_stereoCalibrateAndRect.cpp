/***************************************************
Author : Siddhant Narang
***************************************************/

#include <numeric>
#include <opencv2/calib3d.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/core/types_c.h"
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

    double* readDoubleArrayFromScilab(int cnt)
    {
        SciErr sciErr;
        int *piAddr = NULL;
        double val = 0;
        int intErr = 0;
        int iRows = 0, iCols = 0;
        double *pdblReal;

        sciErr = getVarAddressFromPosition(pvApiCtx, cnt, &piAddr);
        if (sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }

        sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &iRows, &iCols, &pdblReal);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }

        if(isDoubleType(pvApiCtx, piAddr))
        {
            return pdblReal;
        }
        else
        {
            sciprint("Error: the %d argument is not of the type double!", cnt);
            return 0;
        }
    }


    int opencv_stereoCalibrateAndRect(char *fname, unsigned long fname_len)
    {
        /*----- Local variables -----*/
        int i = 0, j = 0, k = 0, n = 0, m = 0;
        int iRows = 0, iCols = 0;
        int *piAddr = NULL;
        double *pdblReal = NULL;
        double x, y, rms;
        SciErr sciErr;

        /*----- Processing Variables -----*/
        vector<std::vector< Point2f> >imagePoints1(1), imagePoints2(1);
        vector<std::vector< Point3f> >objectPoints(1);
    	Mat cameraMatrix1;
    	Mat cameraMatrix2;
        Mat distCoeffsActual1;
        Mat distCoeffsActual2;
        Mat rotationMatrix;
    	Mat translationVector;
        Mat fundamentalMatrix;
    	int num;
        double width = 0, height = 0;

         /*----- Check for input and output arguments -----*/
        CheckInputArgument(pvApiCtx, 4, 8);
        CheckOutputArgument(pvApiCtx, 9, 9);

        /*----- Parsing input -----*/
        n = *getNbInputArgument(pvApiCtx);
        
        // First
        sciErr = getVarAddressFromPosition(pvApiCtx, 1, &piAddr);
        if (sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &iRows, &iCols, &pdblReal);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }

        // Stores 
        for(i = 0; i < iRows; ++i)
        {
            objectPoints[0].push_back(Point3f(float(pdblReal[(0 * iRows) + i]),
                                              float(pdblReal[(1 * iRows) + i]),
                                              0.0));
        }
        int pointCount = iRows;

        // Second Input
        sciErr = getVarAddressFromPosition(pvApiCtx, 2, &piAddr);
        if (sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }

        if(!isListType(pvApiCtx, piAddr))
        {
            Scierror(999,"The imagePoints1 Argument must be a list of points.\n");
            return 0;
        }
        sciErr = getListItemNumber(pvApiCtx, piAddr, &num);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }

        // Get items from list
        for(int i = 1; i <= num; i++)
        {
        	sciErr = getMatrixOfDoubleInList(pvApiCtx, piAddr, i, &iRows, &iCols, &pdblReal);
        	
        	if(sciErr.iErr)
            {
                printError(&sciErr, 0);
                return 0;
            }
            
        	for(j = 0; j < iRows; ++j)
            {
                imagePoints1[i-1].push_back(Point2f(float(pdblReal[(0 * iRows) + j]), 
                                                    float(pdblReal[(1 * iRows) + j])));
            }
        }

        // Third input
        sciErr = getVarAddressFromPosition(pvApiCtx, 3, &piAddr);
        if (sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }

        if(!isListType(pvApiCtx, piAddr))
        {
            Scierror(999,"\nthe imagePoints2 Argument must be a list of points \n");
            return 0;
        }
        sciErr = getListItemNumber(pvApiCtx, piAddr, &num);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            
            return 0;
        }

        // Get items from list
        for(int i = 1; i <= num; i++)
        {
        	sciErr = getMatrixOfDoubleInList(pvApiCtx, piAddr,i,&iRows, &iCols, &pdblReal);
        	if(sciErr.iErr)
            {
                printError(&sciErr, 0);
                return 0;
            }

        	for(j = 0; j < iRows; ++j)
            {
                imagePoints2[i - 1].push_back(Point2f(float(pdblReal[(0 * iRows) + j]),
                                                      float(pdblReal[(1 * iRows) + j])));
            }
        }
        
        // Fourth
        sciErr = getVarAddressFromPosition(pvApiCtx, 4, &piAddr);
        if (sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &iRows, &iCols, &pdblReal);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        if(iCols > 2 && iRows > 1)
        {
            Scierror(999, "The image size should be of a grayscale image.\n");
            return 0;
        }

        sciprint("\nImage size obtained\n");
        width = pdblReal[0];
        height = pdblReal[1];

        /*----- Getting optional arguments -----*/
        int p;
        switch(n)
        {
        	case 5:
        	pdblReal = readDoubleArrayFromScilab(4);
        	for(i = 0; i < 3; i++)
            {
                for(j = 0; j < 3; j++)
                {
                    cameraMatrix1.at<double>(i, j) = pdblReal[(j * 3) + i];
                }
            }
        	break;

        	case 6:
    			pdblReal = readDoubleArrayFromScilab(4);
    			for(i = 0; i < 3; i++)
                {
            		for(j = 0; j < 3; j++)
                    {
                        cameraMatrix1.at<double>(i, j) = pdblReal[(j * 3) + i];
                    }
                }

                sciErr = getVarAddressFromPosition(pvApiCtx,5,&piAddr);
        		if (sciErr.iErr)
        		{
            		printError(&sciErr, 0);
            		return 0;
        		}
        		sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &iRows, &iCols, &pdblReal);
        		if(sciErr.iErr)
        		{
            		printError(&sciErr, 0);
            		return 0;
        		}

    		    if(iRows== 1)
                {
    		        p = iCols;
                }
    		    else if (iCols == 1)
                {
    		        p = iRows;
                }
    		    else
                {
    		        Scierror(1,"Distortion Points matrix (arg 5) must be a 1 X N or N X 1 matrix");
    		        return 0;
    		    }
    		    if(p == 4 or p == 5 or p == 8);
    		    else
                {
    		        Scierror(1," N must be 4 or 5 or 8");
    		        return 0;
    		    }
        	break;

        	case 7:
        		pdblReal = readDoubleArrayFromScilab(4);
    			for(i = 0; i < 3; i++)
                {
            		for(j = 0; j < 3; j++)
                    {
                        cameraMatrix1.at<double>(i, j) = pdblReal[(j * 3) + i];
                    }
                }

    			sciErr = getVarAddressFromPosition(pvApiCtx,5,&piAddr);
        		if (sciErr.iErr)
        		{
            		printError(&sciErr, 0);
            		return 0;
        		}
        		sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &iRows, &iCols, &pdblReal);
        		if(sciErr.iErr)
        		{
            		printError(&sciErr, 0);
            		return 0;
        		}

    		    if(iRows == 1)
                {
    		        p = iCols;
                }
    		    else if (iCols == 1)
                {
    		        p = iRows;
                }
    		    else
                {
    		        Scierror(1,"Distortion Points matrix (arg 5) must be a 1 X N or N X 1 matrix");
    		        return 0;
    		    }
    		    if(p == 4 or p == 5 or p == 8);
    		    else
                {
    		        Scierror(1," N must be 4 or 5 or 8");
    		        return 0;
    		    }
			    for(i = 0; i < p; i++){
			        distCoeffsActual1.at<double>(0, i) = pdblReal[i];
                }
                pdblReal=readDoubleArrayFromScilab(6);
            	for(i = 0; i < 3; i++)
                {
                    for(j = 0; j < 3; j++)
                    {
                        cameraMatrix2.at<double>(i,j) = pdblReal[(j * 3) + i];
                    }
                }
        	break;

        	case 8:
    			pdblReal = readDoubleArrayFromScilab(4);
    			for(i = 0; i < 3; i++)
                {
                    for(j = 0; j < 3; j++)
                    {
                        cameraMatrix1.at<double>(i, j) = pdblReal[(j * 3) + i];
                    }
                }

    			sciErr = getVarAddressFromPosition(pvApiCtx,5,&piAddr);
        		if (sciErr.iErr)
        		{
            		printError(&sciErr, 0);
            		return 0;
        		}
        		sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &iRows, &iCols, &pdblReal);
        		if(sciErr.iErr)
        		{
            		printError(&sciErr, 0);
            		return 0;
        		}

    		    if(iRows == 1)
                {
    		        p = iCols;
                }
    		    else if (iCols == 1)
                {
    		        p = iRows;
                }
    		    else
                {
    		        Scierror(1,"Distortion Points matrix (arg 5) must be a 1 X N or N X 1 matrix");
    		        return 0;
    		    }
    		    if(p == 4 or p == 5 or p == 8);
    		    else{
    		        Scierror(1," N must be 4 or 5 or 8");
    		        return 0;
    		    }
			    for(i = 0; i < p; i++)
                {
			        distCoeffsActual1.at<double>(0, i) = pdblReal[i];
                }

    	        pdblReal = readDoubleArrayFromScilab(6);
    	    	for(i = 0; i < 3; i++)
                {
    	           for(j = 0; j < 3; j++)
                   {
                        cameraMatrix2.at<double>(i,j) = pdblReal[(j * 3) + i];
                   }
                }
        	        sciErr = getVarAddressFromPosition(pvApiCtx,7,&piAddr);
        		if (sciErr.iErr)
        		{
            		printError(&sciErr, 0);
            		return 0;
        		}
        		sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &iRows, &iCols, &pdblReal);
        		if(sciErr.iErr)
        		{
            		printError(&sciErr, 0);
            		return 0;
        		}

    		    if(iRows == 1)
    		        p = iCols;
    		    else if (iCols == 1)
    		        p = iRows;
    		    else
                {
    		        Scierror(1,"Distortion Points matrix (arg 5) must be a 1 X N or N X 1 matrix");
    		        return 0;
    		    }
    		    if(p == 4 or p == 5 or p == 8);
    		    else
                {
    		        Scierror(1," N must be 4 or 5 or 8");
    		        return 0;
    		    }
    		    for(i = 0; i < p; i++)
                {
    		        distCoeffsActual2.at<double>(0, i) = pdblReal[i];
                }
        	break;
        }

        Size imageSize (width, height);
        Mat E;
        try
        {
            rms = stereoCalibrate(objectPoints, imagePoints1, imagePoints2,
                                  cameraMatrix1, distCoeffsActual1,
                                  cameraMatrix2, distCoeffsActual2,
                                  imageSize, rotationMatrix, 
                                  translationVector, E, fundamentalMatrix,
                                  CV_CALIB_SAME_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST,
                                  cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5)
                                );
        }    
        catch(cv::Exception& e)
        {
            sciprint("%s", e.what());
            return 0;
        }

        Mat R1, R2, P1, P2, Q;
        try
        {
            stereoRectify(cameraMatrix1, distCoeffsActual1,
                          cameraMatrix2, distCoeffsActual2, 
                          imageSize, rotationMatrix,
                          translationVector, R1, R2, P1, P2, Q, 
                          CALIB_ZERO_DISPARITY);
        }
        catch(cv::Exception& e)
        {
            sciprint("%s", e.what());
            return 0;
        }

        sciprint("Calibration done with RMS error=%f\n", rms);

        /*----- Return Arguments to Scilab -----*/
    	double *pstdata1 = NULL;
        double *pstdata2 = NULL;
        double *pstdata3 = NULL;
        double *pstdata4 = NULL;
        double *pstdata5 = NULL;
        double *pstdata6 = NULL;
        double *pstdata7 = NULL;
        double *pstdata8 = NULL;
        double *pstdata9 = NULL;

        pstdata1 = (double*)malloc(sizeof(double) * 3 * 3);

        for(i = 0; i < 3; i++)
        {
            for(j = 0; j < 3; j++)
            {
                pstdata1[(j * 3) + i] = cameraMatrix1.at<double>(i, j); 
            }
        }

        sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 1, 3, 3, pstdata1);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }

        pstdata2 = (double*)malloc(sizeof(double) * 4 * 1);
        for(i = 0; i < 4; i++)
        {
            pstdata2[i] = distCoeffsActual1.at<double>(0, i);
        }
        sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 2, 1, 4, pstdata2);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }

        pstdata3 = (double*)malloc(sizeof(double) * 3 * 3);

        for(i = 0; i < 3; i++)
        {
            for(j = 0; j < 3; j++)
            {
                pstdata3[(j * 3) + i] = cameraMatrix2.at<double>(i,j); 
            }
        }

        sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 3, 3, 3, pstdata3);
        if(sciErr.iErr){
          printError(&sciErr, 0);
          return 0;
        }

        pstdata4 = (double*)malloc(sizeof(double) * 4 * 1);
        for(i = 0; i < 4; i++)
        {
            pstdata4[i] = distCoeffsActual2.at<double>(0, i);
        }

        sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 4, 1, 4, pstdata4);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }

        pstdata5 = (double*)malloc(sizeof(double) *rotationMatrix.rows*rotationMatrix.cols);
        for(i = 0; i < rotationMatrix.rows; i++)
        {
        	for(j = 0; j < rotationMatrix.cols; j++)
        	{
                pstdata5[(j * rotationMatrix.rows) + i] = rotationMatrix.at<double>(i, j); 
        	}
        }

        sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 5, rotationMatrix.rows, rotationMatrix.cols, pstdata5);
        if(sciErr.iErr){
          printError(&sciErr, 0);
          return 0;
        }

        pstdata6 = (double*)malloc(sizeof(double) * translationVector.rows * translationVector.cols);

        for(i = 0; i < translationVector.rows; i++)
        {
        	for(j = 0; j < translationVector.cols; j++)
        	{
                pstdata6[(j * translationVector.rows) + i] = translationVector.at<double>(i, j); 
        	}
        }

        sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 6, translationVector.rows, translationVector.cols, pstdata6);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }

        pstdata7 = (double*)malloc(sizeof(double) * Q.rows * Q.cols);
        for(i = 0; i < Q.rows; i++)
        {
            for(j = 0; j < Q.cols; j++)
            {
                pstdata7[(j * Q.rows) + i] = Q.at<double>(i, j); 
            }
        }

        sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 7, Q.rows, Q.cols, pstdata7);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }

        pstdata8 = (double*)malloc(sizeof(double) * P1.rows * P1.cols);
        for(i = 0; i < P1.rows; i++)
        {
            for(j = 0; j < P1.cols; j++)
            {
                pstdata8[(j * P1.rows) + i] = P1.at<double>(i, j); 
            }
        }

        sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 8, P1.rows, P1.cols, pstdata8);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }

        pstdata9 = (double*)malloc(sizeof(double) * P2.rows * P2.cols);
        for(i = 0; i < P2.rows; i++)
        {
            for(j = 0; j < P2.cols; j++)
            {
                pstdata9[(j * P2.rows) + i] = P2.at<double>(i, j); 
            }
        }

        sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 9, P2.rows, P2.cols, pstdata9);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }

        for(int i = 1; i <= 9; i++)
        {
            AssignOutputVariable(pvApiCtx, i) = nbInputArgument(pvApiCtx) + i;
        }
        
        /*----- Returning the Output Variables as arguments to the Scilab environment -----*/
        ReturnArguments(pvApiCtx);
        return 0;       
	}
}
