/***************************************************
Author : Siddhant Narang
***************************************************/
#include <iostream>
#include <bits/stdc++.h>
#include <sys/stat.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/ml.hpp"
#include "opencv2/opencv.hpp"


using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;
using namespace cv::ml;


inline bool file_exists_check(const std::string& name) 
{
    struct stat buffer;   
    return (stat (name.c_str(), &buffer) == 0); 
}


extern "C"
{
    #include "api_scilab.h"
    #include "Scierror.h"
    #include "BOOL.h"
    #include <localization.h>
    #include "sciprint.h"
    #include "../common.h"
    int opencv_getParamsEM(char *fname, unsigned long fname_len)
    {
        SciErr sciErr;
    	int *piAddr = NULL;
        int *piChild = NULL;
        int iRows, iCols;
        char **pstData = NULL;
        int *piLen = NULL;
        char **classifierDescription = NULL;
        int classifierDescriptionCount;

        double nClusters = 0;
        Mat means;
        Mat weights;
        double *_means;
        double *_weights;

        /*------ Check number of parameters ------*/
        CheckInputArgument(pvApiCtx, 1, 1);
        CheckOutputArgument(pvApiCtx, 1, 1);

        /*------ Get input arguments ------*/
        sciErr = getVarAddressFromPosition(pvApiCtx, 1, &piAddr);
        if (sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        if(!isListType(pvApiCtx, piAddr))
        {
            Scierror(999, "Error: The input argument #1 is not of type classifier.\n");
            return 0;
        }

        /*----- Extracting object type and checking if type is classifier -----*/
        sciErr = getMatrixOfStringInList(pvApiCtx, piAddr, 1, &iRows, &iCols, NULL, NULL);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }

        piLen = (int*)malloc(sizeof(int) * iRows * iCols);

        sciErr = getMatrixOfStringInList(pvApiCtx, piAddr, 1, &iRows, &iCols, piLen, NULL);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }

        pstData = (char**)malloc(sizeof(char*) * iRows * iCols);

        for(int iter = 0 ; iter < iRows * iCols ; iter++)
        {
            pstData[iter] = (char*)malloc(sizeof(char) * (piLen[iter] + 1));//+ 1 for null termination
        }

        sciErr = getMatrixOfStringInList(pvApiCtx, piAddr, 1, &iRows, &iCols, piLen, pstData);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }

        if(!(strcmp(pstData[0],"EMClassifier")))
        {
            Scierror(999, "Error: The input argument #1 is not of type classifier.\n");
            return 0;
        }

        /*----- Extracting classifier location from classifier object -----*/
        sciErr = getMatrixOfStringInList(pvApiCtx, piAddr, 2, &iRows, &iCols, NULL, NULL);
        if (sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }

        piLen = (int*) malloc(sizeof(int) * iRows * iCols);

        sciErr = getMatrixOfStringInList(pvApiCtx,  piAddr, 2,  &iRows,  &iCols,  piLen,  NULL);
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

        sciErr = getMatrixOfStringInList(pvApiCtx, piAddr, 2, &iRows, &iCols, piLen, pstData);
        if (sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }

        if(iRows!=1 || iCols!=1)
        {
            Scierror(999, "1x1 Matrix expected for classifier argument.");
            return 0;
        }
        string classifierLocation = string(pstData[0]);
        
        if(!file_exists_check(classifierLocation))
        {
            Scierror(999, "Error: Input "".yml"" File not found in the pwd.\n");
            return 0;
        }

        Ptr<EM> em = Algorithm::load<EM>(classifierLocation);

	    try
	    {
	        if(!em->isTrained())
	        {
	            Scierror(999,"\nEM Model not trained\n");
	            return 0;
	        }
	        nClusters = em->getClustersNumber();
            
            means = em->getMeans();
            // Converting from type matrix to double
            _means = (double*) malloc(means.cols * sizeof(double));
            for(int i = 0; i < means.cols; i++)
            {
                _means[i] = means.at<double>(i);
            }

            weights = em->getWeights();
            // Converting from type matrix to double
            _weights = (double*) malloc(weights.cols * sizeof(double));
            for(int i = 0; i < weights.cols; i++)
            {
                _weights[i] = weights.at<double>(i);
            }
	    }
	    catch(Exception &e)
	    {
	    	const char *err = e.what();
			sciprint("%s", err);
	    }

        /*----- Creating output arguments -----*/
	    sciErr = createList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, 6, &piAddr);
	    if(sciErr.iErr)
	    {
	        printError(&sciErr, 0);
	        return 0;
	    }
        sciErr = createMatrixOfDoubleInList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, piAddr, 1, 1, 1, &nClusters);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        sciErr = createMatrixOfDoubleInList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, piAddr, 2, 1, means.cols, _means);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        sciErr = createMatrixOfDoubleInList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, piAddr, 3, 1, weights.cols, _weights);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        /*------ Return Arguments ------*/
        AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx)+1;
        ReturnArguments(pvApiCtx);
        return 0;
	}	
}