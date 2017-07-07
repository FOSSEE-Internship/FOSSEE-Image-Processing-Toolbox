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
    int opencv_getParamsLR(char *fname, unsigned long fname_len)
    {
        SciErr sciErr;
    	int *piAddr = NULL;
        int *piChild = NULL;
        int iRows, iCols;
        char **pstData = NULL;
        int *piLen = NULL;
        char **classifierDescription = NULL;
        int classifierDescriptionCount;

        double iterations = 0;
        double learningRate = 0;
        double miniBatchSize = 0;
        char *regularization = NULL;
        char *trainMethod = NULL;
        Mat glm;
        double *_glm;

        /*------ Check number of parameters ------*/
        CheckInputArgument(pvApiCtx, 1, 1);
        CheckOutputArgument(pvApiCtx, 1, 1);

        /* ------Get input arguments------ */
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

        if(!(strcmp(pstData[0],"LRClassifier")))
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

        /*----- Check if .yml file is in pwd -----*/
        if(!file_exists_check(classifierLocation))
        {
            Scierror(999, "Error: Input "".yml"" File not found in the pwd.\n");
            return 0;
        }

        /*------ Processing steps -----*/
	    try
	    {
	        Ptr<LogisticRegression> lr = Algorithm::load<LogisticRegression>(classifierLocation);
	        if(!lr->isTrained())
	        {
	            Scierror(999,"\nLR Model not trained\n");
	            return 0;
	        }
	        iterations = lr->getIterations();
            learningRate = lr->getLearningRate();

            int reg = lr->getRegularization();
            if(reg == -1)
            {
                regularization = "REG_DISABLED";
            }
            else if(reg == 0)
            {
                regularization = "REG_L1";
            }
            else
            {
                regularization = "REG_L2";
            }   
            miniBatchSize = lr->getMiniBatchSize();

            int tm = lr->getTrainMethod();
            if(tm == 0)
            {
                trainMethod = "BATCH";
            }
            else
            {
                trainMethod = "MINI_BATCH";
            }

            glm = lr->get_learnt_thetas();

            // Converting learnt thetas matrix from type matrix to double.
            _glm = (double*) malloc(glm.cols * sizeof(double));
            for(int i = 0; i < glm.cols; i++)
            {
                _glm[i] = glm.at<double>(i);
            }
	    }
	    catch(Exception &e)
	    {
	    	const char *err = e.what();
			sciprint("%s", err);
	    }

        /*----- Creating Output Arguments ------*/
	    sciErr = createList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, 6, &piAddr);
	    if(sciErr.iErr)
	    {
	        printError(&sciErr, 0);
	        return 0;
	    }
        sciErr = createMatrixOfDoubleInList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, piAddr, 1, 1, 1, &iterations);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        sciErr = createMatrixOfDoubleInList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, piAddr, 2, 1, 1, &learningRate);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        sciErr = createMatrixOfDoubleInList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, piAddr, 3, 1, 1, &miniBatchSize);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
	    sciErr = createMatrixOfStringInList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, piAddr, 4, 1, 1, &regularization);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
        sciErr = createMatrixOfStringInList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, piAddr, 5, 1, 1, &trainMethod);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        sciErr = createMatrixOfDoubleInList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, piAddr, 6, 1, glm.cols, _glm);
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