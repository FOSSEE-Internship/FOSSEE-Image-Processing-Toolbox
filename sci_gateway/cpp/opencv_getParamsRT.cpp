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
#include <sys/stat.h>

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
    int opencv_getParamsRT(char *fname, unsigned long fname_len)
    {
        SciErr sciErr;
    	int *piAddr = NULL;
        int *piChild = NULL;
        int iRows, iCols;
        char **pstData = NULL;
        int *piLen = NULL;
        char **classifierDescription = NULL;
        int classifierDescriptionCount;

        double vrcount = 0;
        Mat vrimp;
        //Mat weights;
        double *_vrimp=NULL;
	bool isVar=0;

        double cvfolds;
        double maxCat ;
        double maxDepth ;
        double minSmp ;
        double regacc ;
        int isPruned ;
        int isUse1SE ;
        int UseSurrogates;


        //------Check number of parameters------//
        CheckInputArgument(pvApiCtx, 1, 1);
        CheckOutputArgument(pvApiCtx, 1, 1);

        //------Get input arguments------//
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

        // Extracting object type and checking if type is classifier
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

        if(!(strcmp(pstData[0],"RTclassifier")==0))
        {
            Scierror(999, "Error: The input argument #1 is not of type classifier.\n");
            return 0;
        }
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

	    try
	    {
	        Ptr<RTrees> rt = Algorithm::load<RTrees>(classifierLocation);
	        if(!rt->isTrained())
	        {
	            Scierror(999,"\nEM Model not trained\n");
	            return 0;
	        }
	        vrcount = rt->getActiveVarCount();
            	//sciprint("here\n");

		isVar=rt->getCalculateVarImportance();
	

            cvfolds = rt->getCVFolds();
            maxCat = rt->getMaxCategories();
            maxDepth = rt->getMaxDepth();
            minSmp = rt->getMinSampleCount();
            regacc = rt->getRegressionAccuracy();
            isPruned= rt->getTruncatePrunedTree();
            isUse1SE=rt->getUse1SERule() ;

            UseSurrogates=rt->getUseSurrogates();

		//sciprint("here3\n");	

            /*weights = rt->getWeights();
            _weights = (double*) malloc(weights.cols * sizeof(double));
            for(int i = 0; i < weights.cols; i++)
            {
                _weights[i] = weights.at<double>(i);
            }*/
	    }
	    catch(Exception &e)
	    {
	    	const char *err = e.what();
			sciprint("%s", err);
	    }
	
	    sciErr = createList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, 9, &piAddr);
	    if(sciErr.iErr)
	    {
	        printError(&sciErr, 0);
	        return 0;
	    }
	
        sciErr = createMatrixOfDoubleInList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, piAddr, 1, 1, 1, &vrcount);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        sciErr = createMatrixOfDoubleInList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, piAddr, 2, 1, 1,&cvfolds);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        sciErr = createMatrixOfDoubleInList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, piAddr, 3, 1, 1,&maxCat);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        sciErr = createMatrixOfDoubleInList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, piAddr,4, 1, 1,&maxDepth);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        sciErr = createMatrixOfDoubleInList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, piAddr, 5, 1, 1,&minSmp);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        sciErr = createMatrixOfDoubleInList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, piAddr, 6, 1, 1,&regacc);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        sciErr = createMatrixOfBooleanInList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, piAddr, 7, 1, 1,&isPruned);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        sciErr = createMatrixOfBooleanInList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, piAddr, 8, 1, 1,&UseSurrogates);

        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }

        sciErr = createMatrixOfBooleanInList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, piAddr, 9, 1, 1,&isUse1SE);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }

	
        //------Return Arguments------//
        AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx)+1;
        ReturnArguments(pvApiCtx);
        return 0;
	}	
}
