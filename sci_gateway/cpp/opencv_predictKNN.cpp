/***************************************************
Author : Siddhant Narang
***************************************************/
#include <iostream>
#include <string.h>
#include <sys/stat.h>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/ml.hpp>
#include <opencv2/core.hpp>


using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;
using namespace cv::ml;


inline bool file_exists_check(const std::string& name) 
{
    /*----- Checks if the input file is in the mentioned wd -----*/
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
    int opencv_predictKNN(char *fname, unsigned long fname_len)
    {
        // Error management variables
        SciErr sciErr;

        //------Local variables------//
        int upright = 1;
        Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("FlannBased");
        Ptr<SURF> detector = SURF::create(400, 4, 2, 1, upright);
        Ptr<DescriptorExtractor> extractor = detector;
        Ptr<BOWImgDescriptorExtractor> bowDE = makePtr<BOWImgDescriptorExtractor>(extractor, matcher);
        //String *classifierLocation = NULL;
        Mat dictionary,features;
        Mat response;
        std::vector<KeyPoint> keyPoints;
        int k = 4;
	    int iRet;
        int dictionarySize;
        int *piAddr = NULL;
        int *piChild = NULL;
        int iRows, iCols;
        char **pstData = NULL;
        int *piLen = NULL;
        char **classifierDescription = NULL;
        int classifierDescriptionCount;
        char *bagOfFeaturesLocation = NULL;
        int descriptionCount;
        Mat input;

        //------Check number of parameters------//
        CheckInputArgument(pvApiCtx, 3, 3);
        CheckOutputArgument(pvApiCtx, 1, 1);
	
        //------Get input arguments------//
        retrieveImage(input, 2);
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

        if(!(strcmp(pstData[0],"KNNClassifier")))
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
	    //*classifierLocation = (String*) malloc(100 * sizeof(String));
        String classifierLocation = String(pstData[0]);
	
        sciErr = getMatrixOfStringInList(pvApiCtx, piAddr, 3, &iRows, &iCols, NULL, NULL);
        if (sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }

        piLen = (int*) malloc(sizeof(int) * iRows * iCols);

        sciErr = getMatrixOfStringInList(pvApiCtx,  piAddr, 3,  &iRows,  &iCols,  piLen,  NULL);
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

        sciErr = getMatrixOfStringInList(pvApiCtx, piAddr, 3, &iRows, &iCols, piLen, pstData);
        if (sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }

        if(iRows!=1 || iCols!=1)
        {
            Scierror(999, "1x1 Matrix expected for bagOfFeatures argument.");
            return 0;
        }
        bagOfFeaturesLocation = pstData[0];
        sciErr = getMatrixOfStringInList(pvApiCtx, piAddr, 4, &iRows, &iCols, NULL, NULL);
        if (sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }

        piLen = (int*) malloc(sizeof(int) * iRows * iCols);

        sciErr = getMatrixOfStringInList(pvApiCtx,  piAddr, 4,  &iRows,  &iCols,  piLen,  NULL);
        if (sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }

        classifierDescription = (char**) malloc(sizeof(char*) * iRows * iCols);
        for(int iterPstData = 0; iterPstData < iRows * iCols; iterPstData++)
        {
            classifierDescription[iterPstData] = (char*) malloc(sizeof(char) * piLen[iterPstData] + 1);
        }

        sciErr = getMatrixOfStringInList(pvApiCtx, piAddr, 4, &iRows, &iCols, piLen, classifierDescription);
        if (sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        // Handling the argument 3.
        sciErr = getVarAddressFromPosition(pvApiCtx, 3, &piAddr);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        if(isIntegerType(pvApiCtx, piAddr)||isDoubleType(pvApiCtx, piAddr))
        {       
            if(isScalar(pvApiCtx, piAddr))
            {
                double dData  = 0;
                iRet = getScalarDouble(pvApiCtx, piAddr, &dData);
                if(!iRet)
                {
                    k = (int)dData;
                }
            }
            else
            {
                Scierror(999, "Error: The input argument #3 is not of scalar type.\n");
                return 0;
            }
        }
        else 
        {
            Scierror(999, "Error: The input argument #3 is not of type Integer.\n");
            return 0;
        }

        // Handling missing yml file
        if(!file_exists_check(classifierLocation))
        {
            Scierror(999, "Error: Input "".yml"" File not found in the pwd.\n");
            return 0;
        }

        // sciprint("arguments done");
        //------Actual processing------//
    	sciprint("\n%s\n", bagOfFeaturesLocation);
        FileStorage fs(bagOfFeaturesLocation, FileStorage::READ);
        fs["dictionary"] >> dictionary;
        fs.release();
        dictionarySize = dictionary.rows;
    	try
    	{
    	        bowDE->setVocabulary(dictionary);
    	}
    	catch(Exception &e)
    	{
    		const char *err = e.what();
            sciprint("%s", err);
            return 0;
    	}
    	// sciprint("\n%d  %d\n", input.rows, input.cols);
    	// Convert input image to and detect keypoints.
    	try
    	{
    		input.convertTo(input, CV_8U);
            detector->detect(input, keyPoints);
    	}
    	catch(Exception &e)
    	{
    		const char *err = e.what();
            sciprint("%s", err);
            return 0;
    	}
    	// Compute features.
    	try
    	{
            bowDE->compute(input, keyPoints, features);
    	}
    	catch(Exception &e)
    	{
    		const char *err = e.what();
            sciprint("%s", err);
            return 0;
    	}
	    // Load and train classifier.
        try
        {
	        Ptr<KNearest> knearest = Algorithm::load<KNearest>(classifierLocation);
            if(!knearest->isTrained())
            {
                Scierror(999, "Error: the model is not trained!.\n");
                return 0;
            } 
            features.convertTo(features, CV_32F);
            knearest->setIsClassifier(1);
            knearest->findNearest(features, k, response);
        }   
        catch(cv::Exception &e)
        {
            const char *err = e.what();
            sciprint("%s", err);
            return 0;
        }

        //------Create output arguments------//
        sciErr = createList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, 1, &piAddr);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        sciErr = createMatrixOfStringInList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, piAddr, 1, 1, 1, &classifierDescription[(int)response.at<double>(0, 0)]);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        //------Return Arguments------//
        AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
        ReturnArguments(pvApiCtx);
        return 0;
    }
    /* ==================================================================== */
}
