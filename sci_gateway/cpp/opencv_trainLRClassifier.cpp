/***************************************************
Author : Siddhant Narang
***************************************************/
#include <iostream>
#include <sys/stat.h>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/ml.hpp>


inline bool file_exists_check(const std::string& name) 
{
    struct stat buffer;   
    return (stat (name.c_str(), &buffer) == 0); 
}


using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;
using namespace cv::ml;

extern "C"
{
    #include "api_scilab.h"
    #include "Scierror.h"
    #include "BOOL.h"
    #include <localization.h>
    #include "sciprint.h"


    int opencv_trainLRClassifier(char *fname, unsigned long fname_len)
    {
        // Error management variables
        SciErr sciErr;

        //------Local variables------//
        int upright = 1;
        Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("FlannBased");
        Ptr<SURF> detector = SURF::create(400, 4, 2, 1, upright);
        Ptr<DescriptorExtractor> extractor = detector;
        Ptr<BOWImgDescriptorExtractor> bowDE = makePtr<BOWImgDescriptorExtractor>(extractor, matcher);
        char *fileName = NULL;
        Mat dictionary,inp,features;
        vector<KeyPoint> keyPoints;

        int iRet = 0;
		int iPrec = 0;
		int iBool = 0;
        int *piAddr = NULL;
        int *piChild = NULL;
        int iRows, iCols;
        char **pstData = NULL;
        int *piLen = NULL;
        int *count = NULL;
        char **description = NULL;
        char ***location = NULL;
        char *bagOfFeaturesLocation = NULL;
        int descriptionCount;
		
        double learningRate = 1.0;
        double iteration = 100;
        double regularization = 1;
        double trainMethod = 0;
        double miniBatchSize = 10;

        char *classifierName = NULL;
        char *classifierLocation = NULL;
        char *objectType = "LRClassifier";
        //------Check number of parameters------//
        CheckInputArgument(pvApiCtx, 2, 8);
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
            Scierror(999, "Error: The input argument #1 is not of type imageSet.\n");
            return 0;
        }

        // Extracting object type and checking if type is imageSet
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
            pstData[iter] = (char*)malloc(sizeof(char) * (piLen[iter] + 1));  // + 1 for null termination
        }

        sciErr = getMatrixOfStringInList(pvApiCtx, piAddr, 1, &iRows, &iCols, piLen, pstData);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }

        if(!(strcmp(pstData[0],"imageSet")==0))
        {
            Scierror(999, "Error: The input argument #1 is not of type imageSet.\n");
            return 0;
        }

        // Extracting Description attribute of input argument
        sciErr = getMatrixOfStringInList(pvApiCtx, piAddr, 2, &iRows, &iCols, NULL, NULL);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }

        piLen = (int*)malloc(sizeof(int) * iRows * iCols);

        sciErr = getMatrixOfStringInList(pvApiCtx, piAddr, 2, &iRows, &iCols, piLen, NULL);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }

        description = (char**)malloc(sizeof(char*) * iRows * iCols);

        for(int iter = 0 ; iter < iRows * iCols ; iter++)
        {
            description[iter] = (char*)malloc(sizeof(char) * (piLen[iter] + 1));//+ 1 for null termination
        }

        sciErr = getMatrixOfStringInList(pvApiCtx, piAddr, 2, &iRows, &iCols, piLen, description);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        descriptionCount = iRows;

        // Extracting Count attribute of input argument
        sciErr = getMatrixOfInteger32InList(pvApiCtx, piAddr, 3, &iRows, &iCols, &count);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }

        location = (char***) malloc(sizeof(char**) * descriptionCount);

        // Extracting Location of images from input argument.
        sciErr = getListItemAddress(pvApiCtx, piAddr, 4, &piChild);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        for(int iter = 1; iter<=descriptionCount; iter++)
        {
            sciErr = getMatrixOfStringInList(pvApiCtx, piChild, iter, &iRows, &iCols, NULL, NULL);
            if(sciErr.iErr)
            {
                printError(&sciErr, 0);
                return 0;
            }

            piLen = (int*)malloc(sizeof(int) * iRows * iCols);

            sciErr = getMatrixOfStringInList(pvApiCtx, piChild, iter, &iRows, &iCols, piLen, NULL);
            if(sciErr.iErr)
            {
                printError(&sciErr, 0);
                return 0;
            }

            location[iter-1] = (char**)malloc(sizeof(char*) * iRows * iCols);

            for(int colIter = 0 ; colIter < iRows * iCols ; colIter++)
            {
                location[iter-1][colIter] = (char*)malloc(sizeof(char) * (piLen[colIter] + 1));//+ 1 for null termination
            }

            sciErr = getMatrixOfStringInList(pvApiCtx, piChild, iter, &iRows, &iCols, piLen, location[iter-1]);
            if(sciErr.iErr)
            {
                printError(&sciErr, 0);
                return 0;
            }
        }

        // Check is imageSet is in pwd.
        if(!file_exists_check(location[0][1]))
        {
            Scierror(999, "Error: Image set not found in the pwd.\n");
            return 0;
        }

        // Second argument
        sciErr = getVarAddressFromPosition(pvApiCtx, 2, &piAddr);
        if (sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        if(!isListType(pvApiCtx, piAddr))
        {
            Scierror(999, "Error: The input argument #2 is not of type bagOfFeatures.\n");
            return 0;
        }

        // Extracting object type and checking if type is bagOfFeatures
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
            pstData[iter] = (char*)malloc(sizeof(char) * (piLen[iter] + 1));  //+ 1 for null termination
        }

        sciErr = getMatrixOfStringInList(pvApiCtx, piAddr, 1, &iRows, &iCols, piLen, pstData);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }

        if(!(strcmp(pstData[0],"bagOfFeatures")==0))
        {
            Scierror(999, "Error: The input argument #2 is not of type bagOfFeatures.\n");
            return 0;
        }
        // Extracting name of next argument takes three calls to getMatrixOfString
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
            Scierror(999, "1x1 Matrix expected for bagOfFeatures argument.");
            return 0;
        }
        bagOfFeaturesLocation = pstData[0];

        // Check is bagOfFeatures.yml is in pwd.
        if(!file_exists_check(bagOfFeaturesLocation))
        {
            Scierror(999, "Error: Bag of Features "".yml"" file not found in the pwd.\n");
            return 0;
        }
        
        // Handling 3rd argument.
        sciErr = getVarAddressFromPosition(pvApiCtx, 3, &piAddr);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        if(isStringType(pvApiCtx, piAddr))
        {
            if(isScalar(pvApiCtx, piAddr))
            {
                char* stringData = NULL;

                iRet = getAllocatedSingleString(pvApiCtx, piAddr, &stringData);
                if(!iRet)
                {
                    classifierName = stringData;
                }
                else
                {
                    Scierror(999, "Error: Could not assign input arg #3 to local var.\n");
                    return 0;
                }
            }
            else
            {
                Scierror(999, "Error: The input argument #3 is not scalar type.\n");
                return 0;
            }
        }
        else 
        {
            Scierror(999, "Error: The input argument #3 is not a string.\n");
            return 0;
        }

        // Handling more than 3 arguments.
		if(*getNbInputArgument(pvApiCtx) > 3)
		{
			sciErr = getVarAddressFromPosition(pvApiCtx, 4, &piAddr);
			if(sciErr.iErr)
			{
				printError(&sciErr, 0);
				return 0;
			}
			if(isDoubleType(pvApiCtx, piAddr))
			{
					
				if(isScalar(pvApiCtx, piAddr))
				{
					double dData  = 0;
					iRet = getScalarDouble(pvApiCtx, piAddr, &dData);
					if(!iRet)
					{
						learningRate = dData;
					}
				}
				else
				{
					Scierror(999, "Error: The input argument #4 is not of scalar type.\n");
	        		return 0;
				}
			}
			else 
			{
				Scierror(999, "Error: The input argument #4 is not of type Double.\n");
	        	return 0;
			}
		}

        // Handling more than 4 arguments.
        if(*getNbInputArgument(pvApiCtx) > 4)
        {
            sciErr = getVarAddressFromPosition(pvApiCtx, 5, &piAddr);
            if(sciErr.iErr)
            {
                printError(&sciErr, 0);
                return 0;
            }
            if(isIntegerType(pvApiCtx, piAddr) || isDoubleType(pvApiCtx, piAddr))
            {
                    
                if(isScalar(pvApiCtx, piAddr))
                {
                    double dData  = 0;
                    iRet = getScalarDouble(pvApiCtx, piAddr, &dData);
                    if(!iRet)
                    {
                        iteration = dData;
                    }
                }
                else
                {
                    Scierror(999, "Error: The input argument #5 is not of scalar type.\n");
                    return 0;
                }
            }
            else 
            {
                Scierror(999, "Error: The input argument #5 is not of type Integer.\n");
                return 0;
            }
        }

        // Handling more than 5 arguments.
        if(*getNbInputArgument(pvApiCtx) > 5)
        {
            sciErr = getVarAddressFromPosition(pvApiCtx, 6, &piAddr);
            if(sciErr.iErr)
            {
                printError(&sciErr, 0);
                return 0;
            }
            if(isIntegerType(pvApiCtx, piAddr) || isDoubleType(pvApiCtx, piAddr))
            {
                if(isScalar(pvApiCtx, piAddr))
                {
                    double dData  = 0;
                    iRet = getScalarDouble(pvApiCtx, piAddr, &dData);
                    if(!iRet)
                    {   
                        if(dData < -1 || dData > 1)
                        {
                            Scierror(999, "Error: The input argument ""regularization"" should be in range [-1, 1].\n");
                            return 0;
                        }
                        
                        regularization = dData;
                    }
                }
                else
                {
                    Scierror(999, "Error: The input argument #6 is not of scalar type.\n");
                    return 0;
                }
            }
            else 
            {
                Scierror(999, "Error: The input argument #6 is not of type Integer.\n");
                return 0;
            }
        }

        // Handling more than 6 arguments.
        if(*getNbInputArgument(pvApiCtx) > 6)
        {
            sciErr = getVarAddressFromPosition(pvApiCtx, 7, &piAddr);
            if(sciErr.iErr)
            {
                printError(&sciErr, 0);
                return 0;
            }
            if(isIntegerType(pvApiCtx, piAddr) || isDoubleType(pvApiCtx, piAddr))
            {
                    
                if(isScalar(pvApiCtx, piAddr))
                {
                    double dData  = 0;
                    iRet = getScalarDouble(pvApiCtx, piAddr, &dData);
                    if(!iRet)
                    {   
                        if(!(dData >= 0 && dData <= 1))
                        {
                            Scierror(999, "Error: The input argument ""trainMethod"" should be in range [0, 1].\n");
                            return 0;
                        }
                        trainMethod = dData;
                    }
                }
                else
                {
                    Scierror(999, "Error: The input argument #7 is not of scalar type.\n");
                    return 0;
                }
            }
            else 
            {
                Scierror(999, "Error: The input argument #7 is not of type Integer.\n");
                return 0;
            }
        }

        // Handling more than 7 arguments.
        if(*getNbInputArgument(pvApiCtx) > 7)
        {
            sciErr = getVarAddressFromPosition(pvApiCtx, 8, &piAddr);
            if(sciErr.iErr)
            {
                printError(&sciErr, 0);
                return 0;
            }
            if(isIntegerType(pvApiCtx, piAddr) || isDoubleType(pvApiCtx, piAddr))
            {
                    
                if(isScalar(pvApiCtx, piAddr))
                {
                    double dData  = 0;
                    iRet = getScalarDouble(pvApiCtx, piAddr, &dData);
                    if(!iRet)
                    {
                        miniBatchSize = dData;
                    }
                }
                else
                {
                    Scierror(999, "Error: The input argument #8 is not of scalar type.\n");
                    return 0;
                }
            }
            else 
            {
                Scierror(999, "Error: The input argument #8 is not of type Integer.\n");
                return 0;
            }
        }

        classifierLocation = classifierName;
        strcat(classifierLocation, ".yml");

        //------Actual processing------//
        FileStorage fs(bagOfFeaturesLocation, FileStorage::READ);
        fs["dictionary"] >> dictionary;
        fs.release();
        if(dictionary.rows==0 || dictionary.cols==0)
        {
            sciprint("Error: The provided file for bagOfFeatures may be invalid.\n");
        }
        sciprint("Training an image category classifier for %d classes.\n",descriptionCount);
        sciprint("-------------------------------------------------------\n\n");
        for(int i=0;i<descriptionCount;i++)
        {
            sciprint("# Class %d: %s\n",i+1,description[i]);
        }
        sciprint("\n");
        int dictionarySize = dictionary.rows;
        Mat labels(0, 1, CV_32FC1);
        Mat trainingData(0, dictionarySize, CV_32FC1);
        bowDE->setVocabulary(dictionary);
        for(int i=0; i<descriptionCount;i++)
        {
            sciprint("# Encoding features for Class %d ...",i+1);
            for(int j=0; j<count[i]; j++)
            {
                features.release();
                keyPoints.clear();
                fileName = location[i][j];
                inp = imread(fileName);
                try
                {
                    detector->detect(inp,keyPoints);
                }
                catch(cv::Exception &e)
                {
                    const char *err = e.what();
                    sciprint("%s", err);
                }
                try
                {
                    bowDE->compute(inp, keyPoints,features);
                }
                catch(cv::Exception &e)
                {
                    const char *err = e.what();
                    sciprint("%s", err);
                }
                trainingData.push_back(features);
                labels.push_back((float) i);
            }
            sciprint("done.\n");
        }
        sciprint("\n# Training the Logistic Regression classifier...");

        Ptr<LogisticRegression> p = LogisticRegression::create();
        Ptr<TrainData> tdata = TrainData::create(trainingData, ml::ROW_SAMPLE, labels);

        try
        {
        	p->setLearningRate(learningRate);	
        }
        catch(cv::Exception &e)
        {
        	const char *err = e.what();
        	sciprint("%s", err);
        }

        try
        {
            p->setIterations(iteration);   
        }
        catch(cv::Exception &e)
        {
            const char *err = e.what();
            sciprint("%s", err);
        }

        try
        {
            p->setRegularization(regularization); 
        }
        catch(cv::Exception &e)
        {
            const char *err = e.what();
            sciprint("%s", err);
        }

        try
        {
            p->setTrainMethod(trainMethod);
        }
        catch(cv::Exception &e)
        {
            const char *err = e.what();
            sciprint("%s", err);
        }

        try
        {
             if(trainMethod == 1)
            {
                p->setMiniBatchSize(miniBatchSize);    
            }
        }
        catch(cv::Exception &e)
        {
            const char *err = e.what();
            sciprint("%s", err);
        }

        try
        {
        	p->train(tdata);
		    trainingData = tdata->getSamples();
		    transpose(trainingData, trainingData);
        }
        catch(cv::Exception &e)
        {
        	const char *err = e.what();
        	sciprint("%s", err);
        }
        
		try
        {
        	p->save(classifierLocation);	
        }
        catch(cv::Exception &e)
        {
        	const char *err = e.what();
        	sciprint("%s", err);
        }
	    		
        sciprint("done.\n");
        //------Create output arguments------//
        sciErr = createList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, 4, &piAddr);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        sciErr = createMatrixOfStringInList(pvApiCtx, nbInputArgument(pvApiCtx)+1, piAddr, 1, 1, 1, &objectType);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        sciErr = createMatrixOfStringInList(pvApiCtx, nbInputArgument(pvApiCtx)+1, piAddr, 2, 1, 1, &classifierLocation);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        sciErr = createMatrixOfStringInList(pvApiCtx, nbInputArgument(pvApiCtx)+1, piAddr, 3, 1, 1, &bagOfFeaturesLocation);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        sciErr = createMatrixOfStringInList(pvApiCtx, nbInputArgument(pvApiCtx)+1, piAddr, 4, descriptionCount, 1, description);
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
    /* ==================================================================== */
}
