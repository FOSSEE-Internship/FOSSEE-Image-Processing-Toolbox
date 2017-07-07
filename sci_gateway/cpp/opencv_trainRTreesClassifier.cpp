/***************************************************
Author : Gursimar Singh,Rohit Suri
***************************************************/
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/ml/ml.hpp>
#include <sys/stat.h>

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;
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

//******************************function to convert double matrix to vector*************************//
    vector <double> double2Vec( double * in,int rows,int cols)
    {
        vector<double> out (rows*cols);
        for (int i = 0; i < rows; i++) 
        {
        
           for (int j = 0; j < cols; j++) 
           {
                out.at(i*cols + j) = in[i + j*rows];
               
           } 
        
        }
        return out ;  

    }
//****************************main function*****************************************************
    int opencv_trainRTreesClassifier(char *fname, unsigned long fname_len)
    {
        // Error management variables
        SciErr sciErr;
        //------Local variables------//
        int upright = 1;
        Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("FlannBased");
        Ptr<SURF> detector = SURF::create(400,4,2,1,upright);
	    Ptr<DescriptorExtractor> extractor = detector;
        Ptr<BOWImgDescriptorExtractor> bowDE=makePtr<BOWImgDescriptorExtractor>(extractor, matcher);
        char *fileName = NULL;
        Mat dictionary,inp,features;
        vector<KeyPoint> keyPoints;

        int *piAddr = NULL;
        int *piChild = NULL;
        int iRows, iCols;
        char **pstData = NULL;
        int *piLen = NULL;
	    int piBool=0;
        int iType=0;
	    double param=0;
        bool iRet= FALSE;
	    double* piPrior=NULL;
        int *count = NULL;
        char **description = NULL;
        char ***location = NULL;
        char *bagOfFeaturesLocation = NULL;
        int descriptionCount;
        char *classifierLocation = NULL;
        char *classifierName = NULL;
        char *objectType = "RTclassifier";
        bool CalVarImp= FALSE;
        bool TruncPruneTree=TRUE;
        bool UseSurrogates=FALSE;
        bool Use1SERule=TRUE;
        int ActiveVarCount=0;
        int CVfolds=1;
        int MaxCategories=10;
        int MaxDepth=20;
        int MinSampleCount=10;
        float RegressionAccuracy=0.01;
        Mat prior;


        //------Check number of parameters------//
        CheckInputArgument(pvApiCtx, 3, 14);
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
            pstData[iter] = (char*)malloc(sizeof(char) * (piLen[iter] + 1));//+ 1 for null termination
        }

        sciErr = getMatrixOfStringInList(pvApiCtx, piAddr, 1, &iRows, &iCols, piLen, pstData);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
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

        if(!file_exists_check(location[0][1]))
        {
     
            Scierror(999, "Error: the imageSet File not found!.Please enter correct path!\n");
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
            pstData[iter] = (char*)malloc(sizeof(char) * (piLen[iter] + 1));//+ 1 for null termination
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
        //File existence check
        if(!file_exists_check(bagOfFeaturesLocation))
        {
     
            Scierror(999, "Error: the bagOfFeatures  File not found!.Please enter correct path!\n");
            return 0;
        }           


        //third argument
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

	///fourth argument
	if (*getNbInputArgument(pvApiCtx)>3)
	{

	    sciErr = getVarAddressFromPosition(pvApiCtx, 4,&piAddr);
        if (sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        if(!isDoubleType(pvApiCtx, piAddr) || !isScalar(pvApiCtx, piAddr))
        {
            Scierror(999, "Error: The input argument #4 is not of type integer or scalar.\n");
            return 0;

		
        }
        double cData=0;
	    iRet=getScalarDouble(pvApiCtx,piAddr,&param);
        if(!iRet)
        {
            cData = param;
        }
	
	    ActiveVarCount=cData;


	}
	else
	   ActiveVarCount=0;

	//fifth argument
		
	if (*getNbInputArgument(pvApiCtx)>4)
	{

	    sciErr = getVarAddressFromPosition(pvApiCtx, 5,&piAddr);
        if (sciErr.iErr)
        {
           printError(&sciErr, 0);
           return 0;
        }
        if(!isBooleanType(pvApiCtx, piAddr) || !isScalar(pvApiCtx,piAddr))
        {
            Scierror(999, "Error: The input argument #5 is not of type boolean.\n");
            return 0;

		
        }
	    bool iBool = 0;
            
        iRet = getScalarBoolean(pvApiCtx, piAddr, &piBool);
     
        if(!iRet)
        {
      
          iBool= piBool;
        }
    
        CalVarImp=iBool;
	}

	else
	    CalVarImp=FALSE;
//6th argument

	if (*getNbInputArgument(pvApiCtx)>5)
	{

	sciErr = getVarAddressFromPosition(pvApiCtx, 6,&piAddr);
        if (sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        if(!isDoubleType(pvApiCtx, piAddr) || !isScalar(pvApiCtx,piAddr))
        {
            Scierror(999, "Error: The input argument #6 is not of type integer.\n");
            return 0;		
        }
	
        double cData=0;
        iRet=getScalarDouble(pvApiCtx,piAddr,&param);
        if(!iRet)
        {
            cData = param;
        }
	    CVfolds=cData;
        
	}
	else
	    CVfolds=1;

//7th argument
	if (*getNbInputArgument(pvApiCtx)>6)
	{

	    sciErr = getVarAddressFromPosition(pvApiCtx, 7,&piAddr);
        if (sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        if(!isDoubleType(pvApiCtx, piAddr) || !isScalar(pvApiCtx,piAddr))
        {
            Scierror(999, "Error: The input argument #7 is not of type integer.\n");
            return 0;

		
        }
	    double cData=0;
        iRet=getScalarDouble(pvApiCtx,piAddr,&param);
        if(!iRet)
        {
            cData = param;
        }
	    MaxCategories=cData;
       
	}
	else
	    MaxCategories=10;

//8th argument
	if (*getNbInputArgument(pvApiCtx)>7)
	{

	sciErr = getVarAddressFromPosition(pvApiCtx, 8,&piAddr);
        if (sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        if(!isDoubleType(pvApiCtx, piAddr) || !isScalar(pvApiCtx,piAddr))
        {
            Scierror(999, "Error: The input argument #8 is not of type integer.\n");
            return 0;

		
        }
	//sciErr=getScalarDouble(pvApiCtx,piAddr,&param);
	   double cData=0;
        iRet=getScalarDouble(pvApiCtx,piAddr,&param);
        if(!iRet)
        {
            cData = param;
        }
	    MaxDepth=cData;
	}
	else
	    MaxDepth=20;

	//9th argument
	if (*getNbInputArgument(pvApiCtx)>8)
	{

	    sciErr = getVarAddressFromPosition(pvApiCtx, 9,&piAddr);
        if (sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        if(!isDoubleType(pvApiCtx, piAddr) || !isScalar(pvApiCtx,piAddr))
        {
            Scierror(999, "Error: The input argument #9 is not of type integer.\n");
            return 0;

		
        }
	//sciErr=getScalarDouble(pvApiCtx,piAddr,&param);
	    double cData=0;
        iRet=getScalarDouble(pvApiCtx,piAddr,&param);
        if(!iRet)
            {
                cData = param;
            }
	     MinSampleCount=cData;
	}
	else
	     MinSampleCount=10;


	//10th argument
	if (*getNbInputArgument(pvApiCtx)>9)
	{

	    sciErr = getVarAddressFromPosition(pvApiCtx, 10,&piAddr);
        if (sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
	    sciErr = getVarType(pvApiCtx, piAddr, &iType);
	   if(sciErr.iErr || iType != sci_matrix)
	   {
		  printError(&sciErr, 0);
		  return 0;
	   }

        if(!isDoubleType(pvApiCtx, piAddr) || isVarComplex(pvApiCtx,piAddr))
        {
            Scierror(999, "Error: The input argument #10 is not of type double.\n");
            return 0;
        }
	   sciErr=getMatrixOfDouble(pvApiCtx,piAddr,&iRows,&iCols,&piPrior);
	   if(sciErr.iErr)
    	{
            printError(&sciErr, 0);
            return 0;
    	}
	   if (iCols != descriptionCount && iRows !=1)
	   {
		
            Scierror(999, "invalid  prior argument.\n");
            return 0;
	   }
         prior = Mat(iRows,iCols,CV_32F);
    
        for (int i =0; i<descriptionCount;i++)
        {
        
            prior.at<float>(i) = piPrior[i];   

        }

	}
	else
	    prior=Mat();

	//11th argument
	if (*getNbInputArgument(pvApiCtx)>10)
	{

	    sciErr = getVarAddressFromPosition(pvApiCtx, 11,&piAddr);
        if (sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        if(!isDoubleType(pvApiCtx, piAddr) || !isScalar(pvApiCtx,piAddr))
        {
            Scierror(999, "Error: The input argument #11 is not of type double.\n");
            return 0;
        }
	    double cData=0;
        iRet=getScalarDouble(pvApiCtx,piAddr,&param);
        if(!iRet)
        {
            cData = param;
        }
         RegressionAccuracy=cData;
	}
	else
	    RegressionAccuracy=0.01;
	
	//12th argument
	if (*getNbInputArgument(pvApiCtx)>11)
	{

	    sciErr = getVarAddressFromPosition(pvApiCtx, 12,&piAddr);
        if (sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        if(!isBooleanType(pvApiCtx, piAddr) || !isScalar(pvApiCtx,piAddr))
        {
            Scierror(999, "Error: The input argument #12 is not of type boolean.\n");
            return 0;

		
        }
    	bool iBool = 0;
            
        iRet = getScalarBoolean(pvApiCtx, piAddr, &piBool);
             
        if(!iRet)
        {
              
            iBool= piBool;
        }

	    TruncPruneTree = iBool;
	}
	else
	     TruncPruneTree = TRUE;
	
	//13th argument
	if (*getNbInputArgument(pvApiCtx)>12)
	{

	    sciErr = getVarAddressFromPosition(pvApiCtx, 13,&piAddr);
        if (sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        if(!isBooleanType(pvApiCtx, piAddr) || !isScalar(pvApiCtx,piAddr))
        {
            Scierror(999, "Error: The input argument #13 is not of type boolean.\n");
            return 0;

		
        }
    	bool iBool = 0;
            
        iRet = getScalarBoolean(pvApiCtx, piAddr, &piBool);
             
        if(!iRet)
        {      
            iBool= piBool;
        }
	    UseSurrogates = iBool;

	}
	else
	    UseSurrogates = FALSE;
	//14th argument
	if (*getNbInputArgument(pvApiCtx)>13)
	{

	sciErr = getVarAddressFromPosition(pvApiCtx, 14,&piAddr);
        if (sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        if(!isBooleanType(pvApiCtx, piAddr) || !isScalar(pvApiCtx,piAddr))
        {
            Scierror(999, "Error: The input argument #14 is not of type boolean.\n");
            return 0;

		
        }
	    bool iBool = 0;
            
        iRet = getScalarBoolean(pvApiCtx, piAddr, &piBool);
             
        if(!iRet)
        {      
            iBool= piBool;
        }
	    
        Use1SERule = iBool;
	}
	else
	    Use1SERule = FALSE;
	
        
    classifierLocation=classifierName;
    strcat(classifierLocation,".yml");
    //------Actual processing------//
    FileStorage fs(bagOfFeaturesLocation, FileStorage::READ);
    fs["dictionary"] >> dictionary;
    fs.release();
    if(dictionary.rows==0 || dictionary.cols==0)
    {
        sciprint("Error: The provided file for bagOfFeatures may be invalid.\n");
    }
    sciprint("Training an image category classifier for %d categories.\n",descriptionCount);
    sciprint("-------------------------------------------------------\n\n");
    for(int i=0;i<descriptionCount;i++)
    {
        sciprint("# Category %d: %s\n",i+1,description[i]);
    }
    sciprint("\n");
    int dictionarySize = dictionary.rows;
    Mat labels(0, 1, CV_8U);

    Mat trainingData(0, dictionarySize, CV_32FC1);

	//Setting Vocabulary
	try
    {
        bowDE->setVocabulary(dictionary);
	}
	catch(cv::Exception&e)
    {
        const char* err=e.what();
        Scierror(999,e.what());
    }

    for(int i=0; i<descriptionCount;i++)
    {
        sciprint("# Encoding features for Category %d ...",i+1);
        for(int j=0; j<count[i]; j++)
        {
            features.release();
            keyPoints.clear();
            fileName = location[i][j];
            if(!file_exists_check(fileName))
            {
                Scierror(999, "Error: the image file not found.Please enter correct path.\n");
                return 0;
            }
            inp = imread(fileName);
            try
            {
                detector->detect(inp,keyPoints);
            }
            catch(cv::Exception&e)
            {
                const char* err=e.what();
                Scierror(999,e.what());
            }
            try
            {
                bowDE->compute(inp,keyPoints,features);
            }
            catch(cv::Exception&e)
	        {
	           const char* err=e.what();
	           Scierror(999,e.what());
	        }

            trainingData.push_back(features);
            labels.push_back((int) i);
        
        
        }
    }
	

    sciprint("\n# Training the category classifier...");
	
	Ptr<ml::RTrees> model = cv::ml::RTrees::create();	
	try
    {
        model->setActiveVarCount(ActiveVarCount);
    }
	catch(cv::Exception &e)
    {
       const char *err = e.what();
       Scierror(999,e.what());
    }
	try
    {
       model->setCalculateVarImportance(CalVarImp);
    }
	catch(cv::Exception &e)
    {
       const char *err = e.what();
       Scierror(999,e.what());
    }
	try
    {
       model->setCVFolds(CVfolds);
    }
	catch(cv::Exception &e)
    {
       const char *err = e.what();
       Scierror(999,e.what());
    }	
	try
    {
       model->setMaxDepth(MaxDepth);
    }
    catch(cv::Exception &e)
    {
       const char *err = e.what();
       Scierror(999,e.what());
    }
        
    try
    {
       model->setMinSampleCount(MinSampleCount);   
    }
    catch(cv::Exception &e)
    {
       const char *err = e.what();
       Scierror(999,e.what());
    }      
        
    try
    {
      model->setRegressionAccuracy(RegressionAccuracy);
    }
    catch(cv::Exception &e)
    {
       const char *err = e.what();
       Scierror(999,e.what());
    }
    
    try
    {
      model->setUseSurrogates(UseSurrogates); 
    }
    catch(cv::Exception &e)
    {
       const char *err = e.what();
       Scierror(999,e.what());
    }
    
    try
    {
      model->setMaxCategories(MaxCategories);
    }
    catch(cv::Exception &e)
    {
       const char *err = e.what();
       Scierror(999,e.what());
    }
    
    try
    {
      model->setUse1SERule(Use1SERule);
    }
    catch(cv::Exception &e)
    {
       const char *err = e.what();
       Scierror(999,e.what());
    }

    try
    {
      model->setTruncatePrunedTree(TruncPruneTree);

    }
    catch(cv::Exception &e)
    {
       const char *err = e.what();
       Scierror(999,e.what());
    }
    
    try
    {

      model->setPriors(prior);

    }
    catch(cv::Exception &e)
    {
      const char *err = e.what();
      Scierror(999,e.what());
    }
 
   try
    {
       TermCriteria t =TermCriteria(TermCriteria::MAX_ITER,100,0.0000001);
       model->setTermCriteria(t);
    }
    catch(cv::Exception &e)
    {
       const char *err = e.what();
       Scierror(999,e.what());
    }
	
	try
    {
	   model->train(trainingData,ROW_SAMPLE,labels);
	}
 	catch(cv::Exception &e)
    {
       const char *err = e.what();
       sciprint("%s",err);
    }

    model->Algorithm::save(classifierLocation);

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
