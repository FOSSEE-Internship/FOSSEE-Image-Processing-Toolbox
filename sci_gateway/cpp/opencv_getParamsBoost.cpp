/***************************************************
Author : Nihar Rao
***************************************************/
#include <iostream>
#include<bits/stdc++.h>
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

extern "C"
{
    #include "api_scilab.h"
    #include "Scierror.h"
    #include "BOOL.h"
    #include <localization.h>
    #include "sciprint.h"
    #include "../common.h"
    int opencv_getParamsBoost(char *fname, unsigned long fname_len)
    {
        // Error management variables
        SciErr sciErr;

        //------Local variables------//
        int upright = 1;
        Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("FlannBased");
        Ptr<SURF> detector = SURF::create(400, 4, 2, 1, int(upright));
        Ptr<DescriptorExtractor> extractor=detector;
        Ptr<BOWImgDescriptorExtractor> bowDE=makePtr<BOWImgDescriptorExtractor>(extractor,matcher);
        //SurfFeatureDetector detector(1, 4, 2, 1, int(upright));
       // char *classifierLocation = NULL;
        Mat dictionary,features;
        double response;
        vector<KeyPoint> keyPoints;
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
         Mat layermat,weights;
         double cv_f,max_cat,md,msc,reg_acc,bt,wc,wtr;
         bool prune,use1_se,use_surr;
         int lid =1;
	 //double C=0,gamma=1,coef=0,degree=0,P=0,Nu=0,kernelType=2,setType=100;
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

        if(!(strcmp(pstData[0],"classifier")==0))
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
           sciprint("getting the parameters of trained ANN Model...\n");
       //------Actual processing------//
	try{
       Ptr<Boost> dt = Algorithm::load<Boost>(classifierLocation);
        if(!dt->isTrained())
            {
             Scierror(999,"\nthe given model is not trained!,please try again!\n");
                return 0;

            }
        bt=dt->getBoostType();
        wc=dt->getWeakCount();
        //layermat=ann->getLayerSizes();
        wtr = dt->getWeightTrimRate();
        md = dt->getMaxDepth();
        //msc = dt->getMinSampleCount();
        //reg_acc = dt->getRegressionAccuracy();
        //prune = dt->getTruncatePrunedTree();
        //use1_se = dt->getUse1SERule();
        use_surr = dt->getUseSurrogates();

        //weights = ann->getWeights (lid);
        //Nu=svm->getNu();
        //P=svm->getP();
        //setType=svm->getType();

        sciprint("\ndone");
        }
        catch(cv::Exception& e)
	{
		const char* err=e.what();
		sciprint("%s",err);

	}
	//int prune1 = prune;
        //int use1_se1 = use1_se;
        int use_surr1 = use_surr;
       //------Create output arguments------//
        sciErr = createList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, 5, &piAddr);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        sciErr = createMatrixOfDoubleInList(pvApiCtx, nbInputArgument(pvApiCtx)+1, piAddr, 1, 1, 1, &bt);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        sciErr = createMatrixOfDoubleInList(pvApiCtx, nbInputArgument(pvApiCtx)+1, piAddr, 2, 1, 1, &wc);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
       /*sciErr = createMatrixOfDoubleInList(pvApiCtx, nbInputArgument(pvApiCtx)+1, piAddr, 3, 1, 1, &layermat);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }*/
        sciErr = createMatrixOfDoubleInList(pvApiCtx, nbInputArgument(pvApiCtx)+1, piAddr, 3, 1, 1, &wtr);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
	sciErr = createMatrixOfDoubleInList(pvApiCtx, nbInputArgument(pvApiCtx)+1, piAddr, 4, 1, 1, &md);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
	/*sciErr = createMatrixOfDoubleInList(pvApiCtx, nbInputArgument(pvApiCtx)+1, piAddr, 5, 1, 1, &reg_acc);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
	sciErr = createMatrixOfBooleanInList(pvApiCtx, nbInputArgument(pvApiCtx)+1, piAddr, 6, 1, 1, &prune1);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
	sciErr = createMatrixOfBooleanInList(pvApiCtx, nbInputArgument(pvApiCtx)+1, piAddr, 7, 1, 1, &use1_se1);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }*/
        sciErr = createMatrixOfBooleanInList(pvApiCtx, nbInputArgument(pvApiCtx)+1, piAddr, 5, 1, 1, &use_surr1);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        /*sciErr = createMatrixOfDoubleInList(pvApiCtx, nbInputArgument(pvApiCtx)+1, piAddr, 10, 1, 1, &weights);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }*/
        //------Return Arguments------//
        AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx)+1;
        ReturnArguments(pvApiCtx);
        return 0;
    }
    /* ==================================================================== */
}

