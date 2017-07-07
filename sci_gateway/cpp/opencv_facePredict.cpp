/********************************************************
Author:Gursimar Singh
*********************************************************
*/

#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/face.hpp"
#include "opencv2/opencv.hpp"
#include <sys/stat.h>

using namespace cv;
using namespace cv::face;
using namespace std;

extern "C"
{
    #include "api_scilab.h"
    #include "Scierror.h"
    #include "BOOL.h"
    #include <localization.h>
    #include "sciprint.h"
    #include "../common.h"
    int opencv_facePredict(char *fname, unsigned long fname_len)
    {
        // Error management variables
        SciErr sciErr;

        //------Local variables------//
        
        int *piAddr = NULL;
        int *piChild = NULL;
        int iRows, iCols,iRet;
        char **pstData = NULL;
        int *piLen = NULL;
        char **classifierDescription = NULL;
        
        int descriptionCount;
        Mat input,inp;
        //------Check number of parameters------//
        CheckInputArgument(pvApiCtx, 2, 2);
        CheckOutputArgument(pvApiCtx, 1, 2);

        //------Get input arguments------//
        retrieveImage(input,2);
        sciprint("image retrievd\n");
        try{
            input.convertTo(input,CV_8U);
            cvtColor(input, input, cv::COLOR_RGB2GRAY);

        }
        catch(cv::Exception&e)
            {
                const char* err=e.what();
                Scierror(999,e.what());
            }
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
        const char  *algoName = pstData[0];
        sciprint("getting algo name\n");
        
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
        sciprint("getting classifier Location\n");
        sciErr = getMatrixOfStringInList(pvApiCtx, piAddr, 3, &iRows, &iCols, NULL, NULL);
        if (sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        //classifier description
        piLen = (int*) malloc(sizeof(int) * iRows * iCols);

        sciErr = getMatrixOfStringInList(pvApiCtx,  piAddr, 3,  &iRows,  &iCols,  piLen,  NULL);
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

        sciErr = getMatrixOfStringInList(pvApiCtx, piAddr, 3, &iRows, &iCols, piLen, classifierDescription);
        if (sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }

        descriptionCount=iRows;
        sciprint("getting Description Count\n");

////////////////////Actual Processing/////////////////////
        int predictedLabel = -1;
        double predictedConfidence = 0.0;

        if (strcmp(algoName,"LBPH")==0)  
        { 
            Ptr<LBPHFaceRecognizer> model = createLBPHFaceRecognizer();
            sciprint("model created");
            try
            {
                model->load(classifierLocation);
// Get the prediction and associated confidence from the model
                 model->predict(input, predictedLabel, predictedConfidence);
            }
            catch(cv::Exception&e)
            {
                const char* err=e.what();
                Scierror(999,e.what());
            }

        }    
        else if (strcmp(algoName,"EIGEN")==0)
        {   
            Ptr<FaceRecognizer> model = createEigenFaceRecognizer();
            model->load(classifierLocation);
             
             try
             {
                resize(input,inp,Size(128,128),0,0,cv::INTER_AREA);
                model->predict(inp, predictedLabel, predictedConfidence);
             }
        
            catch(cv::Exception&e)
            {
                const char* err=e.what();
                Scierror(999,e.what());
            }
        }
        else if (strcmp(algoName,"FISHER")==0)
        {    
            Ptr<BasicFaceRecognizer> model = createFisherFaceRecognizer();
            model->load(classifierLocation);
            try
            {
                resize(input,inp,Size(128,128),0,0,cv::INTER_AREA);
                model->predict(inp, predictedLabel, predictedConfidence);
            }
        
            catch(cv::Exception&e)
            {
                const char* err=e.what();
                Scierror(999,e.what());
            }
        }    
        else 
        {
            Scierror(999, "Error: Undefined Algorithm name.\n");
            return 0;
        }  

        char *predictedPerson=classifierDescription[(int)predictedLabel];
        iRet = createSingleString(pvApiCtx, nbInputArgument(pvApiCtx)+1, predictedPerson);
        if(iRet)
        {
            
            return iRet;
        }
        sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 2, 1, 1, &predictedConfidence);
        
        if(sciErr.iErr){
            printError(&sciErr, 0);
            return 0;
        }
        AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx)+1;
        AssignOutputVariable(pvApiCtx, 2) = nbInputArgument(pvApiCtx)+2;
        //AssignOutputVariable(pvApiCtx, 2) = nbInputArgument(pvApiCtx)+2;
        ReturnArguments(pvApiCtx);
        return 0;
    
    }
}


          