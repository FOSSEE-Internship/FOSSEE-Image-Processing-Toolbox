/********************************************************
Author:Gursimar Singh
*********************************************************
*/

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/face.hpp>
#include <sys/stat.h>

using namespace std;
using namespace cv;
using namespace cv:: face ;
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

    int opencv_trainFaceRecognizer(char *fname, unsigned long fname_len)
    {
        SciErr sciErr;

        Mat inp;
    	int *piAddr = NULL;
        int *piChild = NULL;
        int iRows, iCols,iRet;
        char **pstData = NULL;
        int *piLen = NULL;
        int *count = NULL;
        char **description = NULL;
        char ***location = NULL;
        int descriptionCount=0;
        char *algoName=NULL;
        char *classifierLocation = "face_recognizer.xml";
        char *fileName=NULL;

	    CheckInputArgument(pvApiCtx, 2, 2);
        CheckOutputArgument(pvApiCtx, 1, 1);

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
        //////////// 2nd input///////////////

        sciErr = getVarAddressFromPosition(pvApiCtx, 2, &piAddr);
        if (sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }

        iRet=getAllocatedSingleString(pvApiCtx,piAddr,&algoName);

        	if(iRet)
        	{
        		freeAllocatedSingleString(algoName);
				return iRet;
        	}
        if(!file_exists_check(location[0][1]))
        {
     
            Scierror(999, "Error: the imageSet. File not found!.Please enter correct path!\n");
            return 0;
        }

        Mat labels(0, 1, CV_8U);
        vector <Mat> images;

         for(int i=0; i<descriptionCount;i++)
        {
            sciprint("# Encoding features for Category %d ...",i+1);
            for(int j=0; j<count[i]; j++)
            {
                
                fileName = location[i][j];
	       		try
                {
                 	inp=imread(fileName, CV_LOAD_IMAGE_GRAYSCALE);
                 	resize(inp,inp,Size(128,128),0,0,cv::INTER_AREA);
                 	images.push_back(inp);
                 	labels.push_back((int) i);
	           	}
            	catch(cv::Exception&e)
                {
                  const char* err=e.what();
                  Scierror(999,e.what());
                }
	     	}
	    }
	     	  
	    if (strcmp(algoName,"LBPH")==0)  
        { 
	    	Ptr<LBPHFaceRecognizer> model = createLBPHFaceRecognizer();
            model->setThreshold(123.0);
            model->train(images,labels);
            model->save("face_recognizer.xml");
        }    
    	else if (strcmp(algoName,"EIGEN")==0)
    	{	
             Ptr<FaceRecognizer> model = createEigenFaceRecognizer();
             //sciprint("here\n");
            try
            {
                model->train(images,labels);
                model->save("face_recognizer.xml");
            }

            catch(cv::Exception&e)
            {
                const char* err=e.what();
                Scierror(999,e.what());
            }
        }
    	else if (strcmp(algoName,"FISHER")==0)
    	{	 Ptr<BasicFaceRecognizer> model = createFisherFaceRecognizer();	 
            try
            {
                model->train(images,labels);
                model->save("face_recognizer.xml");
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

    	sciprint("done\n");	

    	 sciErr = createList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, 3, &piAddr);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        sciErr = createMatrixOfStringInList(pvApiCtx, nbInputArgument(pvApiCtx)+1, piAddr, 1, 1, 1, &algoName);
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
       
        sciErr = createMatrixOfStringInList(pvApiCtx, nbInputArgument(pvApiCtx)+1, piAddr, 3, descriptionCount, 1, description);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }

        /////----//////-----Return Arguments---///////---/////
        AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx)+1;
        ReturnArguments(pvApiCtx);
        return 0;  


    }
}
