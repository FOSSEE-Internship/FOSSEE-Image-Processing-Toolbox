/********************************************************
Author: Suraj Prakash & Gursimar Singh
[bboxes] = peopleDetector(image)

// 1->image

Optional arguments :

// 2->hitThreshold 
// 3->winStride
// 4->padding
// 5->scale
// 6->finalThreshold
// 7->useMeanshiftGrouping

********************************************************/

//#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

extern "C"{
  #include "api_scilab.h"
  #include "Scierror.h"
  #include "BOOL.h"
  #include <localization.h>
  #include <sciprint.h>
  #include "../common.h"
  
  int opencv_peopleDetector(char *fname, unsigned long fname_len){

    /// Error management variable
    SciErr sciErr;
    
    /// Variables
    int i, j;
    int intErr;
    int iRows = 0;
    int iCols = 0;
    int iRows1 = 0;
    int iCols1 = 0;
    
    int *piLen = NULL;
    int *piAddr = NULL;
    int *piAddr2 = NULL;
    int *piAddr3 = NULL;
    char **pstData = NULL;
    char *currentArg = NULL;
    bool *providedArgs = NULL; 
    
    double hitThreshold = 0;
    double scale = 1.05; 
    double *winStride = NULL;
    double *padding = NULL;
    double finalThreshold = 2.0;
    int useMeanshiftGrouping = 0;

    double *bboxes = NULL;
    int c=2;


    /// for the Hog_SVM_Detector
    HOGDescriptor hog;
    
    vector< Rect > found, found_filtered, answer;

    CheckInputArgument(pvApiCtx, 1, 13);
    CheckOutputArgument(pvApiCtx, 1, 1);

    /// retrieve image 
    Mat image;
    retrieveImage(image, 1);
   // Size sz(128,256);
    //resize(image,image,sz);
    resize(image,image,Size(min(400,image.cols),min(800,image.rows)));
    image.convertTo(image,CV_8UC1);

    /// Train hog detector for people
    /// Default training is 48 * 96 people images
    /// can be used for detection of larger instances


    /// For the optional arguments
    int nbInputArguments = *getNbInputArgument(pvApiCtx);

    providedArgs = (bool*) malloc(sizeof(bool) * 5); 
    memset(providedArgs, 0, 5);

    winStride = (double*)malloc(sizeof(double) * 2);
    winStride[0] = 8;
    winStride[1] = 8;

    padding = (double*)malloc(sizeof(double) * 2);
    padding[0] = 4;
    padding[1] = 4;


    for(int iter = 2; iter <= nbInputArguments; ++iter){

        /// Address of next argument
        
        sciErr = getVarAddressFromPosition(pvApiCtx, iter, &piAddr); 
        if (sciErr.iErr)
        {
            printError(&sciErr, 0); 
            return 0; 
        }

        // Three calls to getMatrixOfString
        sciErr = getMatrixOfString(pvApiCtx, piAddr, &iRows, &iCols, NULL, NULL); 
        if (sciErr.iErr)
        {
            printError(&sciErr, 0); 
            return 0; 
        }
        
        piLen = (int*) malloc(sizeof(int) * iRows * iCols); 
        
        sciErr = getMatrixOfString(pvApiCtx,  piAddr,  &iRows,  &iCols,  piLen,  NULL); 
        if (sciErr.iErr)
        {
            printError(&sciErr, 0); 
            return 0; 
        }
        
        pstData = (char**) malloc(sizeof(char*) * iRows * iCols); 
        for(int k = 0; k < iRows * iCols; ++k)
        {
            pstData[k] = (char*) malloc(sizeof(char) * piLen[k] + 1); 
        }
        
        sciErr = getMatrixOfString(pvApiCtx, piAddr, &iRows, &iCols, piLen, pstData); 
        if (sciErr.iErr)
        {
            printError(&sciErr, 0); 
            return 0; 
        }

        currentArg = pstData[0];         

            /// getting hit_threshold
            if(strcmp(currentArg, "hitThreshold") == 0)
            {
                if(iter + 1 <= nbInputArguments and !providedArgs[0])
                {
                    sciErr = getVarAddressFromPosition(pvApiCtx, ++iter, &piAddr); 
                    if (sciErr.iErr)
                    {
                        printError(&sciErr, 0); 
                        return 0; 
                    }
                    intErr = getScalarDouble(pvApiCtx, piAddr, &hitThreshold); 
                    if(intErr)
                    {
                        return intErr; 
                    }
                    // Checking if values are in proper range. Same for all optional arguments
                    if(hitThreshold < 0)
                    {
                        Scierror(999, "Error: Nonnegative hitThreshold only allowed\n"); 
                        return 0; 
                    }
                    providedArgs[0] = 1; 
                }
                else if(providedArgs[0]) // Send an error message if an argument is provided more than once. Same for all optional arguments.
                {
                    Scierror(999, "Please provide optional arguments only once.\n"); 
                    return 0; 
                }
                else // Send an error message if name of argument is given but type is incorrect. Same for all optional arguments.
                {
                    Scierror(999, "Incorrect number of arguments provided. Please check the documentation for more information.\n"); 
                    return 0; 
                }
            }

            /// WindowStride
            if(strcmp(currentArg, "winStride") == 0)
            {
                if(iter+1 <= nbInputArguments && !providedArgs[1])
                {
                    sciErr = getVarAddressFromPosition(pvApiCtx, ++iter, &piAddr); 
                    if (sciErr.iErr)
                    {
                        printError(&sciErr, 0); 
                        return 0; 
                    }
                    sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &iRows1, &iCols1, &winStride); 
                    if(sciErr.iErr)
                    {
                        printError(&sciErr, 0); 
                        return 0; 
                    }
                    if(iRows1 != 1 or iCols1 != 2)
                    {
                        Scierror(999, "Incorrect dimension of matrix for argument .\n"); 
                        return 0; 
                    }
                    
                    // Checking if values are in proper range. Same for all optional arguments
                    providedArgs[1] = 1;
                }
                else if(providedArgs[1]) // Send an error message if an argument is provided more than once. Same for all optional arguments.
                {
                    Scierror(999, "Please provide optional arguments only once.\n"); 
                    return 0; 
                }
                else // Send an error message if name of argument is given but type is incorrect. Same for all optional arguments.
                {
                    Scierror(999, "Incorrect number of arguments provided. Please check the documentation for more information.\n"); 
                    return 0; 
                }
            }

            /// Padding
            if(strcmp(currentArg, "padding") == 0)
            {
                if(iter+1 <= nbInputArguments and !providedArgs[2])
                {
                    sciErr = getVarAddressFromPosition(pvApiCtx, ++iter, &piAddr); 
                    if (sciErr.iErr)
                    {
                        printError(&sciErr, 0); 
                        return 0; 
                    }
                    sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &iRows1, &iCols1, &padding); 
                    if(sciErr.iErr)
                    {
                        printError(&sciErr, 0); 
                        return 0; 
                    }
                    if(iRows1 != 1 or iCols1 != 2)
                    {
                        Scierror(999, "Incorrect dimension of matrix for argument .\n"); 
                        return 0; 
                    }
                    
                    // Checking if values are in proper range. Same for all optional arguments
                    providedArgs[2] = 1;
                }
                else if(providedArgs[2]) // Send an error message if an argument is provided more than once. Same for all optional arguments.
                {
                    Scierror(999, "Please provide optional arguments only once.\n"); 
                    return 0; 
                }
                else // Send an error message if name of argument is given but type is incorrect. Same for all optional arguments.
                {
                    Scierror(999, "Incorrect number of arguments provided. Please check the documentation for more information.\n"); 
                    return 0; 
                }
            }

            /// Scalefactor
            if(strcmp(currentArg, "scale") == 0)
            {
                if(iter + 1 <= nbInputArguments and !providedArgs[3])
                {
                    sciErr = getVarAddressFromPosition(pvApiCtx, ++iter, &piAddr); 
                    if (sciErr.iErr)
                    {
                        printError(&sciErr, 0); 
                        return 0; 
                    }
                    intErr = getScalarDouble(pvApiCtx, piAddr, &scale); 
                    if(intErr)
                    {
                        return intErr; 
                    }
                    // Checking if values are in proper range. Same for all optional arguments
                    if(scale < 1.0001)
                    {
                        Scierror(999, "Error: Wrong ScaleFactor given. Cannot be less than 1.0001\n"); 

                        return 0; 
                    }
                    providedArgs[3] = 1; 
                }
                else if(providedArgs[3]) // Send an error message if an argument is provided more than once. Same for all optional arguments.
                {
                    Scierror(999, "Please provide optional arguments only once.\n"); 
                    return 0; 
                }
                else // Send an error message if name of argument is given but type is incorrect. Same for all optional arguments.
                {
                    Scierror(999, "Incorrect number of arguments provided. Please check the documentation for more information.\n"); 
                    return 0; 
                }
            }

            

            /// finalThreshold
            if(strcmp(currentArg, "finalThreshold") == 0)
            {
                if(iter + 1 <= nbInputArguments and !providedArgs[4])
                {
                    sciErr = getVarAddressFromPosition(pvApiCtx, ++iter, &piAddr); 
                    if (sciErr.iErr)
                    {
                        printError(&sciErr, 0); 
                        return 0; 
                    }
                    intErr = getScalarDouble(pvApiCtx, piAddr, &finalThreshold); 
                    if(intErr)
                    {
                        return intErr; 
                    }

                    if(finalThreshold < 0)
                    {
                        Scierror(999, "Incorrect value for argument .\n"); 
                        return 0; 
                    }
                    
                    // Checking if values are in proper range. Same for all optional arguments
                    providedArgs[4] = 1;
                }
                else if(providedArgs[4]) // Send an error message if an argument is provided more than once. Same for all optional arguments.
                {
                    Scierror(999, "Please provide optional arguments only once.\n"); 
                    return 0; 
                }
                else // Send an error message if name of argument is given but type is incorrect. Same for all optional arguments.
                {
                    Scierror(999, "Incorrect number of arguments provided. Please check the documentation for more information.\n"); 
                    return 0; 
                }
            }

            /// UseMeanshiftGrouping
            if(strcmp(currentArg, "useMeanshiftGrouping")==0)
            {
                if(iter + 1 <= nbInputArguments and !providedArgs[5])
                {
                    sciErr = getVarAddressFromPosition(pvApiCtx, ++iter, &piAddr); 
                    if (sciErr.iErr)
                    {
                        printError(&sciErr, 0); 
                        return 0; 
                    }
                    intErr = getScalarBoolean(pvApiCtx, piAddr, &useMeanshiftGrouping);  
                    if(sciErr.iErr)
                    {
                        printError(&sciErr, 0); 
                        return 0; 
                    }
                    if(useMeanshiftGrouping > 1 or useMeanshiftGrouping < 0)
                    {
                        Scierror(999, "Incorrect value for argument .\n"); 
                        return 0; 
                    }
                    
                    // Checking if values are in proper range. Same for all optional arguments
                    providedArgs[5] = 1;
                }
                else if(providedArgs[5]) // Send an error message if an argument is provided more than once. Same for all optional arguments.
                {
                    Scierror(999, "Please provide optional arguments only once.\n"); 
                    return 0; 
                }
                else // Send an error message if name of argument is given but type is incorrect. Same for all optional arguments.
                {
                    Scierror(999, "Incorrect number of arguments provided. Please check the documentation for more information.\n"); 
                    return 0; 
                }
            }

    }  



    /// End of error check and input get;

    hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());

    ///virtual void cv::HOGDescriptor::detectMultiScale 
        // ( InputArray  img, 
        //   std::vector< Rect > &   foundLocations,
        //   double  hitThreshold = 0,
        //   Size  winStride = Size(),
        //   Size  padding = Size(),
        //   double  scale = 1.05,
        //   double  finalThreshold = 2.0,
        //   bool  useMeanshiftGrouping = false 
        // ) 

    sciprint("hitThreshold:%f\n",hitThreshold);
    sciprint("scale%f\n",scale);
    sciprint("finalThreshold:%f\n",finalThreshold);
    sciprint("winStride[0] :%f;winStride[1]:%f;\n",winStride[0],winStride[1]);
    sciprint("padding[0] :%f;padding[1]:%f;\n",padding[0],padding[1]);

    try
    {
        hog.detectMultiScale(image, found, hitThreshold, Size(padding[0],padding[1]), Size(winStride[0],winStride[1]), scale, finalThreshold, false);
    }

    catch(cv::Exception&e)
    {
      const char* err=e.what();
      Scierror(999,e.what());
    }
        /// finding out the rectangles   
    for (i = 0; i < found.size(); ++i)
    {
        Rect r = found[i];
        for (j = 0; j < found.size(); ++j)
            /// removing same identified people
            if (j != i and (r & found[j]) == r)
                break;

        /// if no duplicate found
        if (j == found.size())
            found_filtered.push_back(r);
    }

        /// filtering out the rectangles
    for (i=0; i<found_filtered.size(); i++){
        Rect r = found_filtered[i];
        
        r.x += cvRound(r.width*0.1);
        r.y += cvRound(r.height*0.07);

        r.width = cvRound(r.width*0.8);
        r.height = cvRound(r.height*0.8);

        answer.push_back(r);
            
        }

///  bboxex[i] = [x y width height];
    bboxes = (double*)malloc(sizeof(double) * (int)answer.size() * 4);

    int total_boxes = (int)answer.size();
    
    for(i = 0; i < answer.size(); ++i){
      Rect temp = answer[i];

      // x coordinate
      bboxes[i + 0 * total_boxes] = temp.x;

      // y coordinate
      bboxes[i + 1 * total_boxes] = temp.y;

      // width
      bboxes[i + 2 * total_boxes] = temp.width;

      // height
      bboxes[i + 3 * total_boxes] = temp.height;
    }



                                                                                            // rows  // columns
    sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 1, total_boxes, 4, bboxes);
    if(sciErr.iErr){
      printError(&sciErr, 0);
      return 0;
    }

    //Assigning the list as the Output Variable
    AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;

    //Returning the Output Variables as arguments to the Scilab environment
    ReturnArguments(pvApiCtx);
    return 0;

  }
/* ==================================================================== */
}
