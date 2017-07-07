#include <numeric>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <iostream>

using namespace cv;
using namespace std;

extern "C"
{
    #include "api_scilab.h"
    #include "Scierror.h"
    #include "BOOL.h"
    #include <localization.h>
    #include <sciprint.h>
    #include "../common.h"
  
    int opencv_detectAndComputeORB(char *fname, unsigned long fname_len)
    {

        // Error management variable
        SciErr sciErr;
        
        //Variables
        int i, j;
        int intErr;
        int iRows1 = 0;
        int iCols1 = 0;
    	
    	int iRows = 0;
        int iCols = 0;
        int *piLen = NULL;
        int *piAddr = NULL;
        char **pstData = NULL;
        char *currentArg = NULL;
        bool *providedArgs = NULL; 
        
        double edgeThreshold = 20;
        double fastThreshold = 31;
        double firstLevel = 0;
        double maxFeatures = 80;
        double nLevels = 8;
        double patchSize = 30;
        double scaleFactor = 1.5f;
        double scoreType = ORB::HARRIS_SCORE;
        int WTA_K = 2;

        double *ROImat = NULL;
        int iRet;

        // checking input argument
        // 1 : image
        // 2 : edgeThreshold 
        // 3 : fastThreshold
        // 4 : firstLevel
        // 5 : ROI
        // 6 : maxFeatures
        // 7 : nLevels
        // 8 : patchSize
        // 9 : scaleFactor
        // 10 : scoreType
        CheckInputArgument(pvApiCtx, 1, 19);
        CheckOutputArgument(pvApiCtx, 1, 1);

        // retrieve image 
        Mat image;
        retrieveImage(image, 1);

        // For the optional arguments
        int nbInputArguments = *getNbInputArgument(pvApiCtx);
        //sciprint("%d\n",nbInputArguments);
        
        if((nbInputArguments%2) == 0)
        {
        	Scierror(999, "%d:  The number of arguments must be odd\n");
            return 0;
        }
        
        providedArgs = (bool*) malloc(sizeof(bool) * 9); 
        memset(providedArgs, 0, 9);

        for(int iter = 2; iter <= nbInputArguments; iter += 2)
        {
            sciErr = getVarAddressFromPosition(pvApiCtx, iter, &piAddr); 
            if (sciErr.iErr)
            {
                printError(&sciErr, 0); 
                return 0; 
            }

            /// Three calls to getMatrixOfString
            //First Call
            sciErr = getMatrixOfString(pvApiCtx, piAddr, &iRows, &iCols, NULL, NULL); 
            if (sciErr.iErr)
            {
                printError(&sciErr, 0); 
                return 0; 
            }
            
            // Second call to get length of string
            piLen = (int*) malloc(sizeof(int) * iRows1 * iCols1); 
            sciErr = getMatrixOfString(pvApiCtx,  piAddr,  &iRows1,  &iCols1,  piLen,  NULL); 
            if (sciErr.iErr)
            {
                printError(&sciErr, 0); 
                return 0; 
            }
            
            // third call
            pstData = (char**) malloc(sizeof(char*) * iRows1 * iCols1); 
            for(int k = 0; k < iRows1 * iCols1; ++k)
            {
                pstData[k] = (char*) malloc(sizeof(char) * piLen[k] + 1); 
            }
            
            sciErr = getMatrixOfString(pvApiCtx, piAddr, &iRows1, &iCols1, piLen, pstData); 
            if (sciErr.iErr)
            {
                printError(&sciErr, 0); 
                return 0; 
            }
            currentArg = pstData[0];

            // getting edgeThreshold
            if(strcmp(currentArg, "edgeThreshold") == 0)
            {
                if(iter + 1 <= nbInputArguments and !providedArgs[0])
                {
                    if(isIntegerType(pvApiCtx, piAddr) || isDoubleType(pvApiCtx, piAddr))
                    {
                        if(isScalar(pvApiCtx, piAddr))
                        {
                            double dData  = 0;
                            iRet = getScalarDouble(pvApiCtx, piAddr, &dData);
                            if(!iRet)
                            {
                                edgeThreshold = dData;
                            }
                        }
                        else
                        {
                            Scierror(999, "Error: The input argument ""edgeThreshold"" is not of scalar type.\n");
                            return 0;
                        }
                    }
                    else 
                    {
                        Scierror(999, "Error: The input argument ""edgeThreshold"" is not of type Integer.\n");
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

            if(strcmp(currentArg, "fastThreshold") == 0)
            {
                if(iter + 1 <= nbInputArguments and !providedArgs[1])
                {
                    if(isIntegerType(pvApiCtx, piAddr) || isDoubleType(pvApiCtx, piAddr))
                    {
                        if(isScalar(pvApiCtx, piAddr))
                        {
                            double dData  = 0;
                            iRet = getScalarDouble(pvApiCtx, piAddr, &dData);
                            if(!iRet)
                            {
                                fastThreshold = dData;
                            }
                        }
                        else
                        {
                            Scierror(999, "Error: The input argument ""fastThreshold"" is not of scalar type.\n");
                            return 0;
                        }
                    }
                    else 
                    {
                        Scierror(999, "Error: The input argument ""fastThreshold"" is not of type Integer.\n");
                        return 0;
                    }
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

            if(strcmp(currentArg, "firstLevel") == 0)
            {
                if(iter + 1 <= nbInputArguments and !providedArgs[2])
                {
                    if(isIntegerType(pvApiCtx, piAddr) || isDoubleType(pvApiCtx, piAddr))
                    {
                        if(isScalar(pvApiCtx, piAddr))
                        {
                            double dData  = 0;
                            iRet = getScalarDouble(pvApiCtx, piAddr, &dData);
                            if(!iRet)
                            {
                                firstLevel = dData;
                            }
                        }
                        else
                        {
                            Scierror(999, "Error: The input argument ""firstLevel"" is not of scalar type.\n");
                            return 0;
                        }
                    }
                    else 
                    {
                        Scierror(999, "Error: The input argument ""firstLevel"" is not of type Integer.\n");
                        return 0;
                    }
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

            else if(strcmp(currentArg,"ROI")==0)
            {
                // val_position=i+1;
                
                if(providedArgs[3] == 1)
                {
                    Scierror(999,"Do not enter the same parameter\n");
                    return 0;
                }
                
                sciErr = getVarAddressFromPosition(pvApiCtx, i + 1, &piAddr);
                if (sciErr.iErr)
                {
                    printError(&sciErr, 0);
                    return 0;
                }

                if(!isDoubleType(pvApiCtx, piAddr) || isVarComplex(pvApiCtx, piAddr))
                {
                    Scierror(999,"Enter a List of 4 arguments\n");
                    return 0;
                }

                sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &iRows, &iCols, &ROImat);  
                if(sciErr.iErr)
                {
                    printError(&sciErr, 0);
                    return 0;
                }

                if(iRows*iCols!=4)
                {   
                    Scierror(999,"Invalid Argument\n");
                    return 0;
                }

                if(ROImat[0]<0 || ROImat[1]<0 || ROImat[2]<0 || ROImat[3]<0)
                {   
                    Scierror(999,"Arguments cannot be negative\n");
                    return 0;
                }       

                if(ROImat[0]>image.cols || ROImat[1]>image.rows || ROImat[0]+ROImat[2]>image.cols
                    || ROImat[1]+ROImat[3]>image.rows) 
                {
                    Scierror(999,"Please make sure the arguments are within the image range\n");
                    return 0;
                }

                providedArgs[3] = 1;
            }

            if(strcmp(currentArg, "maxFeatures") == 0)
            {
                if(iter + 1 <= nbInputArguments and !providedArgs[4])
                {
                    if(isIntegerType(pvApiCtx, piAddr) || isDoubleType(pvApiCtx, piAddr))
                    {
                        if(isScalar(pvApiCtx, piAddr))
                        {
                            double dData  = 0;
                            iRet = getScalarDouble(pvApiCtx, piAddr, &dData);
                            if(!iRet)
                            {
                                maxFeatures = dData;
                            }
                        }
                        else
                        {
                            Scierror(999, "Error: The input argument ""maxFeatures"" is not of scalar type.\n");
                            return 0;
                        }
                    }
                    else 
                    {
                        Scierror(999, "Error: The input argument ""maxFeatures"" is not of type Integer.\n");
                        return 0;
                    }
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

            if(strcmp(currentArg, "nLevels") == 0)
            {
                if(iter + 1 <= nbInputArguments and !providedArgs[5])
                {
                    if(isIntegerType(pvApiCtx, piAddr) || isDoubleType(pvApiCtx, piAddr))
                    {
                        if(isScalar(pvApiCtx, piAddr))
                        {
                            double dData  = 0;
                            iRet = getScalarDouble(pvApiCtx, piAddr, &dData);
                            if(!iRet)
                            {
                                nLevels = dData;
                            }
                        }
                        else
                        {
                            Scierror(999, "Error: The input argument ""nLevels"" is not of scalar type.\n");
                            return 0;
                        }
                    }
                    else 
                    {
                        Scierror(999, "Error: The input argument ""nLevels"" is not of type Integer.\n");
                        return 0;
                    }
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

            if(strcmp(currentArg, "patchSize") == 0)
            {
                if(iter + 1 <= nbInputArguments and !providedArgs[6])
                {
                    if(isIntegerType(pvApiCtx, piAddr) || isDoubleType(pvApiCtx, piAddr))
                    {
                        if(isScalar(pvApiCtx, piAddr))
                        {
                            double dData  = 0;
                            iRet = getScalarDouble(pvApiCtx, piAddr, &dData);
                            if(!iRet)
                            {
                                patchSize = dData;
                            }
                        }
                        else
                        {
                            Scierror(999, "Error: The input argument ""patchSize"" is not of scalar type.\n");
                            return 0;
                        }
                    }
                    else 
                    {
                        Scierror(999, "Error: The input argument ""patchSize"" is not of type Integer.\n");
                        return 0;
                    }
                    providedArgs[6] = 1; 
                }
                else if(providedArgs[6]) // Send an error message if an argument is provided more than once. Same for all optional arguments.
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

            if(strcmp(currentArg, "scaleFactor") == 0)
            {
                if(iter + 1 <= nbInputArguments and !providedArgs[7])
                {
                    // if(isDoubleType(pvApiCtx, piAddr))
                    // {
                        if(isScalar(pvApiCtx, piAddr))
                        {
                            double dData  = 0;
                            iRet = getScalarDouble(pvApiCtx, piAddr, &dData);
                            if(!iRet)
                            {
                                scaleFactor = dData;
                            }
                        }
                        else
                        {
                            Scierror(999, "Error: The input argument ""scaleFactor"" is not of scalar type.\n");
                            return 0;
                        }
                    // }
                    // else 
                    // {
                    //     Scierror(999, "Error: The input argument ""scaleFactor"" is not of type Integer.\n");
                    //     return 0;
                    // }
                    providedArgs[7] = 1; 
                }
                else if(providedArgs[7]) // Send an error message if an argument is provided more than once. Same for all optional arguments.
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

            if(strcmp(currentArg, "scoreType") == 0)
            {
                if(iter + 1 <= nbInputArguments and !providedArgs[8])
                {
                    if(isIntegerType(pvApiCtx, piAddr) || isDoubleType(pvApiCtx, piAddr))
                    {
                        if(isScalar(pvApiCtx, piAddr))
                        {
                            double dData  = 0;
                            iRet = getScalarDouble(pvApiCtx, piAddr, &dData);
                            if(!iRet)
                            {
                                if(dData == 0 || dData == 1 || dData == 32)
                                {
                                    Scierror(999, "Error: The input argument ""scoreType"" should be 0, 1 or 32.\n");
                                    return 0;
                                }
                                scoreType = dData;
                            }
                        }
                        else
                        {
                            Scierror(999, "Error: The input argument ""scoreType"" is not of scalar type.\n");
                            return 0;
                        }
                    }
                    else 
                    {
                        Scierror(999, "Error: The input argument ""scoreType"" is not of type Integer.\n");
                        return 0;
                    }
                    providedArgs[8] = 1; 
                }
                else if(providedArgs[8]) // Send an error message if an argument is provided more than once. Same for all optional arguments.
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

        /// Creating model and setting its params;
        Ptr<ORB> orb = ORB::create(maxFeatures, scaleFactor, nLevels, 
                                   edgeThreshold, firstLevel, WTA_K, 
                                   scoreType, patchSize, fastThreshold);
        
        sciprint("\nET:%d\n", orb->getEdgeThreshold());
        /// to store the keypoints of orb detected
        vector<KeyPoint> keyPoints;

        // if(providedArgs[0])
        // {
        //     orb->setEdgeThreshold(edgeThreshold);
        // }
        // if(providedArgs[1])
        // {
        //     orb->setFastThreshold(fastThreshold);
        // }
        // if(providedArgs[2])
        // {
        //     orb->setFirstLevel(firstLevel);
        // }

        // if(providedArgs[4])
        // {
        //     orb->setMaxFeatures(maxFeatures);
        // }
        // if(providedArgs[5])
        // {
        //     orb->setNLevels(nLevels);
        // }
        // if(providedArgs[6])
        // {
        //     orb->setPatchSize(patchSize);
        // }
        // if(providedArgs[7])
        // {
        //     orb->setScaleFactor(scaleFactor);
        // }
        // if(providedArgs[8])
        // {
        //     orb->setScoreType(scoreType);
        // }

        // Handling ROI argument
        if(providedArgs[3]==0)
        {
            ROImat=(double *)malloc(sizeof(double)*1*4);
            ROImat[0]=0;
            ROImat[1]=0;
            ROImat[2]=image.cols;
            ROImat[3]=image.rows;       
        }

        Rect masker(ROImat[0], ROImat[1], ROImat[2], ROImat[3]);
        Mat mask(image.size(), CV_8UC1, Scalar::all(0));
        
        // Creating ROI
        try
        {
            mask(masker).setTo(Scalar::all(255));
        }
        catch(Exception &e)
        {
          const char *err = e.what();
          Scierror(999, "%s", err);
        }

        Mat descriptor;

        try
        {
            image.convertTo(image, CV_8U);
            // mask.rows -= 500;
            // mask.cols -= 500;
            // sciprint("M:%d %d I:%d %d", mask.rows, mask.cols, image.rows, image.cols);
            orb->detectAndCompute(image, mask, keyPoints, descriptor);

            // orb->detect(image, keyPoints);
            // orb->compute(image, keyPoints, descriptor);
            // int dcols = descriptor.cols;
            // for(int i = 0; i < dcols; i++)
            // {
            //     sciprint("%d ", descriptor.at<uchar>(i));
            // }
            // sciprint("%d %d", keyPoints[104].pt.x, keyPoints[104].pt.y);
        }
        catch(Exception &e)
        {
            const char *err = e.what();
            Scierror(999, "%s", err);
            return 0;
        }

        int total_keypoints = (int)keyPoints.size();

        double *key_value = (double*)malloc(sizeof(double) * (int)keyPoints.size() * 2);

        vector< KeyPoint >::iterator it;
        i = 0;
        for(it = keyPoints.begin(); it != keyPoints.end(); ++it)
        {
          KeyPoint temp = keyPoints[i];

          // x coordinate
          key_value[i + 0 * total_keypoints] = it->pt.x;

          // y coordinate
          key_value[i + 1 * total_keypoints] = it->pt.y;
          i++;
        }

        // Converting descriptor from Mat to double*
        int dcols = descriptor.cols;
        int drows = descriptor.rows;
        // for(int i = 0; i < dcols; i++)
        // {
        //     sciprint("%f ", descriptor.at<double>(i));
        // }
        double *_descriptors = (double*) malloc(sizeof(double) * dcols * drows);
        
        for(int i = 0; i < drows * dcols; i++)
        {
            _descriptors[i] = int(descriptor.at<uchar>(i));
        }

        // Creating output arguments
        sciErr = createList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, 2, &piAddr);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }

        sciErr = createMatrixOfDoubleInList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, piAddr, 1, total_keypoints, 2, key_value);
        if(sciErr.iErr)
        {
          printError(&sciErr, 0);
          return 0;
        }

        sciErr = createMatrixOfDoubleInList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, piAddr, 2, drows, dcols, _descriptors);
        if(sciErr.iErr)
        {
          printError(&sciErr, 0);
          return 0;
        }

        //Assigning the list as the Output Variable
        AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;

        //Returning the Output Variables as arguments to the Scilab environment
        ReturnArguments(pvApiCtx);
        return 0;
    }
}