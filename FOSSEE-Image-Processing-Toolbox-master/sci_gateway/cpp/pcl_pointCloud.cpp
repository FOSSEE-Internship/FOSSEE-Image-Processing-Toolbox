/***************************************************
Authors : Gursimar Singh & Siddhant Narang
***************************************************/
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// #include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;
using namespace pcl;

extern "C"
{
	#include "api_scilab.h"
	#include "Scierror.h"
	#include "BOOL.h"
	#include <localization.h>
	#include "sciprint.h"
	// #include "../common.h"

	int pcl_pointCloud(char *fname, unsigned long fname_len)
	{
	  	/*----- Error management variables -----*/
	    SciErr sciErr;
	    int intErr;

	  	CheckInputArgument(pvApiCtx, 1, 13);
	    CheckOutputArgument(pvApiCtx, 1, 8);

	    /*---- Local Variables -----*/
	    int *piAddr = NULL;
	    int iRows = 0, iCols = 0;
	    int *piLen = NULL;
	    char **pstData = NULL;
	    int providedArgs[6];
	    for(int i = 0; i < 6; i++)
	    	providedArgs[i] = 0;
	    char *currentArg;

	    /*----- Variables used in processing -----*/
	    double *pointsColorMatrix = NULL;
	    double *normalMatrix = NULL;
	    double *intensityMatrix = NULL;
	    double *pointCoordinates = NULL;
	    double numberOfPoints = 0;
	    double *XLimits = NULL;
	    double *YLimits = NULL;
	    double *ZLimits = NULL;
	    int width = 0;
	    //PointCloud<PointXYZ> cloud;

	    // Handling the first input arg
	    sciErr = getVarAddressFromPosition(pvApiCtx, 1, &piAddr);
	    if (sciErr.iErr)
	    {
	        printError(&sciErr, 0);
	        return 0;
	    }
	    
	    // Taking matrix as input
	    if(isDoubleType(pvApiCtx, piAddr))
	    {
	    	sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &iRows, &iCols, &pointCoordinates);
	        if (sciErr.iErr)
	        {
	            printError(&sciErr, 0);
	            return 0;
	        }
	        // Checking if matrix is of proper format
	        if(iCols > 3)
	        {
	            Scierror(999, "Error: Please provide proper value for \"%s\". Number columns should be = 3.\n", currentArg);
	            return 0;
	        }
	        numberOfPoints = iRows;
	        width = iCols;
	    }

	    int nbInputArguments = *getNbInputArgument(pvApiCtx); // Nummber of input arguments
	    // The following loop is for checking if optional arguments have been provided
	    for(int iter = 2; iter <= nbInputArguments; iter++)
	    {
	        // Getting address of next argument
	        sciErr = getVarAddressFromPosition(pvApiCtx, iter, &piAddr);
	        if (sciErr.iErr)
	        {
	            printError(&sciErr, 0);
	            return 0;
	        }

	        // Extracting name of next argument takes three calls to getMatrixOfString
	        sciErr = getMatrixOfString(pvApiCtx, piAddr, &iRows, &iCols, NULL, NULL);
	        if (sciErr.iErr)
	        {
	            printError(&sciErr, 0);
	            return 0;
	        }

	        piLen = (int*) malloc(sizeof(int) * iRows * iCols);

	        // Second call to get the dimensions
	        sciErr = getMatrixOfString(pvApiCtx,  piAddr,  &iRows,  &iCols,  piLen,  NULL);
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

	        // Third call to get the elements
	        sciErr = getMatrixOfString(pvApiCtx, piAddr, &iRows, &iCols, piLen, pstData);
	        if (sciErr.iErr)
	        {
	            printError(&sciErr, 0);
	            return 0;
	        }

	        currentArg = pstData[0];
	        
	        // Extracting the "Value" from Name, Value.. pair
	        if(strcmp(currentArg, "Color") == 0)
	        {
	            //sciprint("%d", providedArgs[0]);
	            if(iter + 1 <= nbInputArguments && providedArgs[0] == 0)
	            {
	                sciErr = getVarAddressFromPosition(pvApiCtx, ++iter, &piAddr);
	                if (sciErr.iErr)
	                {
	                    printError(&sciErr, 0);
	                    return 0;
	                }
	                
	                // Taking matrix as input
	                if(isDoubleType(pvApiCtx, piAddr))
	                {
	                	sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &iRows, &iCols, &pointsColorMatrix);
		                if (sciErr.iErr)
				        {
				            printError(&sciErr, 0);
				            return 0;
				        }
		                // Checking if matrix is of proper format
		                if(iCols > 3)
		                {
		                    Scierror(999, "Error: Please provide proper value for \"%s\". Number columns should be = 3.\n", currentArg);
		                    return 0;
		                }
		                providedArgs[0] = 1;
	                }
	            }
	            else if(providedArgs[0] == 1) // Send an error message if an argument is provided more than once. Same for all optional arguments.
	            {
	                Scierror(999, "Please provide optional arguments only once.\n");
	                return 0;
	            }
	            else // Send an error message if name of argument is given but type is incorrect. Same for all optional arguments.
	            {
	                Scierror(999, "Incorrect number of arguments provided. Please check the documentation for more information.1\n");
	                return 0;
	            }
	        }

	        else if(strcmp(currentArg, "Normal") == 0)
	        {
	            if(iter + 1 <= nbInputArguments  && providedArgs[1] == 0)
	            {
	                sciErr = getVarAddressFromPosition(pvApiCtx, ++iter, &piAddr);
	                if (sciErr.iErr)
	                {
	                    printError(&sciErr, 0);
	                    return 0;
	                }
	                
	                if(isDoubleType(pvApiCtx, piAddr))
	                {
	                	sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &iRows, &iCols, &normalMatrix);
		                if (sciErr.iErr)
				        {
				            printError(&sciErr, 0);
				            return 0;
				        }
		                
		                // Checking if matrix is of proper format
		                if(iCols > 3)
		                {
		                    Scierror(999, "Error: Please provide proper value for \"%s\". Number columns should be = 3.\n", currentArg);
		                    return 0;
		                }
		                providedArgs[1] = 1;
	                }
	            }
	            else if(providedArgs[1] == 1)
	            {
	                Scierror(999, "Please provide optional arguments only once.\n");
	                return 0;
	            }
	            else
	            {
	                Scierror(999, "Incorrect number of arguments provided. Please check the documentation for more information.2\n");
	                return 0;
	            }
	        }

	        else if(strcmp(currentArg, "Intensity") == 0)
	        {
	            if(iter + 1 <= nbInputArguments && providedArgs[2] == 0)
	            {
	                sciErr = getVarAddressFromPosition(pvApiCtx, ++iter, &piAddr);
	                if (sciErr.iErr)
	                {
	                    printError(&sciErr, 0);
	                    return 0;
	                }
	                
	                if(!isDoubleType(pvApiCtx, piAddr) || isVarComplex(pvApiCtx, piAddr))
	                {
	                    Scierror(999, "%s: Wrong type for input argument #%d: A real matrix expected.\n", fname, iter);
	                    return 0;
	                }

	                // Taking matrix as input
	            	sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &iRows, &iCols, &intensityMatrix);
	                if (sciErr.iErr)
			        {
			            printError(&sciErr, 0);
			            return 0;
			        }
	                // Checking if matrix is of proper format
	                providedArgs[2] = 1;
	            }
	            else if(providedArgs[2] == 1)
	            {
	                Scierror(999, "Please provide optional arguments only once.\n");
	                return 0;
	            }
	            else
	            {
	                Scierror(999, "Incorrect number of arguments provided. Please check the documentation for more information.3\n");
	                return 0;
	            }
	        }

	        else if(strcmp(currentArg, "XLimits") == 0)
	        {
	            if(iter + 1 <= nbInputArguments && providedArgs[3] == 0)
	            {
	                sciErr = getVarAddressFromPosition(pvApiCtx, ++iter, &piAddr);
	                if (sciErr.iErr)
	                {
	                    printError(&sciErr, 0);
	                    return 0;
	                }
	                
	                if(!isDoubleType(pvApiCtx, piAddr) || isVarComplex(pvApiCtx, piAddr))
	                {
	                    Scierror(999, "%s: Wrong type for input argument #%d: A real matrix expected.\n", fname, iter);
	                    return 0;
	                }

	                // Taking matrix as input
	            	sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &iRows, &iCols, &XLimits);
	                if (sciErr.iErr)
			        {
			            printError(&sciErr, 0);
			            return 0;
			        }
	                // Checking if matrix is of proper format
	                if(iCols > 2 && iRows > 1)
	                {
	                	Scierror(999, "matrix should be of dimensions (1 x 2)");
	                	return 0;
	                }
	                providedArgs[3] = 1;
	            }
	            else if(providedArgs[3] == 1)
	            {
	                Scierror(999, "Please provide optional arguments only once.\n");
	                return 0;
	            }
	            else
	            {
	                Scierror(999, "Incorrect number of arguments provided. Please check the documentation for more information.4\n");
	                return 0;
	            }
	        }

	        else if(strcmp(currentArg, "YLimits") == 0)
	        {
	            if(iter + 1 <= nbInputArguments && providedArgs[4] == 0)
	            {
	                sciErr = getVarAddressFromPosition(pvApiCtx, ++iter, &piAddr);
	                if (sciErr.iErr)
			        {
			            printError(&sciErr, 0);
			            return 0;
			        }
	                
	                if(!isDoubleType(pvApiCtx, piAddr) || isVarComplex(pvApiCtx, piAddr))
	                {
	                    Scierror(999, "%s: Wrong type for input argument #%d: A real matrix expected.\n", fname, iter);
	                    return 0;
	                }

	                // Taking matrix as input
	            	sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &iRows, &iCols, &XLimits);
	                if (sciErr.iErr)
			        {
			            printError(&sciErr, 0);
			            return 0;
			        }
	                // Checking if matrix is of proper format
	                if(iCols > 2 && iRows > 1)
	                {
	                	Scierror(999, "matrix should be of dimensions (1 x 2)");
	                	return 0;
	                }
	                providedArgs[4] = 1;
	            }
	            else if(providedArgs[4] == 1)
	            {
	                Scierror(999, "Please provide optional arguments only once.\n");
	                return 0;
	            }
	            else
	            {
	                Scierror(999, "Incorrect number of arguments provided. Please check the documentation for more information.5\n");
	                return 0;
	            }
	        }

	        else if(strcmp(currentArg, "ZLimits") == 0)
	        {
	            if(iter + 1 <= nbInputArguments && providedArgs[5] == 0)
	            {
	                sciErr = getVarAddressFromPosition(pvApiCtx, ++iter, &piAddr);
	                if (sciErr.iErr)
	                {
	                    printError(&sciErr, 0);
	                    return 0;
	                }
	                
	                if(!isDoubleType(pvApiCtx, piAddr) || isVarComplex(pvApiCtx, piAddr))
	                {
	                    Scierror(999, "%s: Wrong type for input argument #%d: A real matrix expected.\n", fname, iter);
	                    return 0;
	                }

	                // Taking matrix as input
	            	sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &iRows, &iCols, &XLimits);
	                if (sciErr.iErr)
			        {
			            printError(&sciErr, 0);
			            return 0;
			        }
	                // Checking if matrix is of proper format
	                if(iCols > 2 && iRows > 1)
	                {
	                	Scierror(999, "matrix should be of dimensions (1 x 2)");
	                	return 0;
	                }
	                providedArgs[5] = 1;
	            }
	            else if(providedArgs[5] == 1)
	            {
	                Scierror(999, "Please provide optional arguments only once.\n");
	                return 0;
	            }
	            else
	            {
	                Scierror(999, "Incorrect number of arguments provided. Please check the documentation for more information.6\n");
	                return 0;
	            }
	        }
	    }
	    
	    sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 1, numberOfPoints, 3, pointCoordinates);
	    if(sciErr.iErr)
	    {
	      printError(&sciErr, 0);
	      return 0;
	    }
	    AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
	    sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 2, 1, 1, &numberOfPoints);
	    if(sciErr.iErr)
	    {
		 printError(&sciErr, 0);
		 return 0;
	    }
	    AssignOutputVariable(pvApiCtx, 2) = nbInputArgument(pvApiCtx) + 2;	    

	    if(pointsColorMatrix != NULL)
	    {
		sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 3, numberOfPoints, 3, pointsColorMatrix);
        	if(sciErr.iErr)
        	{
			printError(&sciErr, 0);
			return 0;
        	}
        	AssignOutputVariable(pvApiCtx, 3) = nbInputArgument(pvApiCtx) + 3;
	    }

            if(intensityMatrix != NULL)
            {
	    	sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 4, 1, numberOfPoints, intensityMatrix);
		if(sciErr.iErr)
		{
	              printError(&sciErr, 0);
	      	      return 0;
	    	}
	    	AssignOutputVariable(pvApiCtx, 4) = nbInputArgument(pvApiCtx) + 4;
	    }
	    if(normalMatrix != NULL)
	    {
	    	sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 5, numberOfPoints, 3, normalMatrix);
	    	if(sciErr.iErr)
	    	{
	      		printError(&sciErr, 0);
	      		return 0;
	    	}
	    	AssignOutputVariable(pvApiCtx, 5) = nbInputArgument(pvApiCtx) + 5;
	    }
	    if(XLimits != NULL)
	    {
		    sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 6, 1, 2, XLimits);
		    if(sciErr.iErr)
		    {
		      printError(&sciErr, 0);
		      return 0;
		    }
		    AssignOutputVariable(pvApiCtx, 6) = nbInputArgument(pvApiCtx) + 6;
            }
            if(YLimits != NULL)
            {
	    	sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 7, 1, 2, YLimits);
	    	if(sciErr.iErr)
	    	{
	      		printError(&sciErr, 0);
	      		return 0;
	    	}
	    	AssignOutputVariable(pvApiCtx, 7) = nbInputArgument(pvApiCtx) + 7;
	    }
	    if(ZLimits != NULL)
	    {
	    	sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 8, 1, 2, ZLimits);
	    	if(sciErr.iErr)
	    	{
	      		printError(&sciErr, 0);
	      		return 0;
	    	}
	    	AssignOutputVariable(pvApiCtx, 8) = nbInputArgument(pvApiCtx) + 8;
	    }

    		//Returning the Output Variables as arguments to the Scilab environment
    		ReturnArguments(pvApiCtx);

  		return 0;
  	}
}
