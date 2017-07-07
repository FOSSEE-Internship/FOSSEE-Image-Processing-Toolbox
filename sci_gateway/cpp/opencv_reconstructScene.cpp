/***************************************************
Author : Siddhant Narang
***************************************************/
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/ximgproc.hpp>
#include <iostream>
#include <string>


using namespace cv;
using namespace std;
using namespace cv::ximgproc;

extern "C"
{
    #include "api_scilab.h"
    #include "Scierror.h"
    #include "BOOL.h"
    #include <localization.h>
    #include "sciprint.h"
    #include "../common.h"

    int opencv_reconstructScene(char *fname, unsigned long fname_len)
	{
		SciErr sciErr;

		/*----- Local Variables -----*/
	    int *piAddr = NULL;
	    int *piLen = NULL;
	    int iRows = 0, iCols = 0;
	    int iType = 0;

	    /*----- Check for input and output arguments -----*/
	    CheckInputArgument(pvApiCtx, 3, 3);
    	CheckOutputArgument(pvApiCtx, 1, 1);

    	/*----- Taking input arguments -----*/
    	double *Q = NULL;
    	int handleMissigValues = 0;

    	
    	// Input Arguments one
    	sciErr = getVarAddressFromPosition(pvApiCtx, 1, &piAddr);
	    if (sciErr.iErr)
	    {
	        printError(&sciErr, 0);
	        return 0;
	    }

	    //check type
		sciErr = getVarType(pvApiCtx, piAddr, &iType);
		if(sciErr.iErr || iType != sci_matrix)
		{
			printError(&sciErr, 0);
			return 0;
		}

	    sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &iRows, &iCols, &Q);
	    if(sciErr.iErr)
	    {
	        printError(&sciErr, 0);
	        return 0;
	    }
	    
	    double temp[iCols][iRows];
	    int k = 0, j = 0;
	    for(int i = 0; i < iRows; i++)
	    {
	    	for(int i = 0; i < iRows; i++)
	    	{
	    		temp[i][j] = Q[(j * 4) + i];
	    	}	
	    }
	    Mat _Q(iRows, iCols, CV_8UC1, &temp);

	    // Input Arguments two
	    Mat _disparity;
    	retrieveImage(_disparity, 2);

		// Input Arguments three
	    sciErr = getVarAddressFromPosition(pvApiCtx, 3, &piAddr);
        if (sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;   
        }
			
   	    getScalarInteger32(pvApiCtx, piAddr, &handleMissigValues);
   	    sciprint("%d", handleMissigValues);
              
        if(handleMissigValues < 0 && handleMissigValues > 1)
        {
            Scierror(999, "Invalid Value for handleMissigValues. Please enter a non negative Integer value which is divisible by 16.\n");
            return 0;
        }    

        Mat image3DOCV;
        try
        {
        	reprojectImageTo3D(_disparity, image3DOCV, _Q, true, CV_64F);
        	sciprint("%d", image3DOCV.rows);
        }
        catch(Exception &e)
        {
        	Scierror(999, "%s", e.what());
        	return 0;
        }

        /*----- Creating output images	 -----*/
        string tempstring = type2str(image3DOCV.type());
		char *checker;
		checker = (char *)malloc(tempstring.size() + 1);
		memcpy(checker, tempstring.c_str(), tempstring.size() + 1);
		returnImage(checker, image3DOCV, 1);
		free(checker);
		
		/*----- Returning output -----*/
		AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
		ReturnArguments(pvApiCtx);
 		return 0;   	
    }
}