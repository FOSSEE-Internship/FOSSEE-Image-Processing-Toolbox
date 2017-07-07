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

    int opencv_disparity(char *fname, unsigned long fname_len)
	{
		SciErr sciErr;

		/*----- Local Variables -----*/
	    int *piAddr = NULL;
	    int *piAddrVal = NULL;
	    int val_position = 0;
	    int *piLen = NULL;
	    int iRows = 0, iCols = 0;
	    char ** pstData = NULL;
	    char *currentArg = NULL;
	    int valOfCA = 0;
	    int argPresence[6];
	    for(int i = 0; i < 6; i++)
	        argPresence[i] = 0;
    	
    	/*----- Variables used in procesing -----*/
        Mat left_for_matcher, right_for_matcher;
    	Mat left_disp,right_disp;
    	Mat filtered_disp;
    	//conf_map = Scalar(255);
    	Rect ROI;
    	Ptr<DisparityWLSFilter> wls_filter;
    	int vis_mult = 1.0;
    	//char *dst_path = "./disparity.jpg"

	    /*----- Optional Input Arguments -----*/
	    char **method = NULL;
	    int numDisparities = 160;
	    int SADWindowSize = 7;
	    int uniquenessRatio = 0;
	    int textureThreshold = 507;
	    int disp12MaxDiff = 1;

	    int noOfarguments = *getNbInputArgument(pvApiCtx);
	    
	    /*----- Check for input and output arguments -----*/
	    CheckInputArgument(pvApiCtx, 2, 14);
    	CheckOutputArgument(pvApiCtx, 2, 2);

    	/*----- Retrieving two images -----*/
    	Mat image_1;
    	retrieveImage(image_1, 1);
    	image_1.convertTo(image_1, CV_8UC1);

    	Mat image_2;
    	retrieveImage(image_2, 2);
    	image_2.convertTo(image_2, CV_8UC1);
    	
    	if(image_1.channels() != 1)
    	{
        	Scierror(999, "Image 1 not a RGB image.\n");
        	return 0;        
    	}

    	if(image_2.channels() != 1)
    	{
        	Scierror(999, "Image 2 not a RGB image.\n");
        	return 0;        
    	}

    	if(noOfarguments % 2 != 0)
    	{
        	Scierror(999," Invalid No of Arguments \n");
        	return 0;
    	}

    	/*----- Retrieving Optional Input Arguments -----*/
    	for(int i = 3; i <= noOfarguments; i = i + 2)
    	{
	        sciErr = getVarAddressFromPosition(pvApiCtx,i,&piAddr);
	        if (sciErr.iErr)
	        {
	            printError(&sciErr, 0);
	            return 0;
	        }

	        if(!isStringType(pvApiCtx, piAddr))
	        {
	            Scierror(999,"Invalid Argument\n");
	            return 0;
	        }

	        // First call to get rows and columns 
	        sciErr = getMatrixOfString(pvApiCtx, piAddr, &iRows, &iCols, NULL, NULL);
	        if(sciErr.iErr)
	        {
	            printError(&sciErr, 0);
	            return 0;
	        }
	        piLen = (int*)malloc(sizeof(int) * iRows * iCols);
	        
	        // Second call to retrieve length of each string
	        sciErr = getMatrixOfString(pvApiCtx, piAddr, &iRows, &iCols, piLen, NULL);
	        if(sciErr.iErr)
	        {
	            printError(&sciErr, 0);
	            return 0;
	        }
	        pstData = (char**)malloc(sizeof(char*) * iRows * iCols);
	        for(int j = 0 ; j < iRows * iCols ; j++)
	        {
	            pstData[j] = (char*)malloc(sizeof(char) * (piLen[j] + 1));//+ 1 for null termination
	        }   
	        
	        // Third call to retrieve data
	        sciErr = getMatrixOfString(pvApiCtx, piAddr, &iRows, &iCols, piLen, pstData);
	        if(sciErr.iErr)
	        {
	            printError(&sciErr, 0);
	            return 0;
	        }

	        currentArg = pstData[0];
	        free(pstData);
	        iRows = 0;
	        iCols = 0;
	        free(piLen);

	        if(strcmp(currentArg, "numDisparities") == 0)
        	{
	            val_position = i + 1;  
	            
	            if(argPresence[0] == 1)
	            {
	                Scierror(999,"Do not enter the same parameter\n");
	                return 0;
	            }
		
	            sciErr = getVarAddressFromPosition(pvApiCtx, val_position, &piAddrVal);
	            if (sciErr.iErr)
	            {
	                printError(&sciErr, 0);
	                return 0;   
	            }

	            if(!isIntegerType(pvApiCtx, piAddrVal) ||
	               isVarComplex(pvApiCtx,piAddrVal) || 
	               !isScalar(pvApiCtx, piAddrVal))
	            {
	                Scierror(999," Invalid Value for numDisparities. Please enter a non negative Integer value.\n");
	                return 0;
	    	    }
					
	       	    getScalarInteger32(pvApiCtx, piAddrVal, &numDisparities);
	                  
	            if(numDisparities < 0 && numDisparities % 16 != 0)
	            {
	                Scierror(999, "Invalid Value for numDisparities. Please enter a non negative Integer value which is divisible by 16.\n");
	                return 0;
	            }                
	            argPresence[0] = 1;       
        	}

        	if(strcmp(currentArg, "SADWindowSize") == 0)
        	{
	            val_position = i + 1;  
	            
	            if(argPresence[1] == 1)
	            {
	                Scierror(999,"Do not enter the same parameter\n");
	                return 0;
	            }
		
	            sciErr = getVarAddressFromPosition(pvApiCtx, val_position, &piAddrVal);
	            if (sciErr.iErr)
	            {
	                printError(&sciErr, 0);
	                return 0;   
	            }

	            if(!isIntegerType(pvApiCtx, piAddrVal) ||
	               isVarComplex(pvApiCtx,piAddrVal) || 
	               !isScalar(pvApiCtx, piAddrVal))
	            {
	                Scierror(999, "Invalid Value for SADWindowSize. Please enter a non negative Integer value.\n");
	                return 0;
	    	    }
					
	       	    getScalarInteger32(pvApiCtx, piAddrVal, &SADWindowSize);
	                  
	            if(SADWindowSize < 1 && SADWindowSize % 2 == 0)
	            {
	                Scierror(999, "Invalid Value for SADWindowSize. Please enter a non negative Integer value, which is odd and greater than 1.\n");
	                return 0;
	            }                
	            argPresence[1] = 1;       
        	}

        	if(strcmp(currentArg, "Method") == 0)
			{
				if(argPresence[2] != 0)
				{
					Scierror(999,"Method Argument has been called twice\n");
					return 0;
				}
				
				val_position = i + 1;

				sciErr = getVarAddressFromPosition(pvApiCtx, val_position, &piAddrVal);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				// Check for Argument type
				if(!isStringType(pvApiCtx, piAddrVal))
				{
					Scierror(999, "%s: Wrong type of 2nd argument #%d. A string is expected.\n", fname, 1);
					return 0;
				}
				
				// Matrix of Stings
				sciErr = getMatrixOfString(pvApiCtx, piAddrVal, &iRows, &iCols, NULL, NULL);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				piLen = (int*)malloc(sizeof(int) * iRows * iCols);
				
				// Second call to retrieve the length of the string
				sciErr = getMatrixOfString(pvApiCtx, piAddrVal, &iRows, &iCols, piLen, NULL);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					free(piLen);
					return 0;
				}
				method = (char**)malloc(sizeof(char*) * iRows * iCols);
				for(int j = 0; j < iRows * iCols; j++)
				{
					method[j] = (char*)malloc(sizeof(char) * (piLen[j] + 1));
				}
				
				// Third call to retrieve data
				sciErr = getMatrixOfString(pvApiCtx, piAddrVal, &iRows, &iCols, piLen, method);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					free(piLen);
					free(method);
					return 0;
				}
				argPresence[2] = 1;
			}

			if(strcmp(currentArg, "uniquenessRatio") == 0)
        	{
	            val_position = i + 1;  
	            
	            if(argPresence[3] == 1)
	            {
	                Scierror(999,"Do not enter the same parameter\n");
	                return 0;
	            }
		
	            sciErr = getVarAddressFromPosition(pvApiCtx, val_position, &piAddrVal);
	            if (sciErr.iErr)
	            {
	                printError(&sciErr, 0);
	                return 0;   
	            }

	            if(!isIntegerType(pvApiCtx, piAddrVal) ||
	               isVarComplex(pvApiCtx,piAddrVal) || 
	               !isScalar(pvApiCtx, piAddrVal))
	            {
	                Scierror(999," Invalid Value for uniquenessRatio. Please enter a non negative Integer value.\n");
	                return 0;
	    	    }
					
	       	    getScalarInteger32(pvApiCtx, piAddrVal, &uniquenessRatio);
	                  
	            if(uniquenessRatio < 0)
	            {
	                Scierror(999," Invalid Value for uniquenessRatio. Please enter a non negative Integer value.\n");
	                return 0;
	            }                
	            argPresence[3] = 1;       
        	}

        	if(strcmp(currentArg, "textureThreshold") == 0)
        	{
	            val_position = i + 1;  
	            
	            if(argPresence[4] == 1)
	            {
	                Scierror(999,"Do not enter the same parameter\n");
	                return 0;
	            }
		
	            sciErr = getVarAddressFromPosition(pvApiCtx, val_position, &piAddrVal);
	            if (sciErr.iErr)
	            {
	                printError(&sciErr, 0);
	                return 0;   
	            }

	            if(!isIntegerType(pvApiCtx, piAddrVal) ||
	               isVarComplex(pvApiCtx,piAddrVal) || 
	               !isScalar(pvApiCtx, piAddrVal))
	            {
	                Scierror(999," Invalid Value for textureThreshold. Please enter a non negative Integer value.\n");
	                return 0;
	    	    }
					
	       	    getScalarInteger32(pvApiCtx, piAddrVal, &textureThreshold);
	                  
	            if(textureThreshold < 0)
	            {
	                Scierror(999," Invalid Value for textureThreshold. Please enter a non negative Integer value.\n");
	                return 0;
	            }                
	            argPresence[4] = 1;       
        	}

        	if(strcmp(currentArg, "disp12MaxDiff") == 0)
        	{
	            val_position = i + 1;  
	            
	            if(argPresence[5] == 1)
	            {
	                Scierror(999,"Do not enter the same parameter\n");
	                return 0;
	            }
		
	            sciErr = getVarAddressFromPosition(pvApiCtx, val_position, &piAddrVal);
	            if (sciErr.iErr)
	            {
	                printError(&sciErr, 0);
	                return 0;   
	            }

	            if(!isIntegerType(pvApiCtx, piAddrVal) ||
	               isVarComplex(pvApiCtx,piAddrVal) || 
	               !isScalar(pvApiCtx, piAddrVal))
	            {
	                Scierror(999," Invalid Value for disp12MaxDiff. Please enter a non negative Integer value.\n");
	                return 0;
	    	    }
					
	       	    getScalarInteger32(pvApiCtx, piAddrVal, &disp12MaxDiff);
	                  
	            if(disp12MaxDiff < 0)
	            {
	                Scierror(999," Invalid Value for disp12MaxDiff. Please enter a non negative Integer value.\n");
	                return 0;
	            }                
	            argPresence[5] = 1;       
        	}
	    }

	    if(argPresence[2] == 0)
	    {
	    	method = (char**)malloc(sizeof(char*) * 1 * 1);
			method[0] = (char*)malloc(sizeof(char) * 13);
			strcpy(method[0],"blockMatching");
	    }

	    /*----- Matching -----*/
	    if(strcmp(method[0], "blockMatching") == 0)
		{
			Ptr<StereoBM> left_matcher = StereoBM::create(numDisparities, SADWindowSize);
			left_matcher -> setTextureThreshold(textureThreshold);
			left_matcher -> setUniquenessRatio(uniquenessRatio);
			left_matcher -> setDisp12MaxDiff(disp12MaxDiff);
			wls_filter = createDisparityWLSFilter(left_matcher);
            Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);
            try
            {
            	sciprint("\n%d %d\n", image_1.rows, image_1.cols);
            	left_matcher -> compute(image_1, image_2,left_disp);
            	right_matcher -> compute(image_2, image_1, right_disp);
            }
            catch(Exception &e)
            {	
            	const char *err = e.what();
            	Scierror(999, "%s", err);
            	return 0;
            }
		}

		else if(strcmp(method[0], "semi-blockMatching") == 0)
		{
			Ptr<StereoSGBM> left_matcher  = StereoSGBM::create(0, numDisparities, SADWindowSize);
            left_matcher -> setP1(24 * SADWindowSize * SADWindowSize);
            left_matcher -> setP2(96 * SADWindowSize * SADWindowSize);
            left_matcher -> setPreFilterCap(63);
            left_matcher -> setMode(StereoSGBM::MODE_SGBM_3WAY);
            left_matcher -> setDisp12MaxDiff(disp12MaxDiff);
			left_matcher -> setUniquenessRatio(uniquenessRatio);
            wls_filter = createDisparityWLSFilter(left_matcher);
            Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);
            try
            {
            	left_matcher -> compute(image_1, image_2, left_disp);
            	right_matcher -> compute(image_2, image_1, right_disp);
            }
            catch(Exception &e)
            {	
            	const char *err = e.what();
            	Scierror(999, "%s", err);
            	return 0;
            }
		}

		/*----- Filtering -----*/
		int lambda = 8000;
		int sigma = 1.5;
		wls_filter->setLambda(lambda);
        wls_filter->setSigmaColor(sigma);
        try
        {
        	wls_filter->filter(left_disp, image_1, filtered_disp, right_disp);
        }
        catch(Exception &e)
        {	
        	const char *err = e.what();
        	Scierror(999, "%s", err);
        	return 0;
        }

        //Mat conf_map = Mat(left.rows, left.cols, CV_8U);	
        //conf_map = wls_filter->getConfidenceMap();

        Mat filtered_disp_vis;
        try
        {
        	getDisparityVis(filtered_disp, filtered_disp_vis, vis_mult);
        }
        catch(Exception &e)
        {	
        	const char *err = e.what();
        	Scierror(999, "%s", err);
        	return 0;
        }

        try
		{
			left_disp.convertTo(left_disp, CV_8U, 255 / (numDisparities * 16.));
		}
		catch(Exception &e)
        {	
        	const char *err = e.what();
        	Scierror(999, "%s", err);
        	return 0;
        }
        
        // imwrite(dst_path, filtered_disp_vis);

        /*----- Creating output images	 -----*/
        string tempstring = type2str(filtered_disp_vis.type());
		char *checker;
		checker = (char *)malloc(tempstring.size() + 1);
		memcpy(checker, tempstring.c_str(), tempstring.size() + 1);
		returnImage(checker, filtered_disp_vis, 1);
		free(checker);

		tempstring = type2str(left_disp.type());
		checker = (char *)malloc(tempstring.size() + 1);
		memcpy(checker, tempstring.c_str(), tempstring.size() + 1);
		returnImage(checker, left_disp, 2);
		free(checker);

		/*----- Returning output -----*/
		AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
		AssignOutputVariable(pvApiCtx, 2) = nbInputArgument(pvApiCtx) + 2;
		ReturnArguments(pvApiCtx);	
		return 0;
 	}
}