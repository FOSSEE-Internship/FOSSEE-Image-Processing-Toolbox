/* ==================================================================== */
/* Author :Priyanka Hiranandani NIT Surat, Ashish Manatosh Barik NIT Rourkela        */
/* ==================================================================== */
/* Syntax : return_area=contourarea(InputArray contour, bool oriented); */
/* ==================================================================== */
#include <numeric>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <sciprint.h>
//#include<bits/stdc++.h>

using namespace cv;
using namespace std;
	extern "C"
  	{
  		#include "api_scilab.h"
  		#include "Scierror.h"
  		#include "BOOL.h"
  		#include <localization.h>
  
		int opencv_contourArea(char *fname, unsigned long fname_len)
    		{
    			// Error management variable
        		SciErr sciErr;
    
			// variables required to read argument #1
			int iRows		= 0;
			int iCols		= 0;
			int* piAddr1		= NULL;
			double *pstData	        = NULL;
        		int error;
        
			// variables required to read argument #2
			int *piAddr2 = NULL;
			int intErr2 = 0;
			int orientation = false;
	
         		//checking input argument 
        		CheckInputArgument(pvApiCtx,2,2);
         		//checking output argument
        		CheckOutputArgument(pvApiCtx, 1, 1);
         		//for first argument 
         		
			// get Address of first input  
        		sciErr =getVarAddressFromPosition(pvApiCtx,1,&piAddr1);
          		//check for any error
             		if(sciErr.iErr)   
               		{
               			printError(&sciErr, 0);
               			return 0;
               		}      
           		//retrieve input array
          		//SciErr getMatrixOfDouble(void* _pvCtx, int* _piAddress, int* _piRows, int* _piCols, double** _pdblReal)
        		sciErr = getMatrixOfDouble(pvApiCtx, piAddr1, &iRows, &iCols,&pstData);
             		if(sciErr.iErr)
               		{
              			printError(&sciErr, 0);
               			return 0;
               		}
        		int k=0;
        		vector<Point> contours;
        		for(int i=0;i<iCols;i++)
               		{   
               			contours.push_back(Point2f(pstData[i],pstData[i+1])) ;   
               			i++;
               		}
    			
			// to get the argument #2
			sciErr = getVarAddressFromPosition(pvApiCtx, 2, &piAddr2);
			if(sciErr.iErr)
			{
				printError(&sciErr, 0);
				return 0;
			}
			intErr2 = getScalarBoolean(pvApiCtx, piAddr2, &orientation);
			if(intErr2)
			{
				Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 2);
				return -1;
			} 

			double res;
	
			try
			{
				res = contourArea(contours, orientation);

			}
			catch(Exception& e)
			{
				const char* err=e.what();
				Scierror(999, "%s", err);
			}			

        		error=createScalarDouble(pvApiCtx,nbInputArgument(pvApiCtx)+1,res);
         		if(error!=0)
         		{
           			Scierror(999, "error occurred");
           			return -1;    
         		}  

    ////////// Return the output arguments to the Scilab engine //////////

        		AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;

        		ReturnArguments(pvApiCtx);

        		return 0;     
		}
	}
