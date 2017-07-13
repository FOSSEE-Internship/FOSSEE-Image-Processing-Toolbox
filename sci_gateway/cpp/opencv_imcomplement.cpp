/********************************************************
Function   :imcomplement
Syntax     :B=imcomplement(A)
Author     : Shubham Lohakare, Tess Zacharias
********************************************************/

#include <numeric>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
#include "string.h"
using namespace cv;
using namespace std;
extern "C"
{
  	#include "api_scilab.h"
  	#include "Scierror.h"
  	#include "BOOL.h"
  	#include <localization.h>
  	#include "sciprint.h"
  	#include "../common.h"
  	int opencv_imcomplement(char *fname, unsigned long fname_len)
  	{
		//DEclaring variables	
    		SciErr sciErr;
    		Mat A,B,C;
    		int* piAddr2=NULL;
    		int iType = 0;
    		int iRet    = 0;
    		double value= 0;
    		
		CheckInputArgument(pvApiCtx, 1,1);//Check input arguments
    		CheckOutputArgument(pvApiCtx, 1, 1);//Check output arguments
    
		retrieveImage(A,1);//Retrieving the first input
		try{
    			cv::subtract(cv::Scalar::all(255),A,A);//Calculates the complement
		}
		catch(Exception& e)
		{
			const char* err=e.what();
			sciprint("%s",err);
		}		
    		int temp = nbInputArgument(pvApiCtx) + 1;
    		string tempstring = type2str(A.type());
    		char *checker;
    		checker = (char *)malloc(tempstring.size() + 1);
    		memcpy(checker, tempstring.c_str(), tempstring.size() + 1);
    		returnImage(checker,A,1);
    		free(checker); 
    		AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
    		ReturnArguments(pvApiCtx);
    		return 0;
  	}
}
