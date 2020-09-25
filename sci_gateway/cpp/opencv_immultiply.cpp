/********************************************************
Function   :immultiply
Syntax     :C=immultiply(A,B)
Author     : Tess  Zacharias, Shubham Lohakare
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
  	int opencv_immultiply(char *fname, unsigned long fname_len)
  	{
		//Declaring variables
    		SciErr sciErr;
    		Mat A,B,C;
    		int* piAddr2=NULL;
    		int iType = 0;
    		int iRet    = 0;
    		double value= 0;

    		CheckInputArgument(pvApiCtx, 2, 2);//Checking Input arguments
    		CheckOutputArgument(pvApiCtx, 1, 1);//Checking Output arguments
    
		retrieveImage(A,1);//Retrieving the first input
    
		sciErr =getVarAddressFromPosition(pvApiCtx,2,&piAddr2);
    		sciErr = getVarType(pvApiCtx, piAddr2, &iType);
    		if(iType==sci_list)//Checking the type of 2nd argument
    		{
       			retrieveImage(B,2);
       			Size s1=A.size();
       			Size s2=B.size();
       
       			multiply(A,B,C);//Calling the opencv function
    		}
    		else
    		{
      			if(isDoubleType(pvApiCtx, piAddr2))
       			{
	 			if(isScalar(pvApiCtx, piAddr2))
	   			{
	      				iRet = getScalarDouble(pvApiCtx, piAddr2, &value);
           			}
        		} 
      		C=A*value;
    		}  
    		//Assigning the output to be returned to scilab console
    		int temp = nbInputArgument(pvApiCtx) + 1;
    		string tempstring = type2str(C.type());
    		char *checker;
    		checker = (char *)malloc(tempstring.size() + 1);
    		memcpy(checker, tempstring.c_str(), tempstring.size() + 1);
    		returnImage(checker,C,1);
    		free(checker); 
    		AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
    		ReturnArguments(pvApiCtx);
    		return 0;
  }
}
