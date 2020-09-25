/********************************************************
    Author: Abhilasha Sancheti, Sukul Bagai, Shubham Lohakare
*********************************************************
   contours = findcontours(input_image(after canny is applied on it), mode, method, point_x, point_y)
********************************************************/

#include <numeric>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
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
  
  	int opencv_findContours(char *fname, unsigned long fname_len)
  	{
		//Declaring variables
    		SciErr sciErr;
    		int intErr=0;
    		int iRows=0,iCols=0;
    		int *piLen2=NULL;
    		int *piLen3 = NULL;
    		int *piAddrNew = NULL;
    		int *piAddr2  = NULL;
    		int *piAddr3  = NULL;
    		int *piAddr4  = NULL;
    		int *piAddr5  = NULL;
    		int i,j,k, findmode , findmethod;
    		double find_method , mode;
    		double x,y;
   

    		CheckInputArgument(pvApiCtx, 5, 5);//checking input argument
    		CheckOutputArgument(pvApiCtx, 1, 1) ;//checking output argument

    
    		Mat src;
    		retrieveImage(src,1);//Retrieving the input image
    		src.convertTo(src,CV_8U);//Converting the input image to 8-bit image
		sciErr = getVarAddressFromPosition(pvApiCtx,2,&piAddr2);
    		if (sciErr.iErr)
    		{
       	 		printError(&sciErr, 0);
        		return 0;
    		}
    		intErr = getScalarDouble(pvApiCtx, piAddr2,&mode);//Getting the second input argument
    		if(intErr)
       			return intErr;

		sciErr = getVarAddressFromPosition(pvApiCtx,3,&piAddr3);
    		if (sciErr.iErr)
    		{
        		printError(&sciErr, 0);
        		return 0;
    		}
    		intErr = getScalarDouble(pvApiCtx, piAddr3,&find_method);//Getting the third input argument
    		if(intErr)
       			return intErr;
       		//toget the x coordinate of point parameter of findcontours
        	sciErr = getVarAddressFromPosition(pvApiCtx,4,&piAddr4);
    		if (sciErr.iErr)
    		{
        		printError(&sciErr, 0);
        		return 0;
    		}
    		intErr = getScalarDouble(pvApiCtx, piAddr4,&x);//Getting the fourth input argument
    		if(intErr)
       		return intErr;
  
           	//toget the y coordinate of point parameter of findcontours
        	sciErr = getVarAddressFromPosition(pvApiCtx,5,&piAddr5);
    		if (sciErr.iErr)
    		{
        		printError(&sciErr, 0);
        		return 0;
    		}
    		intErr = getScalarDouble(pvApiCtx, piAddr5 ,&y);//Getting the fifth input argument
    		if(intErr)
       		return intErr;
     
      	 	//to set hte mode for findcontour
       		if(mode==1)
            		findmode = CV_RETR_EXTERNAL;
        	else if (mode==2)
          		findmode =  CV_RETR_LIST;
        	else if (mode==3)
          		findmode =  CV_RETR_CCOMP;
        	else if (mode==4)
            		findmode =  CV_RETR_TREE;
        	else
           	{
              		findmode =  CV_RETR_LIST;
              		sciprint("wrong mode given , using CV_RETR_TREE instead");
           	}
        	// to set the method for findContours
       		if (find_method==1)
            		findmethod = CV_CHAIN_APPROX_NONE;
        	else if (find_method==2)
          		findmethod =  CV_CHAIN_APPROX_SIMPLE;
        	else if (find_method==3)
          		findmethod =  CV_CHAIN_APPROX_TC89_L1;
        	else if (find_method==4)
            		findmethod =  CV_CHAIN_APPROX_TC89_KCOS;       
        	else
           	{
              		findmethod =  CV_CHAIN_APPROX_NONE;
              		sciprint("wrong method given , using CV_CHAIN_APPROX_SIMPLE instead");
           	}
    		Point pt(x,y);
    		/// Find contours  
    		vector<vector<Point> > contours;
    		vector<Vec4i> hierarchy;
		try{
    			findContours( src, contours, findmode, findmethod);
		}
		catch(Exception& e)
		{
			const char* err=e.what();
			printf("%s",err);
		}		
    		//Assigning the list as the Output Variable
    		sciErr = createList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, contours.size(), &piAddrNew);
    		if(sciErr.iErr)
    		{
       			printError(&sciErr, 0);           
       			return 0;
    		}

    		for(i=0;i<contours.size();i++)
    		{
        		double *arr = (double *)malloc(sizeof(double) * 2 * contours[i].size());

        		for(j=0;j<contours[i].size();j++)
        		{
            			*(arr + j + 0) = contours[i][j].x;
            			*(arr + j + contours[i].size()) = contours[i][j].y;
        		}

        	sciErr = createMatrixOfDoubleInList(pvApiCtx, nbInputArgument(pvApiCtx)+1 , piAddrNew, i+1, contours[i].size(), 2, arr);
        	free(arr);
        	if(sciErr.iErr)
        	{
            		printError(&sciErr, 0);
            		return 0;
        	}
    		}

    		AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
    		//Returning the Output Variables as arguments to the Scilab environment
    		ReturnArguments(pvApiCtx); 
    		return 0;


 	}

}
