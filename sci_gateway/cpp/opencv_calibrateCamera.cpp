/***************************************************
Author : NIhar Rao,Gursimar Singh
***************************************************/

#include <numeric>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/core/types_c.h"
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

 double * readdoublearray_fromscilab(int);//function for reading double matrix

int opencv_calibrateCamera(char *fname, unsigned long fname_len)
  {
    //variables
    
    int i,j,k,n,m;
    int iRows=0,iCols=0;
    int *piAddr=NULL;
    double *pdblReal = NULL;
    double x,y,rms;


    SciErr sciErr;
    
    vector<std::vector<Point3f> >objectPoints(1);

   	Mat cameraMatrix;
    Mat distCoeffsActual;
    vector <Mat> rotationMatrix;
  	vector <Mat> translationVector;
  	int num;

  ////////parsing input
  	CheckInputArgument(pvApiCtx, 3, 5);
    CheckOutputArgument(pvApiCtx, 1, 4) ;
    n=*getNbInputArgument(pvApiCtx);
   //first
    sciErr = getVarAddressFromPosition(pvApiCtx, 1, &piAddr);
    if (sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }
    sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &iRows, &iCols, &pdblReal);
    if(sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }
    //stores 
    sciprint("object points obtained\n");
    //objectPoints = (CvPoint3D64f*)cvAlloc( iRows *1* sizeof(CvPoint3D64f));
    for(i = 0; i < iRows; ++i)
            objectPoints[0].push_back(Point3f(float(pdblReal[(0 * iRows) + i]), float(pdblReal[(1 * iRows) + i]),0.0));


    int pointCount=iRows;
     
       sciprint("object points converted\n"); 
    sciErr = getVarAddressFromPosition(pvApiCtx, 2, &piAddr);
    if (sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }

    if(!isListType(pvApiCtx, piAddr))
        {
            
            Scierror(999,"\nthe imagepoints1 Argument must be a list of points \n");
            return 0;
        }
    sciErr = getListItemNumber(pvApiCtx, piAddr, &num);
    if(sciErr.iErr)
    {
        printError(&sciErr, 0);
        
        return 0;
    }

    //get items from list
    sciprint("list number obtained\n");

    vector<std::vector<Point2f> >imagepoints(num) ;                 ////////////////////declaring imagepoints///////////
    for(int i=1;i<=num;i++)
    {
    	
    	sciErr=getMatrixOfDoubleInList(pvApiCtx, piAddr,i,&iRows, &iCols, &pdblReal);
    	
    	if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        
    	for(j = 0; j < iRows; ++j)
            imagepoints[i-1].push_back( Point2f(float(pdblReal[(0 * iRows) + j]), float(pdblReal[(1 * iRows) + j])));
        


    }
    //imageWidth and Height
    pdblReal=readdoublearray_fromscilab(3);
    double image_width=pdblReal[0];
    double image_height=pdblReal[1];

    //3rd input
   
    ///Getting optional arguments
    int p;
    switch(n)
    {
    	case 4:
    	pdblReal=readdoublearray_fromscilab(4);
    	for(i=0;i<3;i++)
        for(j=0;j<3;j++)
            cameraMatrix.at<double>(i,j) = pdblReal[(j * 3) + i];
    	break;

    	case 5:
        //camera Matrix
    			pdblReal=readdoublearray_fromscilab(4);
    			for(i=0;i<3;i++)
        		for(j=0;j<3;j++)
            		cameraMatrix.at<double>(i,j) = pdblReal[(j * 3) + i];

    		sciErr = getVarAddressFromPosition(pvApiCtx,5,&piAddr);
    		if (sciErr.iErr)
    		{
        		printError(&sciErr, 0);
        		return 0;
    		}
    		sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &iRows, &iCols, &pdblReal);
    		if(sciErr.iErr)
    		{
        		printError(&sciErr, 0);
        		return 0;
    		}
    
			    if(iRows== 1)
			        p = iCols;
			    else if (iCols == 1)
			        p = iRows;
			    else
          {
			        Scierror(1,"Distortion Points matrix (arg 5) must be a 1 X N or N X 1 matrix");
			        return 0;
		       }
			    if(p==4 or p==5 or p==8);
			    else{
			        Scierror(1," N must be 4 or 5 or 8");
			        return 0;
			    }

      for(i=0;i<p;i++)
        distCoeffsActual.at<double>(0,i) = pdblReal[i];
			

    	break;
    }
//***********Actual processing***************//  
  	try
  	{
  	  Size imageSize (image_width,image_height);
  		rms = calibrateCamera(objectPoints,imagepoints,imageSize,
                  cameraMatrix, distCoeffsActual, rotationMatrix, translationVector,CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5,
                  TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 50, 0.1));
    }    
         
    catch(cv::Exception& e)
    {
      const char* err=e.what();
      Scierror(999,e.what());
          
    }

  	sciprint("Calibration done with RMS error=%f\n",rms);

    
  		 ///Return Arguments to Scilab
  	double *pstdata1 = NULL,*pstdata2 = NULL,*pstdata3 = NULL,*pstdata4 = NULL;
    pstdata1 = (double*)malloc(sizeof(double) *3* 3);
    
    for(i=0;i<3;i++)
        for(j=0;j<3;j++)
            pstdata1[(j * 3) + i]=cameraMatrix.at<double>(i,j); 

    sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 1, 3, 3, pstdata1);
    //camera matrix
    if(sciErr.iErr)
    {
      printError(&sciErr, 0);
      return 0;
    }
     pstdata2 = (double*)malloc(sizeof(double) *4*1);
    
    for(i=0;i<4;i++)
         pstdata2[i]=distCoeffsActual.at<double>(0,i);
    //distortion coefficients
    sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 2, 1, 4, pstdata2);
    if(sciErr.iErr){
      printError(&sciErr, 0);
      return 0;
    }

    //rotation matrix
    pstdata3=(double*)malloc(sizeof(double) *rotationMatrix[0].rows*rotationMatrix[0].cols);

    for(i=0;i<rotationMatrix[0].rows;i++)
    	for(j=0;j<rotationMatrix[0].cols;j++)
    	{

    		 pstdata3[(j * rotationMatrix[0].rows) + i]=rotationMatrix[0].at<double>(i,j); 

    	}

    sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 3, rotationMatrix[0].rows, rotationMatrix[0].cols, pstdata3);
    if(sciErr.iErr)
    {
      printError(&sciErr, 0);
      return 0;
    }

     pstdata4=(double*)malloc(sizeof(double) *translationVector[0].rows*translationVector[0].cols);

    for(i=0;i<translationVector[0].rows;i++)
    	for(j=0;j<translationVector[0].cols;j++)
    	{

    		 pstdata4[(j * translationVector[0].rows) + i]=translationVector[0].at<double>(i,j); 

    	}
    //translation vector
    sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 4, translationVector[0].rows, translationVector[0].cols, pstdata4);
    if(sciErr.iErr){
      printError(&sciErr, 0);
      return 0;
    }


    for(int i=1;i<=4;i++)
	  AssignOutputVariable(pvApiCtx, i) = nbInputArgument(pvApiCtx) + i;
    //Returning the Output Variables as arguments to the Scilab environment
    ReturnArguments(pvApiCtx);
    return 0;       



  	}
  	

double * readdoublearray_fromscilab(int cnt)
 {

 	SciErr sciErr;
    int *piAddr=NULL;
    double val=0;
    int  intErr;
    int iRows,iCols;
    double *pdblReal;

    sciErr = getVarAddressFromPosition(pvApiCtx,cnt,&piAddr);
    if (sciErr.iErr)
    {
    printError(&sciErr, 0);
    return 0;
    }

    sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &iRows, &iCols, &pdblReal);
    if(sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }
    if(isDoubleType(pvApiCtx, piAddr))
        return pdblReal;
    else
    {
      sciprint("Error: the %d argument is not of the type double!",cnt);
      return 0;
    }

 }









}







