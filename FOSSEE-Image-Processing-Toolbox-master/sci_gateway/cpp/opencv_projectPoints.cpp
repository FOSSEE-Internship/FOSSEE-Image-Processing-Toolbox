#include <numeric>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
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
  
  int opencv_projectPoints(char *fname, unsigned long fname_len)
  {
  
   SciErr sciErr;
   int intErr=0;
   int iComplex = 0;
   int iType = 0;
   int *piAddr  = NULL;
   int i,size,j;
   double *worldPoints=NULL,*rVector=NULL,*tVector=NULL,*intrinsicMatrix=NULL,*distCoefficients=NULL;
   int wpRows,wpCols,rvRows,rvCols,tvRows,tvCols,imRows,imCols,dcRows,dcCols;
   double *points=NULL;
  
    vector<cv::Point3d> objectPoints;
    vector<cv::Point2d> imagePoints;
    Mat intrisicMat(3, 3, cv::DataType<double>::type);
    Mat rVec(3, 1, cv::DataType<double>::type);
    Mat tVec(3, 1, cv::DataType<double>::type);
    Mat distCoeffs(4, 1, cv::DataType<double>::type);
  
  	CheckInputArgument(pvApiCtx, 5, 5);
        CheckOutputArgument(pvApiCtx, 1, 1);
  
  	sciErr = getVarAddressFromPosition(pvApiCtx, 1, &piAddr);
	if(sciErr.iErr)
	{
		printError(&sciErr, 0);
		return 0;
	}
	sciErr = getVarType(pvApiCtx, piAddr, &iType);
	if(sciErr.iErr || iType != sci_matrix)
	{
		printError(&sciErr, 0);
		return 0;
	}
	//Get complexity
	iComplex = isVarComplex(pvApiCtx, piAddr);
	//Check complexity
	if(iComplex)
	{
		Scierror(999, "%s: Wrong type for input argument: A complex number is not expected.\n");
		return 0;
	}	
 	sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &wpRows, &wpCols, &worldPoints);
 	if(sciErr.iErr)
	{
		printError(&sciErr, 0);
		return 0;
	}
  	if(wpCols!=1)
  	{
		Scierror(999, "%s: Wrong value for input argument: World points must be M*1\n");
		return 0;
  	}
  	
  	sciErr = getVarAddressFromPosition(pvApiCtx, 2, &piAddr);
	if(sciErr.iErr)
	{
		printError(&sciErr, 0);
		return 0;
	}
	sciErr = getVarType(pvApiCtx, piAddr, &iType);
	if(sciErr.iErr || iType != sci_matrix)
	{
		printError(&sciErr, 0);
		return 0;
	}
	//Get complexity
	iComplex = isVarComplex(pvApiCtx, piAddr);
	//Check complexity
	if(iComplex)
	{
		Scierror(999, "%s: Wrong type for input argument: A complex number is not expected.\n");
		return 0;
	}	
 	sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &rvRows, &rvCols, &rVector);
 	if(sciErr.iErr)
	{
		printError(&sciErr, 0);
		return 0;
	}
	if(rvCols!=3)
	{
		
	}
	sciErr = getVarAddressFromPosition(pvApiCtx, 3, &piAddr);
	if(sciErr.iErr)
	{
		printError(&sciErr, 0);
		return 0;
	}
	sciErr = getVarType(pvApiCtx, piAddr, &iType);
	if(sciErr.iErr || iType != sci_matrix)
	{
		printError(&sciErr, 0);
		return 0;
	}
	//Get complexity
	iComplex = isVarComplex(pvApiCtx, piAddr);
	//Check complexity
	if(iComplex)
	{
		Scierror(999, "%s: Wrong type for input argument: A complex number is not expected.\n");
		return 0;
	}	
 	sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &tvRows, &tvCols, &tVector);
 	if(sciErr.iErr)
	{
		printError(&sciErr, 0);
		return 0;
	}
	if(tvCols!=3)
	{
		
	}
	if(rvRows!=tvRows)
	{
		
	}
	
	sciErr = getVarAddressFromPosition(pvApiCtx, 4, &piAddr);
	if(sciErr.iErr)
	{
		printError(&sciErr, 0);
		return 0;
	}
	sciErr = getVarType(pvApiCtx, piAddr, &iType);
	if(sciErr.iErr || iType != sci_matrix)
	{
		printError(&sciErr, 0);
		return 0;
	}
	//Get complexity
	iComplex = isVarComplex(pvApiCtx, piAddr);
	//Check complexity
	if(iComplex)
	{
		Scierror(999, "%s: Wrong type for input argument: A complex number is not expected.\n");
		return 0;
	}	
 	sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &imRows, &imCols, &intrinsicMatrix);
 	if(sciErr.iErr)
	{
		printError(&sciErr, 0);
		return 0;
	}
	if(imRows!=3 || imCols!=3)
	{
		
	}
	
	
	sciErr = getVarAddressFromPosition(pvApiCtx, 5, &piAddr);
	if(sciErr.iErr)
	{
		printError(&sciErr, 0);
		return 0;
	}
	sciErr = getVarType(pvApiCtx, piAddr, &iType);
	if(sciErr.iErr || iType != sci_matrix)
	{
		printError(&sciErr, 0);
		return 0;
	}
	//Get complexity
	iComplex = isVarComplex(pvApiCtx, piAddr);
	//Check complexity
	if(iComplex)
	{
		Scierror(999, "%s: Wrong type for input argument: A complex number is not expected.\n");
		return 0;
	}	
 	sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &dcRows, &dcCols, &distCoefficients);
 	if(sciErr.iErr)
	{
		printError(&sciErr, 0);
		return 0;
	}
	
	//if(dcRows!=1 || dcCols!=3)
	//{
		
	//}
	
	for(i=0;i<wpRows;++i)
	{
		objectPoints.push_back(cv::Point3d(worldPoints[i],worldPoints[i+wpRows],0));
	}
	
    intrisicMat.at<double>(0, 0) = intrinsicMatrix[0];
    intrisicMat.at<double>(1, 0) = 0;
    intrisicMat.at<double>(2, 0) = 0;
    
    intrisicMat.at<double>(0, 1) = 0;
    intrisicMat.at<double>(1, 1) = intrinsicMatrix[4];
    intrisicMat.at<double>(2, 1) = 0;

    intrisicMat.at<double>(0, 2) = intrinsicMatrix[2];
    intrisicMat.at<double>(1, 2) = intrinsicMatrix[5];
    intrisicMat.at<double>(2, 2) = 1;

    rVec.at<double>(0) = rVector[0];
    rVec.at<double>(1) = rVector[1];
    rVec.at<double>(2) = rVector[2];

    tVec.at<double>(0) = tVector[0];
    tVec.at<double>(1) = tVector[1];
    tVec.at<double>(2) = tVector[2];
    
    distCoeffs.at<double>(0) = distCoefficients[0];
    distCoeffs.at<double>(1) = distCoefficients[1];
    distCoeffs.at<double>(2) = distCoefficients[2];
    distCoeffs.at<double>(3) = distCoefficients[3];
    
    if(dcCols==5)
    {
    	distCoeffs.at<double>(4) = distCoefficients[4];
    }
     
    
    //Actual process
   cv::fisheye::projectPoints(objectPoints,imagePoints, rVec, tVec, intrisicMat, distCoeffs, 0 );
    size=imagePoints.size();
    points = (double*) malloc(sizeof(double) * size * 2);
    for(i = 0 ; i < size ; i++)
	{
		points[i] = imagePoints[i].x;
		points[i+size] = imagePoints[i].y;
	}
	
	sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 1, size , 2, points);
	if(sciErr.iErr)
	{
		printError(&sciErr, 0);
		return 0;
	}
	AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
    ReturnArguments(pvApiCtx);
    return 0;
  }  
}
