/***************************************************
Author : Tanmay Chaudhari
***************************************************/

#include <numeric>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include <bits/stdc++.h>
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
 //   #include "../common.cpp"

int opencv_bbox2points(char *fname, unsigned long fname_len)
{
	//Error management variable
	SciErr sciErr;
	
	//Variable declaration
	int i;
	int iComplex = 0;
	int iType = 0;
	int iRows = 0;
	int iCols = 0;
	int *piAddr = NULL;
	double *pdblReal = NULL;
	double *points;

	//Check input output arguments
	checkInputArgument(pvApiCtx, 1, 1);
	checkOutputArgument(pvApiCtx, 1, 1);

	//Get variable address of the first input arguent
	sciErr = getVarAddressFromPosition(pvApiCtx, 1, &piAddr);
	if(sciErr.iErr)
	{		
		printError(&sciErr, 0);
		return 0;
	}

	//Check type
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
	
	sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &iRows, &iCols, &pdblReal);
	if(sciErr.iErr || iCols != 4 || iRows == 0)
	{
		printError(&sciErr, 0);
		return 0;
	}
	
	sciErr = createList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, iRows, &piAddr);
	if(sciErr.iErr)
	{
		printError(&sciErr, 0);
		return 0;
	}
	//sciprint("%d",iRows);
	points = (double*)malloc(sizeof(double) * 8);
	for( i = 0; i < iRows; i++)					///Rows in box
	{

		
		memset(points, 0, sizeof(points));

		//Upper-left
		points[0] = pdblReal[i];
		points[4] = pdblReal[i + iRows];	

		//Upper-right
		points[1] = pdblReal[i] + pdblReal[i + 2 * iRows];
		points[5] = pdblReal[i + iRows];

		//Lower-right
		points[2] = pdblReal[i] + pdblReal[i + 2 * iRows];
		points[6] = pdblReal[i + iRows] + pdblReal[i + 3 * iRows];

		//Lower-left
		points[3] = pdblReal[i];
		points[7] = pdblReal[i + iRows] + pdblReal[i + 3 * iRows];
		//cout<<points[7]<<endl;
		sciErr = createMatrixOfDoubleInList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, piAddr, i + 1, 4, 2, points);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
	}

	//Return output arguments
	AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
	ReturnArguments(pvApiCtx);
	
	return 0;
}
/* ==================================================================== */
}
