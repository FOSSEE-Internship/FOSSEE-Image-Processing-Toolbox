/***************************************************
Author : Tanmay Chaudhari	
***************************************************/

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
	#include "sciprint.h"

	int opencv_bboxOverlapRatio(char *fname, unsigned long fname_len)
	{	
		//Error management variable
		SciErr sciErr;

		//Variable declaration
		int i, j;
		int nbInputArguments = 0;
		int iComplex = 0;
		int iType = 0;
		int iRows = 0;
		int iCols = 0;
		int rowsOfBboxA = 0;
		int colsOfBboxA = 0;
		int rowsOfBboxB = 0;
		int colsOfBboxB = 0;
		int unionOrMin = 0;	//if 0 then union else min
		int *piAddr = NULL;
		int *piLen = NULL;
		char **ratioType = NULL;
		double x1 = 0;
		double x2 = 0;
		double y1 = 0;
		double y2 = 0;
		double w = 0;
		double h = 0;
		double intersectAB = 0;
		double *overlapRatio = 0;
		double *bboxA = NULL;
		double *bboxB = NULL;
		double *x1BboxA = NULL;
		double *x2BboxA = NULL;
		double *y1BboxA = NULL;
		double *y2BboxA = NULL;
		double *x1BboxB = NULL;
		double *x2BboxB = NULL;
		double *y1BboxB = NULL;
		double *y2BboxB = NULL;
		double *areaA = NULL;
		double *areaB = NULL;

		//Checking number of input and output arguments (enviromnet variable, min arguments, max arguments)
		checkInputArgument(pvApiCtx, 2, 3);
		checkOutputArgument(pvApiCtx, 1, 1);

		//Get number of input arguments
		nbInputArguments = *getNbInputArgument(pvApiCtx);

		for( i = 1; i <= nbInputArguments; i++)
		{
			//Get variable address of the input arguent
			sciErr = getVarAddressFromPosition(pvApiCtx, i, &piAddr);
			if(sciErr.iErr)
			{	
				printError(&sciErr, 0);
				return 0;
			}

			if(i != 3)
			{
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

				if(i == 1)
					sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &rowsOfBboxA, &colsOfBboxA, &bboxA);
				else
					sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &rowsOfBboxB, &colsOfBboxB, &bboxB);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}

				if((i == 1) && ((rowsOfBboxA == 0) || (colsOfBboxA !=4)))
				{
					printError(&sciErr, 0);
					return 0;
				}	

				if((i == 2) && ((rowsOfBboxB == 0) || (colsOfBboxB != 4)))
				{
					printError(&sciErr, 0);
					return 0;
				}	
			}
			else
			{
				//first call to retrieve dimensions
				sciErr = getMatrixOfString(pvApiCtx, piAddr, &iRows, &iCols, NULL, NULL);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}

				piLen = (int*)malloc(sizeof(int) * iRows * iCols);

				//second call to retrieve the length of the string
				sciErr = getMatrixOfString(pvApiCtx, piAddr, &iRows, &iCols, piLen, NULL);
				if(sciErr.iErr || iRows != 1 || iCols != 1)
				{
					printError(&sciErr, 0);
					return 0;
				}

				ratioType = (char**)malloc(sizeof(char*) * iRows * iCols);
				for(int i=0;i< iRows * iCols; i++)
				{
					ratioType[i] = (char*)malloc(sizeof(char) * (piLen[i] + 1));	//+1 for NULL character
				}

				//third call to retrieve data
				sciErr = getMatrixOfString(pvApiCtx, piAddr, &iRows, &iCols, piLen, ratioType);
				if(sciErr.iErr)
				{
					printError(&sciErr, 0);
					return 0;
				}
				if(piLen[0] != 5 && piLen[0] != 3)
				{
					printError(&sciErr, 0);
					return 0;
				}
				if(piLen[0] == 5 && ratioType[0][0] == 'U' && ratioType[0][1] == 'n' && ratioType[0][2] == 'i' && ratioType[0][3] == 'o' && ratioType[0][4] == 'n')
					unionOrMin = 0;
				else if(piLen[0] == 3 && ratioType[0][0] == 'M' && ratioType[0][1] == 'i' && ratioType[0][2] == 'n')
					unionOrMin = 1;
				else
				{
					Scierror(999, "%s: Wrong type for input argument: Union or Min is expected.\n");
					return 0;
				}
				free(piLen);
				free(ratioType);
			}
		}

		//Application Code
        try
        {
			x1BboxA = (double*)malloc(sizeof(double) * rowsOfBboxA);
			x2BboxA = (double*)malloc(sizeof(double) * rowsOfBboxA);
			y1BboxA = (double*)malloc(sizeof(double) * rowsOfBboxA);
			y2BboxA = (double*)malloc(sizeof(double) * rowsOfBboxA);

			x1BboxB = (double*)malloc(sizeof(double) * rowsOfBboxB);
			x2BboxB = (double*)malloc(sizeof(double) * rowsOfBboxB);
			y1BboxB = (double*)malloc(sizeof(double) * rowsOfBboxB);
			y2BboxB = (double*)malloc(sizeof(double) * rowsOfBboxB);

			areaA = (double*)malloc(sizeof(double) * rowsOfBboxA);
			areaB = (double*)malloc(sizeof(double) * rowsOfBboxB);

			overlapRatio = (double*)malloc(sizeof(double) * rowsOfBboxA * rowsOfBboxB);

			for( i = 0; i < rowsOfBboxA; i++)
			{
				//Left top corner
				x1BboxA[i] = bboxA[i];
				y1BboxA[i] = bboxA[rowsOfBboxA + i];

				//right bottom corner
				x2BboxA[i] = x1BboxA[i] + bboxA[2 * rowsOfBboxA + i];
				y2BboxA[i] = y1BboxA[i] + bboxA[3 * rowsOfBboxA + i];
			}

			for( i = 0; i < rowsOfBboxB; i++)
			{
				//Left top corner
				x1BboxB[i] = bboxB[i];
				y1BboxB[i] = bboxB[rowsOfBboxB + i];

				//right bottom corner
				x2BboxB[i] = x1BboxB[i] + bboxB[2 * rowsOfBboxB + i];
				y2BboxB[i] = y1BboxB[i] + bboxB[3 * rowsOfBboxB + i];
			}

			//Computing area for each rectangle in bboxA
			for( i = 0; i < rowsOfBboxA; i++)
				areaA[i] = bboxA[2 * rowsOfBboxA + i] * bboxA[3 * rowsOfBboxA + i];

			//Computing area for each rectangle in bboxB
			for( i = 0 ; i < rowsOfBboxB; i++)
				areaB[i] = bboxB[2 * rowsOfBboxB + i] * bboxB[3 * rowsOfBboxB + i];

			for( i = 0; i < rowsOfBboxA * rowsOfBboxB; i++)
				overlapRatio[i] = 0;

			for( i = 0; i < rowsOfBboxA; i++)
			{
				for( j = 0; j < rowsOfBboxB; j++)
				{
					//Computing the corners of the intersect
					x1 = (x1BboxA[i] > x1BboxB[j])?x1BboxA[i]:x1BboxB[j];
					y1 = (y1BboxA[i] > y1BboxB[j])?y1BboxA[i]:y1BboxB[j];
					x2 = (x2BboxA[i] < x2BboxB[j])?x2BboxA[i]:x2BboxB[j];
					y2 = (y2BboxA[i] < y2BboxB[j])?y2BboxA[i]:y2BboxB[j];

					w = x2 - x1;
					if(w <= 0)			//Skip if no intersection
						continue;

					h = y2 - y1;
					if(h <= 0)			//Skip if no intersection
						continue;

					intersectAB = w * h;

					if(nbInputArguments == 2)
						overlapRatio[j * rowsOfBboxA + i] = intersectAB/(areaA[i] + areaB[j] - intersectAB); 
					else
					{
						if(unionOrMin == 0)
							overlapRatio[j * rowsOfBboxA + i] = intersectAB/(areaA[i] + areaB[j] - intersectAB); 
						else 
							overlapRatio[j * rowsOfBboxA + i] = intersectAB/((areaA[i] < areaB[j])?areaA[i]:areaB[j]);
					}
				}
			}
        }
		catch(cv::Exception& e)
        {
            const char* err=e.what();
            Scierror(999,("%s"),err);
        }

		sciErr = createList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, 1, &piAddr);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
		sciErr = createMatrixOfDoubleInList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, piAddr, 1, rowsOfBboxA, rowsOfBboxB, overlapRatio);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
		
		//Assigning the list as the Output Variable
		AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;

		//Returning the Output Variables as arguments to the Scilab environment
		ReturnArguments(pvApiCtx);

		return 0;
	}
	/* ==================================================================== */
}
