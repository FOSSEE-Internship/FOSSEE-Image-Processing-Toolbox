/***************************************************
Author : Tanmay Chaudhari
 **************************************************/
#include <math.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <bits/stdc++.h>
using namespace std;
extern "C"
{
	#include "api_scilab.h"
	#include "Scierror.h"
	#include "BOOL.h"
	#include <localization.h>
	#include "sciprint.h"

	int opencv_rotationVectorToMatrix(char *fname, unsigned long fname_len)
	{

		//Error management variable
		SciErr sciErr;

		//Variable declaration
		int iComplex = 0;
		int iType = 0;
		int iRows = 0;
		int iCols = 0;
		int *piAddr = NULL;
		double sinOfangle = 0;
		double cosOfangle = 0;
		double t=0, x=0, y=0, z=0;
		double theta=0;
		double *pdbl = NULL;
		double *matrix=NULL;
		double *identity=NULL;
		double *A=NULL;
		double *B=NULL;

		//Check input output arguments
		checkInputArgument(pvApiCtx, 1, 1);
		checkOutputArgument(pvApiCtx, 1, 1);

		//Get variable address of the first input argument
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

		sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &iRows, &iCols, &pdbl);
		if(sciErr.iErr || iCols != 3 || iRows != 1)
		{
			printError(&sciErr, 0);
			return 0;
		}

		//Application Code
 		try
 		{
 			x = pdbl[0];
			y = pdbl[1];
			z = pdbl[2];
			theta = sqrt(x * x + y * y + z * z);
			sinOfangle = sin(theta);
			cosOfangle = cos(theta);
			t = 1 - cosOfangle;
			x = pdbl[0] / theta;
			y = pdbl[1] / theta;
			z = pdbl[2] / theta;

			matrix = (double*)malloc(sizeof(double) * 9);
			A = (double*)malloc(sizeof(double) * 9);
			B = (double*)malloc(sizeof(double) * 9);
			identity = (double*)malloc(sizeof(double) * 9);
			
			for(int i = 0; i < 3; i++)
			{
				for(int j = 0; j < 3; j++)
				{
					if(i == 0)
					{
						if(j == 0)
							A[j * 3 + i] = x * x;
						else if(j == 1)
							A[j * 3 + i] = x * y;
						else if(j == 2)
							A[j * 3 + i] = x * z;

					}
					else if(i == 1)
					{
						if(j == 0)
							A[j * 3 + i] = x * y;
						else if(j == 1)
							A[j * 3 + i] = y * y;
						else if(j == 2)
							A[j * 3 + i] = y * z;
					}
					else
					{
						if(j == 0)
							A[j * 3 + i] = x * z;
						else if(j == 1)
							A[j * 3 + i] = y * z;
						else if(j == 2)
							A[j * 3 + i] = z * z;
					}

				}
			}
			
			for(int i = 0; i < 3; i++)
			{
				for(int j = 0; j < 3; j++)
				{
					if(i == 0)
					{
						if(j == 0)
							B[j * 3 + i] = 0;
						else if(j == 1)
							B[j * 3 + i] = -z;
						else if(j == 2)
							B[j * 3 + i] = y;

					}
					else if(i == 1)
					{
						if(j == 0)
							B[j * 3 + i] = z;
						else if(j == 1)
							B[j * 3 + i] = 0;
						else if(j == 2)
							B[j * 3 + i] = -x;
					}
					else
					{
						if(j == 0)
							B[j * 3 + i] = -y;
						else if(j == 1)
							B[j * 3 + i] = x;
						else if(j == 2)
							B[j * 3 + i] = 0;
					}

				}
			}
			for(int i = 0; i < 3; i++)
			{
				for(int j = 0;j < 3; j++)
				{
					if(i == j)
						identity[j * 3 + i] = 1;
					else
						identity[j * 3 + i] = 0;
				}
			}
			for(int i = 0; i < 3; i++)
			{
				for(int j = 0; j < 3; j++)
				{
					matrix[i * 3 + j] = identity[j * 3 + i] * cosOfangle + t * A[j * 3 + i] + sinOfangle * B[j * 3 + i];
				}
			}
			sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 1,3, 3, matrix);
			if(sciErr.iErr)
			{
				printError(&sciErr, 0);
				return 0;
			}
 		}
 		catch(cv::Exception& e)
		{
		 	const char* err=e.what();
		 	Scierror(999,("%s"),err);
	 	}

		//Return output arguments
		AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;

		//Returning the Output Variables as arguments to the Scilab environment
		ReturnArguments(pvApiCtx);
		return 0;
	}
}



