/***************************************************
Author : Tanmay Chaudhari
 **************************************************/
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

	int opencv_cameraMatrix(char *fname, unsigned long fname_len)
	{
		//Error management variable
		SciErr sciErr;

		//Variable declaration
		int iComplex = 0;
		int iType = 0;
		int iRows = 0;
		int iCols = 0;
		int *piAddr = NULL;
		double *instrinsicMatrix = NULL;
		double *rotationMatrix = NULL;
		double *translationVector = NULL;
		double *camMatrix = NULL;
		double *tempMatrix = NULL;

		//Check input output arguments
		checkInputArgument(pvApiCtx, 3, 3);
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

		sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &iRows, &iCols, &instrinsicMatrix);
		if(sciErr.iErr || iCols != 3 || iRows != 3)
		{
			printError(&sciErr, 0);
			return 0;
		}

		//Get variable address of the second input argument
		sciErr = getVarAddressFromPosition(pvApiCtx, 2, &piAddr);
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

		sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &iRows, &iCols, &rotationMatrix);
		if(sciErr.iErr || iCols != 3 || iRows != 3)
		{
			printError(&sciErr, 0);
			return 0;
		}

		//Get variable address of the first third argument
		sciErr = getVarAddressFromPosition(pvApiCtx, 3, &piAddr);
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

		sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &iRows, &iCols, &translationVector);
		if(sciErr.iErr || iCols != 3 || iRows != 1)
		{
			printError(&sciErr, 0);
			return 0;
		}

		//Application Code
		try
		{
			tempMatrix = (double*)malloc(sizeof(double) * 12);
			for(int i = 0; i < 4; i++)
			{
				for(int j = 0; j < 3; j++)
				{
					if(i == 3)
						tempMatrix[j * 4 + i] = translationVector[j];
					else
						tempMatrix[j * 4 + i] = rotationMatrix[j * 3 + i];
				}
			}
			camMatrix = (double*)malloc(sizeof(double) * 12);
			for(int i = 0; i < 12; i++)
				camMatrix[i] = 0;

			for(int i = 0; i < 4; i++)
			{
				for(int j = 0; j < 3; j++)
				{
					for(int k = 0; k < 3; k++)
					{
						camMatrix[j * 4 + i] += (tempMatrix[k * 4 + i] * instrinsicMatrix[j * 3 + k]);
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
		sciErr = createMatrixOfDoubleInList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, piAddr, 1, 4, 3, camMatrix);
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
}
