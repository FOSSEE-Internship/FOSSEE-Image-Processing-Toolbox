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
	//    #include "../common.h"
	//    #include "../common.cpp"

	int opencv_integralFilter(char *fname, unsigned long fname_len)
	{
		//Error management variable
		SciErr sciErr;
		
		//Variable declaration
		int i, j, k;
		int iType = 0;
		int iComplex = 0;
		int rowsOfintImage = 0;
		int colsOfintImage = 0;
		int rowsOfBbox = 0;
		int rowsOfWeights = 0;
		int colsOfBbox = 0;
		int colsOfWeights = 0;
		int rowsOfFilter = 0;
		int colsOfFilter = 0;
		int sR = 0;
		int sC = 0;
		int eR = 0;
		int eC = 0;
		int *piAddr = NULL;
		int *outSize = NULL;
		double bboxSum = 0;
		double *integralFilter = NULL;
		double *intImage = NULL;
		double *bbox = NULL;
		double *weights = NULL;		
		double *filterSize = NULL;

		//Check input output argument
		checkInputArgument(pvApiCtx, 4, 4);
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

		sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &rowsOfintImage, &colsOfintImage, &intImage);
		if(sciErr.iErr || rowsOfintImage == 0 || colsOfintImage == 0)
		{
			printError(&sciErr, 0);
			return 0;
		}

		//Get variable address of the second input arguent
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

		sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &rowsOfBbox, &colsOfBbox, &bbox);
		if(sciErr.iErr || colsOfBbox != 4 || rowsOfBbox == 0)
		{
			printError(&sciErr, 0);
			return 0;
		}

		//Get variable address of the third input arguent
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

		sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &rowsOfWeights, &colsOfWeights, &weights);
		if(sciErr.iErr || colsOfWeights == 0 || rowsOfWeights != 1 || colsOfWeights != rowsOfBbox)
		{
			printError(&sciErr, 0);
			return 0;
		}

		//Get variable address of the fourth input arguent
		sciErr = getVarAddressFromPosition(pvApiCtx, 4, &piAddr);
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

		sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &rowsOfFilter, &colsOfFilter, &filterSize);
		if(sciErr.iErr || colsOfFilter != 2 || rowsOfFilter != 1)
		{
			printError(&sciErr, 0);
			return 0;
		}
	
		outSize = (int*)malloc(sizeof(int) * 2);
		if(filterSize[0] != 0 && filterSize[1] != 0)
		{
			outSize[0] = rowsOfintImage - filterSize[0];	
			outSize[1] = colsOfintImage - filterSize[1];
		}
		else
		{
			outSize[0] = rowsOfintImage - 1;
			outSize[1] = colsOfintImage - 1;
		}
		
		if(outSize[0] <= 0 || outSize[1] <= 0)
			outSize[0] = outSize[1] = 0;
			
		integralFilter = (double*)malloc(sizeof(double) * outSize[0] * outSize[1]);
		
		for( i = 0; i < outSize[0] * outSize[1]; i++)
			integralFilter[i] = 0;
			
		for( i = 1; i <= outSize[1]; i++)
		{
			for( j = 1; j <= outSize[0]; j++)
			{
				for( k = 0; k < rowsOfBbox; k++)
				{
					sR = j + bbox[1 * rowsOfBbox + k] - 1;
					sC = i + bbox[0 * rowsOfBbox + k] - 1;
					eR = sR + bbox[3 * rowsOfBbox + k];
					eC = sC + bbox[2 * rowsOfBbox + k];
					
					bboxSum = intImage[(eC - 1) * rowsOfintImage + (eR - 1)] - intImage[(sC - 1) * rowsOfintImage + (eR - 1)];
					bboxSum -= intImage[(eC - 1) * rowsOfintImage + (sR - 1)];
					bboxSum += intImage[(sC - 1) * rowsOfintImage + (sR - 1)];

					integralFilter[(j - 1) * outSize[1] + (i - 1)] = integralFilter[(j - 1) * outSize[1] + (i - 1)] + weights[k] * bboxSum; 
				}
			}
		}

		//Creating list to store integral filter in it
		sciErr = createList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, 1, &piAddr);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}

		sciErr = createMatrixOfDoubleInList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, piAddr, 1, outSize[0], outSize[1], integralFilter);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}

		//Return output arguments
		AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
		ReturnArguments(pvApiCtx);

		return 0;
	}
	/* ==================================================================== */
}
