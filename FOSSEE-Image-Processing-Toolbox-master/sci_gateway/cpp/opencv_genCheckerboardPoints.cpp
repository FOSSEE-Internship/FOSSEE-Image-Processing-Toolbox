/***************************************************
Author : Tanmay Chaudhari
 **************************************************/

#include <numeric>
#include <iostream>
using namespace std;
extern "C"
{
#include "api_scilab.h"
#include "Scierror.h"
#include "BOOL.h"
#include <localization.h>
#include "sciprint.h"

	int opencv_genCheckerboardPoints(char *fname, unsigned long fname_len)
	{
		//Error management variable
		SciErr sciErr;

		//Variable declaration
		int iType = 0;
		int iComplex = 0;
		int rowsOfBoardSize = 0;
		int rowsOfSquareSize = 0;
		int colsOfBoardSize = 0;
		int colsOfSquareSize = 0;
		int iterator1 = 0;
		int iterator2 = 0;
		int *piAddr = NULL;
		double *boardSize = NULL;
		double *squareSize = NULL;
		double *worldPoints = NULL;

		//Check input output arguments
		checkInputArgument(pvApiCtx, 2, 2);
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

		sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &rowsOfBoardSize, &colsOfBoardSize, &boardSize);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}

		if(((rowsOfBoardSize != 1) || (colsOfBoardSize != 2)) || (boardSize[0] <= 0) || (boardSize[1] <= 0))
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
		
		sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &rowsOfSquareSize, &colsOfSquareSize, &squareSize);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
		
		if(((rowsOfSquareSize != 1) || (colsOfSquareSize != 1)) || (squareSize[0] <= 0))
		{
			printError(&sciErr, 0);
			return 0;
		}

		boardSize[0] -= 1;
		boardSize[1] -= 1;
		worldPoints = (double*)malloc(sizeof(double) * 2 * (int(boardSize[0])) * (int(boardSize[1])));

		iterator1 = 0;
		iterator2 = (int(boardSize[0])) * (int(boardSize[1]));

		for(int j = 0; j <= int(boardSize[1]) - 1; j++)
		{
			for(int i = 0; i <= int(boardSize[0]) - 1; i++)
			{
				worldPoints[iterator1] = j * squareSize[0];
				worldPoints[iterator2] = i * squareSize[0];
				iterator1++;
				iterator2++;
			}
		}

		sciErr = createList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, 1, &piAddr);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}

		sciErr = createMatrixOfDoubleInList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, piAddr, 1, (int(boardSize[0])) * (int(boardSize[1])), 2, worldPoints);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}

		//Return Output Argument
		AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
		ReturnArguments(pvApiCtx);
		return 0;
	}
}



