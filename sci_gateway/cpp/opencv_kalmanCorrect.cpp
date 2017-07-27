#include <numeric>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"	
#include <opencv2/video/background_segm.hpp>
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
    #include "../common.h"

	int opencv_kalmanCorrect(char *fname, unsigned long fname_len)
	{
		SciErr sciErr;
        int intErr = 0;
 		int *piAddr = NULL;
 		int num_InputArgs;  //-> gives total number of arguments
        int iRows, iCols;
        double nframe=0;
        double area=0;
        int iItem = 0;
        int iRet;
        int *piLen = NULL;
        char **pstData = NULL;  
        char *currentArg = NULL; //-> Stores current string representing 'name' of name,value pair arguments
        bool *providedArgs = NULL; //-> Used to check that optional argument is not entered more than once
        //-> Name,Value Pair Variables
        int dynamParams,measureParams;
        int controlParams,type;
        double *temp=NULL;
        double *trMatrix=NULL;
        double *measureMatrix=NULL;
        double *controlMatrix=NULL;
        double *stateMatrix=NULL;
        double *stateCovMatrix=NULL;
        double *pNoiseMatrix=NULL;
        double *mNoiseMatrix=NULL;
        Mat _ntrMatrix;
        Mat _nmeasureMatrix;
        Mat _ncontrolMatrix;
        Mat _nstateMatrix;
        Mat _nstateCovMatrix;
        Mat _npNoiseMatrix;
        Mat _nmNoiseMatrix;

        CheckInputArgument(pvApiCtx, 2, 2);                     
        CheckOutputArgument(pvApiCtx, 2, 2);    

        num_InputArgs = *getNbInputArgument(pvApiCtx);

        sciErr = getVarAddressFromPosition(pvApiCtx, 1, &piAddr); 
        if (sciErr.iErr)
        {
            printError(&sciErr, 0); 
            return 0; 
        }

        if(!isListType(pvApiCtx, piAddr))
    	{
        	Scierror(999, "Error: Invalid first argument. List Expected.\n");
        	return 0;
    	}

		/////////////////// State Transition model////////////	
    	sciErr = getMatrixOfDoubleInList(pvApiCtx, piAddr,1, &iRows, &iCols, &temp);
    	if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
		if (iRows!=iCols)
		{
			Scierror(999,"Transition matrix must be a sqaure matrix.\n");
        	return 0;
		}	
		dynamParams=iRows;
		
   		Mat _trMatrix(iRows,iCols,CV_64F,temp);

		/////////////////// measurementMatrix////////////	
		sciErr = getMatrixOfDoubleInList(pvApiCtx, piAddr,2, &iRows, &iCols, &temp);
    	if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
		if (iCols!=dynamParams)
		{
			Scierror(999,"Invalid dimensions of measurement matrix.\n");
        	return 0;
		}
		measureParams=iRows;
		Mat _measureMatrix(iRows,iCols,CV_64F,temp);
		///////////////////control matrix////////////		
    	sciErr = getMatrixOfDoubleInList(pvApiCtx, piAddr,3, &iRows, &iCols, &temp);
    	if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
		
		controlParams=iCols;
		Mat _controlMatrix(iRows,iCols,CV_64F,temp);
		/////////////////// State matrix////////////	
		sciErr = getMatrixOfDoubleInList(pvApiCtx, piAddr,4, &iRows, &iCols, &temp);
    	if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
		if (iCols!=1)
		{
			Scierror(999,"Invalid dimensions of state vector.\n");
        	return 0;
		}	
		Mat _stateMatrix(iRows,iCols,CV_64F,temp);
		/////////////////// state covariance matrix////////////	
		sciErr = getMatrixOfDoubleInList(pvApiCtx, piAddr,5, &iRows, &iCols, &temp);
    	if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
		Mat _stateCovMatrix(iRows,iCols,CV_64F,temp);	
		/////////////////// process noise model////////////	
		sciErr = getMatrixOfDoubleInList(pvApiCtx, piAddr,6, &iRows, &iCols, &temp);
    	if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
		Mat _pNoiseMatrix(iRows,iCols,CV_64F,temp);
		///////////////////Measurement Noise model////////////		
		sciErr = getMatrixOfDoubleInList(pvApiCtx, piAddr,7,&iRows, &iCols, &temp);
    	if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
		Mat _mNoiseMatrix(iRows,iCols,CV_64F,temp);
		sciErr = getVarAddressFromPosition(pvApiCtx, 2, &piAddr); 
        if (sciErr.iErr)
        {
            printError(&sciErr, 0); 
            return 0; 
        }

        sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &iRows, &iCols, &temp);
    	if(sciErr.iErr)
    	{
        	printError(&sciErr, 0);
        	return 0;
    	}

    	Mat _location(iRows,iCols,CV_64F,temp);
    	if (iRows!=measureParams)
    	{
    		Scierror(999,"Invalid dimensions of location vector.Must be a coloumn vector\n");
        	return 0;
    	}

		//*********************Actual processing********************************//
		KalmanFilter KF(dynamParams,measureParams,controlParams,CV_64F);
		KF.statePre=_stateMatrix;
		KF.statePost=_stateMatrix;
		KF.measurementMatrix=_measureMatrix;
		KF.processNoiseCov=_pNoiseMatrix;
		KF.measurementNoiseCov=_mNoiseMatrix;
		KF.transitionMatrix=_trMatrix;
		KF.errorCovPost=_stateCovMatrix;
		KF.errorCovPre=_stateCovMatrix;		
		KF.controlMatrix=_controlMatrix;
		cout<<_location<<endl;
		Mat estimated = KF.correct(_location);

		_nstateMatrix=KF.statePost;
		_nmeasureMatrix=KF.measurementMatrix;
		_npNoiseMatrix=KF.processNoiseCov;
		_nmNoiseMatrix=KF.measurementNoiseCov;
		_ntrMatrix=KF.transitionMatrix;
		_nstateCovMatrix=KF.errorCovPost;
		_ncontrolMatrix=KF.controlMatrix;

		trMatrix=_ntrMatrix.ptr<double>(0);
		stateMatrix=_nstateMatrix.ptr<double>(0);
		measureMatrix=_nmeasureMatrix.ptr<double>(0);
		pNoiseMatrix=_npNoiseMatrix.ptr<double>(0);
		mNoiseMatrix=_nmNoiseMatrix.ptr<double>(0);
		stateCovMatrix=_nstateCovMatrix.ptr<double>(0);
		controlMatrix=_ncontrolMatrix.ptr<double>(0);

		iCols=estimated.cols; 
		iRows=estimated.rows;

		temp=estimated.ptr<double>(0);
		cout<<estimated<<endl;

		sciErr = createList(pvApiCtx, nbInputArgument(pvApiCtx) + 1, 7, &piAddr);
	    if(sciErr.iErr)
	    {
		    printError(&sciErr, 0);
		    return 0;
	    }

	    sciErr = createMatrixOfDoubleInList(pvApiCtx, nbInputArgument(pvApiCtx)+1, piAddr, 1, dynamParams,dynamParams ,trMatrix);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
         sciErr = createMatrixOfDoubleInList(pvApiCtx, nbInputArgument(pvApiCtx)+1, piAddr, 2, measureParams,dynamParams ,measureMatrix);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
         sciErr = createMatrixOfDoubleInList(pvApiCtx, nbInputArgument(pvApiCtx)+1, piAddr, 3, dynamParams,controlParams ,controlMatrix);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
         sciErr = createMatrixOfDoubleInList(pvApiCtx, nbInputArgument(pvApiCtx)+1, piAddr, 4, dynamParams,1 ,stateMatrix);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
         sciErr = createMatrixOfDoubleInList(pvApiCtx, nbInputArgument(pvApiCtx)+1, piAddr, 5, dynamParams,dynamParams ,stateCovMatrix);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
         sciErr = createMatrixOfDoubleInList(pvApiCtx, nbInputArgument(pvApiCtx)+1, piAddr, 6, dynamParams,dynamParams ,pNoiseMatrix);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }

        sciErr = createMatrixOfDoubleInList(pvApiCtx, nbInputArgument(pvApiCtx)+1, piAddr, 7, measureParams,measureParams ,mNoiseMatrix);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        

		sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx)+2,iRows,iCols,temp);
        if(sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        //------Return Arguments------//
	    AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx)+1;
	    AssignOutputVariable(pvApiCtx, 2) = nbInputArgument(pvApiCtx)+2;
	    ReturnArguments(pvApiCtx);
	    return 0;	
	}    

}

