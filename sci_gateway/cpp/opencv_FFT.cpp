/****************************************
Author: Sridhar Reddy
*****************************************/
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
  #include "../common.h"

  int opencv_FFT(char *fname, unsigned long fname_len)
  {
  		// Error management variables
	  	SciErr sciErr;
	    int intErr=0;

	    // variables required to read arguments
	    int iRows=0,iCols=0,i,j;
	    int *piAddr1=NULL;
	    double *ipReal=NULL;
	    double *ipImg=NULL;
	    double *rsltReal=NULL;
	    double *rsltImg=NULL;
	    vector<complex<double> > vec_out;
	    vector<double> vec_in;

	    //Checking number of input and output arguments (enviromnet variable, min arguments, max arguments)
	  	CheckInputArgument(pvApiCtx, 1, 1);
	  	CheckOutputArgument(pvApiCtx, 1, 1) ;

	  	// to get the argument #1
	 	sciErr = getVarAddressFromPosition(pvApiCtx, 1, &piAddr1);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
	 	sciErr = getMatrixOfDouble(pvApiCtx, piAddr1, &iRows, &iCols, &ipReal);
	 	if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}

		//Application Code
		try
		{

			ipImg = (double*)malloc(sizeof(double) * iRows * iCols);
			if(iRows==1)
			{
				for(i = 0 ; i <iCols ; i++)
						vec_in.push_back(ipReal[i]);
				dft(vec_in,vec_out, DFT_COMPLEX_OUTPUT);
				for(i = 0 ; i < iCols ; i++){
						ipReal[i]=real(vec_out[i]);
						ipImg[i]=imag(vec_out[i]);
					}
				vec_in.clear();
				vec_out.clear();

			}
			else
			{
				for(i = 0 ; i < iCols ; i++){
					for(j = 0 ; j < iRows ; j++)
						vec_in.push_back(ipReal[j+iCols*i]);
					dft(vec_in,vec_out,DFT_COMPLEX_OUTPUT);
					for(j = 0 ; j < iRows ; j++){
						ipReal[j+iCols*i]=real(vec_out[j]);
						ipImg[j+iCols*i]=imag(vec_out[j]);
					}
					vec_in.clear();
					vec_out.clear();
				}
			}
			sciErr = allocComplexMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 1, iRows, iCols, &rsltReal, &rsltImg);
			if(sciErr.iErr)
			{
				printError(&sciErr, 0);
				return 0;
			}
			for(i = 0 ; i < iRows * iCols; i++)
			{
				rsltReal[i]=ipReal[i];
				rsltImg[i]=ipImg[i];
			}
		}
		catch(cv::Exception& e)
        {
            const char* err=e.what();
            Scierror(999,("%s"),err);
        }

        //Assigning the list as the Output Variable
		AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;

		//Returning the Output Variables as arguments to the Scilab environment
		ReturnArguments(pvApiCtx);
		return 0;
  }
}
