/*
 * integralImage
 *        
 * integralImage in scilab
 *     
 */

// Created by Samiran Roy, Shubham Lohakare
// An implementation of integralImage method of matlab
// Usage: 
// integralImage(I) : Calculate the integral image of I, I must be grayscale

// method : 'upright' (default)
// method : 'rotated' The area sums are calulated over a rectangle, which is rotated 45 degrees


// Known Changes from Matlab:
/*
 * 1) None, as of now
 */

#include <numeric>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
using namespace cv;
using namespace std;
extern "C" {
		#include "api_scilab.h"
		#include "Scierror.h"
		#include "BOOL.h"
		#include <localization.h>
		#include "sciprint.h"
		#include "../common.h"

		int opencv_integralImage(char *fname, unsigned long fname_len) {
			SciErr sciErr;
			int intErr = 0;


			int *piAddr1 = NULL;

			int error;
	
			// String holding the second argument
			int iRet = 0;
			char* pstData = NULL;

			// Checking input argument
			CheckInputArgument(pvApiCtx, 1, 2);
			CheckOutputArgument(pvApiCtx, 3, 3);

			// Get input image

			Mat image;
			retrieveImage(image, 1);
			image.convertTo(image,CV_8UC1);
			cvtColor(image,image,CV_BGR2GRAY);




			// Error Checks

			if (image.channels() > 1) {
				sciprint("The image must be grayscale.");
				return 0;
			}

			// Output variables holding integralImage, squared integralImage, integralImage over rectangle rotated by 45 degrees
			Mat new_image, integralimage, squaredimage, rotatedimage;

			integral(image, integralimage, squaredimage, rotatedimage, -1);

			// Get the number of input arguments
			int inputarg = *getNbInputArgument(pvApiCtx);

			if (inputarg == 1)
				integralimage.copyTo(new_image);

			if (inputarg == 2)
			{
				sciErr = getVarAddressFromPosition(pvApiCtx, 2, &piAddr1);

				if (sciErr.iErr) {
					printError(&sciErr, 0);
					return 0;
					}

				if (isStringType(pvApiCtx, piAddr1)) {
				if (isScalar(pvApiCtx, piAddr1)) {

					iRet = getAllocatedSingleString(pvApiCtx, piAddr1, &pstData);
					}
				}
				if (strcmp(pstData, "rotated") == 0) {
					rotatedimage.copyTo(new_image);
				} else if (strcmp(pstData, "upright") == 0) {
				integralimage.copyTo(new_image);
			} else {
				sciprint("Unknown Parameter Name:%s\n", pstData);

			}

	}

	 

  			// new_image is sent to scilab as output

			
			string tempstring1 = type2str(new_image.type());
			char *checker1;
			checker1 = (char *) malloc(tempstring1.size() + 1);
			memcpy(checker1, tempstring1.c_str(), tempstring1.size() + 1);
			returnImage(checker1, new_image, 1);
			free(checker1);

			string tempstring2 = type2str(squaredimage.type());
			char *checker2;
			checker2 = (char *) malloc(tempstring2.size() + 1);
			memcpy(checker2, tempstring2.c_str(), tempstring2.size() + 1);
			returnImage(checker2, squaredimage, 2);
			free(checker2);

			string tempstring3 = type2str(rotatedimage.type());
			char *checker3;
			checker3 = (char *) malloc(tempstring3.size() + 1);
			memcpy(checker3, tempstring3.c_str(), tempstring3.size() + 1);
			returnImage(checker3, rotatedimage, 3);
			free(checker3);

				

			//Assigning the list as the Output Variable
			AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
			AssignOutputVariable(pvApiCtx, 2) = nbInputArgument(pvApiCtx) + 2;
			AssignOutputVariable(pvApiCtx, 3) = nbInputArgument(pvApiCtx) + 3;
			//Returning the Output Variables as arguments to the Scilab environment
			ReturnArguments (pvApiCtx);
			return 0;

}
/* ==================================================================== */
}

