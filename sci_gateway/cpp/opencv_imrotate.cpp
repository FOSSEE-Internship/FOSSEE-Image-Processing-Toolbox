/* ==================================================================== */
/* Authors :Priyanka Hiranandani NIT Surat, Shubham Lohakare NITK Surathkal  */
/* ==================================================================== */
/* Syntax : rotated_image=imrotate(sourcrimage,angle);        
	    rotated_image=imrotate(sourceimage,angle,'crop');
	    rotated_image=imrotate(sourceimage,angle,'loose');                                                  */
/* ==================================================================== */
#include <numeric>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <sciprint.h>
using namespace cv;
using namespace std;
  	extern "C"
  	{
  		#include "api_scilab.h"
  		#include "Scierror.h"
		#include "BOOL.h"
	  	#include <localization.h>
  		#include "../common.h"
  		int opencv_imrotate(char *fname, unsigned long fname_len)
    		{
  			// Error management variable
        		SciErr sciErr;
  			//variable info
			int iRows		= 0;
			int iCols		= 0;
        		int piRows		= 0;
			int piCols		= 0;
			int* piAddr		= NULL;
        		int* piAddr2            = NULL;
        		int* piAddr3            = NULL;
        		int* piAddrNew            = NULL;
			int* piLen		= NULL;
			double angle;
        		char **pstData2    	= NULL;
        		double *rrows;
        		double *rcols;
        		int error;
   			//function to check number of input variable 
        		CheckInputArgument(pvApiCtx, 2, 3);
   			//function to check number of output variable
           		CheckOutputArgument(pvApiCtx, 1, 1);
   			
			int n = *getNbInputArgument(pvApiCtx);//Getting the total number of input variables

			// get Address of first input 
		           Mat src,dst; 
		           retrieveImage(src,1); //Retrieving the first image 
	
			  sciErr = getVarAddressFromPosition(pvApiCtx,2,&piAddr2);
		           //checking for error if any
		           if (sciErr.iErr)
		           {
			           printError(&sciErr, 0);
			           return 0;
		           }
		           //this function will fetch second argument i.e angle
		           error=getScalarDouble(pvApiCtx,piAddr2,&angle); 
		           if(error!=0)
		           {
		                  sciprint("error in retrieving second argument");
		           }      
			if(n==2)//If number of input arguments are 2
			{
				cv::Point2f pc(src.cols/2., src.rows/2.);
    				cv::Mat r = cv::getRotationMatrix2D(pc, angle, 1.0);
				
    				cv::warpAffine(src, dst, r, src.size());
			}
			if(n==3)//If number of arguments are 3
			{	
				//Getting the third variable which is the type of bounding box required
				sciErr = getVarAddressFromPosition(pvApiCtx,3,&piAddr3);
        			sciErr = getMatrixOfString(pvApiCtx, piAddr3, &iRows, &iCols, NULL, NULL);
       				if(sciErr.iErr)
               			{
               				printError(&sciErr, 0);
               				return 0;
               			}
         			piLen = (int*)malloc(sizeof(int) * iRows * iCols);
     				//second call to retrieve length of each string of first argument
         			sciErr = getMatrixOfString(pvApiCtx, piAddr3, &iRows, &iCols, piLen, NULL);
        			if(sciErr.iErr)
               			{
               				printError(&sciErr, 0);
               				return 0;
               			}	
         			pstData2 = (char**)malloc(sizeof(char*) * iRows * iCols);
         			for(int i = 0 ; i < iRows * iCols ; i++)
               			{	
                			pstData2[i] = (char*)malloc(sizeof(char) * (piLen[i] + 1));//+ 1 for null termination
               			}
    				//third call to retrieve data of each string of first argument
         			sciErr = getMatrixOfString(pvApiCtx, piAddr3, &iRows, &iCols, piLen, pstData2);
         			if(sciErr.iErr)
               			{
               				printError(&sciErr, 0);
               				return 0;
               			}
        
        			if(!strcmp("crop",pstData2[0]))//If type of bounding box is crop
              			{
                			cv::Point2f pc(src.cols/2., src.rows/2.);
    					cv::Mat r = cv::getRotationMatrix2D(pc, angle, 1.0);

    					cv::warpAffine(src, dst, r, src.size());
              			}
        			else if(!strcmp("loose",pstData2[0]))//If type of bounding box is loose
              			{
               				cv::Point2f center(src.cols/2.0, src.rows/2.0);
    					cv::Mat rot = cv::getRotationMatrix2D(center, angle, 1.0);
    					// determine bounding rectangle
    					cv::Rect bbox = cv::RotatedRect(center,src.size(), angle).boundingRect();
    					// adjust transformation matrix
    					rot.at<double>(0,2) += bbox.width/2.0 - center.x;
    					rot.at<double>(1,2) += bbox.height/2.0 - center.y;

    			
    					cv::warpAffine(src, dst, rot, bbox.size());
              			}
			}
			//The image which will be shown as output
	 		string tempstring = type2str(dst.type());
            		char* checker = (char *)malloc(tempstring.size() + 1);
           		memcpy(checker, tempstring.c_str(), tempstring.size() + 1);
            		returnImage(checker,dst,1);
            		AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
            		ReturnArguments(pvApiCtx);
       		}
}
