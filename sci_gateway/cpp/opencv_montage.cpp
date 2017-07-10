/***************************************************
Author : Sukul Bagai & M Avinash Reddy
Creates a montage with 1-6 input images
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
    #include "../common.h"

int opencv_montage(char *fname, unsigned long fname_len)
{
    SciErr sciErr;
    int iLen        = 0;
    //variable info
    int iRows       = 0;
    int iCols       = 0;
    int *piAddr     = NULL;
    int *piAddr2     = NULL;
    int *piAddr3     = NULL;
    int *piAddrChild = NULL;
    int *piAddrNew  = NULL;
    int piRows      = 0;
    int piCols      = 0;        
    int *piLen      = NULL;
    char **pstData  = NULL;
    int **pstData1  = NULL;
    int i,j,k=0,iItem=0,height,width,intErr=0,l,num;
    double new_rows,new_cols;
    unsigned char *pstDataR = NULL;
    unsigned char *pstDataG = NULL;
    unsigned char *pstDataB = NULL;

    CheckInputArgument(pvApiCtx, 3, 8);
    CheckOutputArgument(pvApiCtx, 1, 1) ;
    

    try{
    
    num=*getNbInputArgument(pvApiCtx);
    num=num-2;
    iItem=num;
    vector<Mat> image;
    for(l=0;l<num;l++){
    	Mat tempimage;
    	retrieveImage(tempimage,l+3);
    	tempimage.convertTo(tempimage,CV_8U);
    	image.push_back(tempimage);
    }




	sciErr = getVarAddressFromPosition(pvApiCtx, 1, &piAddr2);
      if (sciErr.iErr)
      {
          printError(&sciErr, 0);
          return 0;
      }
      if ( !isDoubleType(pvApiCtx, piAddr2))
      {
          Scierror(999, _("%s: Wrong type for input argument #%d: A scalar expected.\n"), fname, 1);
          return 0;
      }
      intErr = getScalarDouble(pvApiCtx, piAddr2,&new_rows);
      if (intErr)
      {
          return intErr;
      }

	sciErr = getVarAddressFromPosition(pvApiCtx, 2, &piAddr3);
      if (sciErr.iErr)
      {
          printError(&sciErr, 0);
          return 0;
      }
      if ( !isDoubleType(pvApiCtx, piAddr3))
      {
          Scierror(999, _("%s: Wrong type for input argument #%d: A scalar expected.\n"), fname, 2);
          return 0;
      }
      intErr = getScalarDouble(pvApiCtx, piAddr3,&new_cols);
      if (intErr)
      {
          return intErr;
      }

    for(i=1;i<iItem;i++)
    {
        if((image[i].rows != image[i-1].rows) || (image[i].cols != image[i-1].cols))
        {
            Scierror(999,("Images not of same size. Montage did not take place\n"));
            return 0;
        }
    }
    if( ((new_rows-1)*new_cols >= iItem) || ((new_rows*new_cols) < iItem) )
    {
        Scierror(999,("Given Row and Column Dimensions do not match with number of images in list. Montage cannot be created. Please check arguments\n"));
        return 0;
    }
    height=new_rows*image[0].rows;
    width=new_cols*image[0].cols;

    Mat new_image(height,width,CV_8UC3);
    
    k=0;
    
    for(int x=0;x<new_rows;x++)
    {
        for(int y=0;y<new_cols;y++)
        {

            if(k<iItem)
            {
                for(i=0;i<image[x+y].rows;i++)
                {
                    for(j=0;j<image[x+y].cols;j++)
                    {
           				new_image.at<Vec3b>(image[k].rows*x+i,image[k].cols*y+j)[2]=image[k].at<Vec3b>(i,j)[2];
                        new_image.at<Vec3b>(image[k].rows*x+i,image[k].cols*y+j)[1]=image[k].at<Vec3b>(i,j)[1];
                        new_image.at<Vec3b>(image[k].rows*x+i,image[k].cols*y+j)[0]=image[k].at<Vec3b>(i,j)[0];
                    }
                }
            }
            k++;
        }
    }
   

    string tempstring = type2str(new_image.type());
    char *checker;
    checker = (char *)malloc(tempstring.size() + 1);
    memcpy(checker, tempstring.c_str(), tempstring.size() + 1);
    returnImage(checker,new_image,1);
    free(checker);
	}

	catch(cv::Exception& e){
		const char* err=e.what();
		Scierror(999,("%s",err));
	}
    //Assigning the list as the Output Variable
    AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
    //Returning the Output Variables as arguments to the Scilab environment
    ReturnArguments(pvApiCtx);

    return 0;
}
/* ==================================================================== */
}
