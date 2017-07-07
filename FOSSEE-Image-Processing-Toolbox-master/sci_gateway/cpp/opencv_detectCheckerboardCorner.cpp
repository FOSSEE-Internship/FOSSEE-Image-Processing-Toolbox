#include <numeric>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/calib3d/calib3d.hpp"
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
  
  int opencv_detectCheckerboardCorner(char *fname, unsigned long fname_len)
  {
    SciErr sciErr;
    int intErr = 0;
    int *piAddr  = NULL;
    double *flags = NULL;
    double *patternSize=NULL;
    double flag=0;
    int iRows,iCols;
    Size patternSz;
    int found ;
    double *corner;
    //double k;

    CheckInputArgument(pvApiCtx, 2, 3);
    CheckOutputArgument(pvApiCtx, 1, 1);
    //int n=*getNbInputArgument();//get number of input arguments


    Mat image;
    retrieveImage(image,1);
    image.convertTo(image,CV_8UC1);
    sciprint("image converted\n");
    //for blocksize
        sciErr = getVarAddressFromPosition(pvApiCtx,2,&piAddr);
        if (sciErr.iErr)
        {
        printError(&sciErr, 0);
        return 0;
        }
        sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &iRows, &iCols, &patternSize); ///paternSize to be given as height and width of the checkboard
        if(sciErr.iErr)
       {
           printError(&sciErr, 0);
          return 0;
        }

sciprint("pattern size retrieved\n");
     //for flag
    if (*getNbInputArgument(pvApiCtx)>2)
    {
        sciErr = getVarAddressFromPosition(pvApiCtx,3,&piAddr);
        if (sciErr.iErr)
        {
        printError(&sciErr, 0);
        return 0;
        }
        sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &iRows, &iCols, &flags);
        if(sciErr.iErr)
        {
          printError(&sciErr, 0);
          return 0;
        }
        sciprint("flags retrievedd\n");
         for (int i=0;i<iCols;i++)
        {
          flag = flag + flags[i];
        }

        sciprint("flag created\n");
      }
///////////Actual Processing/////////////////////////////////////////////////////
        sciprint("flag: %f \n" ,flag);
        patternSz=Size(patternSize[0]-1,patternSize[1]-1);
        sciprint("patternSz: %f %f\n" ,patternSize[1],patternSize[0]);

       
////////////////////////flags interpretation////////////////////

       // Value        flag 
       // 1            CV_CALIB_CB_ADAPTIVE_THRESH Use adaptive thresholding to convert the image to black and white, rather than a fixed threshold level (computed from the average image brightness).
       // 2             CV_CALIB_CB_NORMALIZE_IMAGE Normalize the image gamma with equalizeHist before applying fixed or adaptive thresholding.
       // 4             CV_CALIB_CB_FILTER_QUADS Use additional criteria (like contour area, perimeter, square-like shape) to filter out false quads extracted at the contour retrieval stage.
       // 8             CALIB_CB_FAST_CHECK Run a fast check on the image that looks for chessboard corners, and shortcut the call if none is found. This can drastically speed up the call in the degenerate condition when no chessboard is observed.
 
        if (flag==0)
          flag=3;

    vector<Point2f> corners ;
    try{
          found=findChessboardCorners(image,patternSz,corners,flag);
        }
    catch(cv::Exception&e)
        {
            const char* err=e.what();
            sciprint("%s",err);
        }  
        
         sciprint("corners\n");  
    
    if(!(found==0))
    {
    cornerSubPix(image, Mat(corners), Size(11, 11), Size(-1, -1),TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    }
    iRows=Mat(corners).rows;
    iCols=Mat(corners).cols;
    sciprint("%d %d",iRows,iCols);

    // drawChessboardCorners(image, patternSz, Mat(corners),found);

    // imshow("opencv",image);
    // waitKey(0);
    corner = (double*) malloc(sizeof(double)*iRows*iCols*2);

    for (int i=0;i<iRows*iCols;i++)
    {
      corner[i]=corners[i].x;
      corner[i+iRows]=corners[i].y;
      //sciprint("%f %f\n",corner[i],corner[i+iRows]);
    }
     sciprint("corners transferred\n");


      if (found==0)
    {    
        sciprint("Unsuccessful detections.Try with different image or parameters\n");
    }
     sciprint("matrix creation\n");
    sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 1,iRows ,iCols*2 ,corner);
    if(sciErr.iErr)
    {
      printError(&sciErr, 0);
      return 0;
    }
    sciprint("output created\n");
    AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx)+1;
    ReturnArguments(pvApiCtx);
    return 0;

  }
}
    
    

