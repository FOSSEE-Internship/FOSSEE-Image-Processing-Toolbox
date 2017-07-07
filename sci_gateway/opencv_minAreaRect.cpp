/* ==================================================================== */
/* Author :Priyanka Hiranandani NIT Surat, Rohan Gurve GCET, Anand                               */
/* ==================================================================== */
/* Syntax : return_rotatedrect=minAreaRect(x,y);
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
  int opencv_minAreaRect(char *fname, unsigned long fname_len)
    {
    // Error management variable
        SciErr sciErr;
    //variable info
	int x_iRows		= 0;
	int x_iCols		= 0;
    
    int y_iRows		= 0;
	int y_iCols		= 0;

            
    int piRows		= 0;
	int piCols		= 0;
	int* piAddr     = NULL;
    int* piAddr2    = NULL;
    //int* piAddr3    = NULL;
    //int* piAddr4    = NULL;
   // int* piAddr5    = NULL;
    //int* piAddr6    = NULL;
    int* piAddrNew  = NULL;
	int* piLen		= NULL;
	double* x	= NULL;
    double* y	= NULL;
    
    double * rectCoordinates_x = NULL; //to store x coordinates of vertices of the min area rect
    double * rectCoordinates_y = NULL; //to store y coordinates of vertices of the min area rect 
    
      rectCoordinates_x  =  (double*)malloc(sizeof(double) * 1 * 4);
      rectCoordinates_y  =  (double*)malloc(sizeof(double) * 1 * 4);
    double sigmaSpace;
    int error;
     ////checking input argument 
    CheckInputArgument(pvApiCtx,2,2);
     //checking output argument
    CheckOutputArgument(pvApiCtx,7,7);
     
     
     //for first argument 
     // get Address of first input  
     sciErr = getVarAddressFromPosition(pvApiCtx,1,&piAddr);
     //check for any error
     if(sciErr.iErr)   
       {
       printError(&sciErr, 0);
       return -1;
       }      
       
     /* Check that the first input argument is a real matrix (and not complex) */
    if ( !isDoubleType(pvApiCtx, piAddr) ||  isVarComplex(pvApiCtx, piAddr) )
    {
        Scierror(999, "%s: Wrong type for input argument #%d: A real matrix expected.\n", fname, 1);
        return -1;
    }  
       
    //retrieve input array an dstore it in x
    sciErr = getMatrixOfDouble(pvApiCtx, piAddr, &x_iRows, &x_iCols,&x);
     if(sciErr.iErr)
       {
       printError(&sciErr, 0);
       return -1;
       }
       
       
       //for second argument 
      // get Address of second input  
      sciErr = getVarAddressFromPosition(pvApiCtx,2,&piAddr2);
   //check for any error
     if(sciErr.iErr)   
       {
       printError(&sciErr, 0);
       return -1;
       }      
    
    /* Check that the second input argument is a real matrix (and not complex) */
    if ( !isDoubleType(pvApiCtx, piAddr2) ||  isVarComplex(pvApiCtx, piAddr2) )
    {
        Scierror(999, "%s: Wrong type for input argument #%d: A real matrix expected.\n", fname, 2);
        return -1;
    }
    
    //retrieve input array and store it in y
    sciErr = getMatrixOfDouble(pvApiCtx, piAddr2, &y_iRows, &y_iCols,&y);
     if(sciErr.iErr)
       {
       printError(&sciErr, 0);
       return -1;
       }
       
       
       
       //error check
       
       if(!(x_iRows==1 && y_iRows==1) )
       {
            Scierror(999, "%s","x and y matrix expects a dimension of 1xn");
            return -1;            
        
       }
       
       if( !(x_iCols==y_iCols))
        {
            Scierror(999, "%s","size of x matrix is not equal to y matrix");
            return -1;            
       
        }
       
       
    int k=0;
    vector<Point> contours;
    for(int i=0;i<x_iCols;i++)
     {     Point pt;
           pt.x = x[i];
           pt.y = y[i];   
           contours.push_back(pt) ;   
           
     }
   
    RotatedRect rect;
    
    try
    {
        rect=minAreaRect((contours));  	
       
    }
    catch(Exception& e)
    {
    const char* err = e.what();
    Scierror(999, "%s",err);
    return -1;
          
    }
   
   double size_width,size_height,center_x,center_y,angle;
   //from rotated rect datatype we can fetch required size,point,angle
   Size2f sz;      //size2f function for floating point values 
   Point2f cen;
   sz=rect.size;
   cen=rect.center;
   angle=rect.angle;
   size_width=sz.width;
   size_height=sz.height;
   center_x=cen.x;
   center_y=cen.y;
         
    cv::Point2f vertices2f[4];
    rect.points(vertices2f);

   
    // extracting the coordinates of the vertices of this rect
    for(int i = 0; i < 4; ++i){
        rectCoordinates_x[i] = vertices2f[i].x;
        rectCoordinates_y[i] = vertices2f[i].y;
        
        
    }
    
   
   
   
   error=createScalarDouble(pvApiCtx,nbInputArgument(pvApiCtx)+1,size_width);
     if(error!=0)
     {
       Scierror(999, "%s","an error occured");
      return -1;
        
     }  
    error=createScalarDouble(pvApiCtx,nbInputArgument(pvApiCtx)+2,size_height);
     if(error!=0)
     {
       Scierror(999, "%s","an error occured");
      return -1;  
     }  
    error=createScalarDouble(pvApiCtx,nbInputArgument(pvApiCtx)+3,center_x);
     if(error!=0)
     {
      Scierror(999, "%s","an error occured");
      return -1;  
     }   
    error=createScalarDouble(pvApiCtx,nbInputArgument(pvApiCtx)+4,center_y);
     if(error!=0)
     {
       Scierror(999, "%s","an error occured");
      return -1;  
     }   
    error=createScalarDouble(pvApiCtx,nbInputArgument(pvApiCtx)+5,angle);
     if(error!=0)
     {
      Scierror(999, "%s","an error occured");
      return -1;  
     }     
    
    sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 6, 1, 4, rectCoordinates_x);
    free(rectCoordinates_x); // Data have been copied into Scilab memory
    if (sciErr.iErr)
    {
        Scierror(999, "%s","an error occured");
      return -1;
    } 
    
    sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 7, 1, 4, rectCoordinates_y);
    free(rectCoordinates_y); // Data have been copied into Scilab memory
    if (sciErr.iErr)
    {
        printError(&sciErr, 0);
        return -1;
    }     
    
    AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
    AssignOutputVariable(pvApiCtx, 2) = nbInputArgument(pvApiCtx) + 2;
    AssignOutputVariable(pvApiCtx, 3) = nbInputArgument(pvApiCtx) + 3;
    AssignOutputVariable(pvApiCtx, 4) = nbInputArgument(pvApiCtx) + 4;
    AssignOutputVariable(pvApiCtx, 5) = nbInputArgument(pvApiCtx) + 5;
    AssignOutputVariable(pvApiCtx, 6) = nbInputArgument(pvApiCtx) + 6;
    AssignOutputVariable(pvApiCtx, 7) = nbInputArgument(pvApiCtx) + 7;
    
    ReturnArguments(pvApiCtx);      
  }
}


