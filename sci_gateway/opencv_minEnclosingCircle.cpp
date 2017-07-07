/* ==================================================================== */
/* Author :Priyanka Hiranandani NIT Surat, Rohan Gurve GCET, Anand                               */
/* ==================================================================== */
/* Syntax : return_minEnclosingCircle = minEnclosingCircle(x,y)
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
 // #include "../common.h"
  int opencv_minEnclosingCircle(char *fname, unsigned long fname_len)
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
	int* piAddr		= NULL;
	int* piAddr2            = NULL;
    int* piAddrNew          = NULL;
	int* piLen		= NULL;
	double *pstData	        = NULL;
    char **pstData2    	= NULL;
    //for storing the input matrix
    double* x	= NULL;
    double* y	= NULL;
    
    Point2f center;
    float radius;
    int error;         
     //checking input argument 
    CheckInputArgument(pvApiCtx,2,2);
     //checking output argument
    CheckOutputArgument(pvApiCtx,3,3);
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
        
        
    try
    {
         minEnclosingCircle(contours,center,radius);
      	
       
    }
    catch(Exception& e)
    {
    const char* err = e.what();
    Scierror(999, "%s",err);
    return -1;
          
    }
        
        
        
       
      //////// Return the output arguments to the Scilab engine //////////
        double p1,p2,c; //another way of returning pointer
        p1=center.x;
        p2=center.y;
        c=radius;        
       error=createScalarDouble(pvApiCtx,nbInputArgument(pvApiCtx)+1,p1);
         if(error!=0)
         {
           Scierror(999, "%s"," an error occurred");
           return -1;    
         }  

       error=createScalarDouble(pvApiCtx,nbInputArgument(pvApiCtx)+2,p2);
         if(error!=0)
         {
           Scierror(999, "%s"," an error occurred");
           return -1;    
         }  
       error=createScalarDouble(pvApiCtx,nbInputArgument(pvApiCtx)+3,c);
         if(error!=0)
         {
           Scierror(999, "%s"," an error occurred");
           return -1;     
         }  
        AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
        AssignOutputVariable(pvApiCtx, 2) = nbInputArgument(pvApiCtx) + 2;
        AssignOutputVariable(pvApiCtx, 3) = nbInputArgument(pvApiCtx) + 3;
        ReturnArguments(pvApiCtx);    

        return 0;     
}
}

