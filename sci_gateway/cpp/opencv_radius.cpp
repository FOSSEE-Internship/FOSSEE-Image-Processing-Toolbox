/********************************************************
Author: Manoj Sree Harsha 
********************************************************/
#include <numeric>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/flann/flann.hpp"
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

int opencv_radius(char *fname, unsigned long fname_len)
{

    // Error management variables
    SciErr sciErr;
    int intErr=0;

    //variable declarations
    int *piaddrvar1=NULL;
    int *piaddrvar2=NULL;
    int *piaddrvar3=NULL;
    int *piaddrvar4=NULL;


    double *qpoint=NULL;//contains the query point
    double r=0; //radius
    double choice;
    int r1=0,c1=0;
    int i,j,l,count,n;
    double*result = NULL;//output matrix containing the indices and corresponding distances.

    //Checking the number of input and output arguments
    CheckInputArgument(pvApiCtx, 3, 4);
    CheckOutputArgument(pvApiCtx, 1, 1);
    n=*getNbInputArgument(pvApiCtx);
    /* get Address of inputs */
    sciErr = getVarAddressFromPosition(pvApiCtx, 1, &piaddrvar1);
    if (sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }
     /* Check that the first input argument is a real matrix (and not complex) */
    if ( !isDoubleType(pvApiCtx, piaddrvar1) ||  isVarComplex(pvApiCtx, piaddrvar1) )
    {
        Scierror(999, _("%s: Wrong type for input argument #%d: A real vector expected.\n"), fname, 1);
        return 0;
    }
    sciErr = getMatrixOfDouble(pvApiCtx, piaddrvar1, &r1, &c1, &qpoint);
    if (sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }
    if(r1!=1)
    {
        Scierror(999, _("%s: Wrong type for input argument #%d: A vector is expected\n"), fname,1);
        return 0;
    }

    //Application Code
    try
    {

        Mat image,img,imgray,image1;
        retrieveImage(image, 2);    //the input image given by the user is loaded into image(Mat container)
        //converting image to cvtColor compatible type
        if(image.channels()>1)
        {
            image.convertTo(img,CV_32F);
            cvtColor(img,imgray,COLOR_BGR2GRAY);
            imgray.convertTo(image1,CV_64F);
        }
        else
        {
            image.convertTo(image1,CV_64F);
        }
        if(c1!=image1.cols)  //Checking whether cols in querypoint are equal to no. of dimensions of datapoints
        {
            Scierror(999, _("%s: Wrong value for input argument #%d: cols. in query should match with no. of columns in matrix\n"), fname,1);
            return 0;
        }
        //the vector querypoint is loaded into qpoi(Mat container)
         Mat qpoi=Mat(1,image1.cols,CV_64F);
            for(i=0;i<image1.cols;i++){
                qpoi.at<double>(0,i)=qpoint[i];
            }
        sciErr = getVarAddressFromPosition(pvApiCtx, 3, &piaddrvar3);
        if (sciErr.iErr)
        {
            printError(&sciErr, 0);
            return 0;
        }
        if ( !isDoubleType(pvApiCtx, piaddrvar3))
        {
            Scierror(999, _("%s: Wrong type for input argument #%d: A scalar expected.\n"), fname, 3);
            return 0;
        }

        intErr = getScalarDouble(pvApiCtx, piaddrvar3,&r);
        if (intErr)
        {
            return intErr;
        }
        if(n==4)
        {
          sciErr = getVarAddressFromPosition(pvApiCtx, 4, &piaddrvar4);
          if (sciErr.iErr)
          {
              printError(&sciErr, 0);
              return 0;
          }
          if ( !isDoubleType(pvApiCtx, piaddrvar4))
          {
              Scierror(999, _("%s: Wrong type for input argument #%d: A scalar expected.\n"), fname, 4);
              return 0;
          }

          intErr = getScalarDouble(pvApiCtx, piaddrvar4,&choice);
          if (intErr)
          {
              return intErr;
          }
        }
        else{
          choice=1;
        }
        if(choice<1 || choice >4)
        {
            Scierror(999, _("%s: Wrong value for input argument #%d: A value between 1 and 4(inclusively) is expected\n"), fname,4);
            return 0;
        }
        if(r<0)
        {
            Scierror(999, _("%s: Wrong value for input argument #%d: A positive value is expected\n"), fname,3);
            return 0;
        }
            //Creating a new object of GenericIndex
            //the index will perform a linear,brute-force search.
            //the distance type used here is euclidean distance(L2)
        Mat indices=Mat(1,image1.rows,CV_32S);//contains the indices of points within 'r' radius
        Mat dists=Mat(1,image1.rows,CV_64F);//contains corresponding distances
        int retval;

        if(choice==1)
        {
            cv::flann::GenericIndex< cvflann::L2<double> > index(image1,cvflann::LinearIndexParams(),cvflann::L2<double>());
            retval = index.radiusSearch(qpoi,indices,dists,r,cvflann::SearchParams());//OpenCv function radiusSearch is called.
        }

        if(choice==2)
        {
            cv::flann::GenericIndex< cvflann::L2<double> > index(image1,cvflann::KDTreeIndexParams(),cvflann::L2<double>());
            retval = index.radiusSearch(qpoi,indices,dists,r,cvflann::SearchParams());//OpenCv function radiusSearch is called.
         }
        if(choice==3)
        {
            cv::flann::GenericIndex< cvflann::L2<double> > index(image1,cvflann::KMeansIndexParams(),cvflann::L2<double>());
            retval = index.radiusSearch(qpoi,indices,dists,r,cvflann::SearchParams());//OpenCv function radiusSearch is called.
        }
        if(choice==4)
        {
            cv::flann::GenericIndex< cvflann::L2<double> > index(image1,cvflann::CompositeIndexParams(),cvflann::L2<double>());
            retval = index.radiusSearch(qpoi,indices,dists,r,cvflann::SearchParams());//OpenCv function radiusSearch is called.
        }
        sciprint("Number of points : %d",retval);
        result = (double*)malloc(sizeof(double)*2*retval);
        int count=0;
        for(j=0;j<2*retval;j++)
        {
            if(j%2==0)
            {
                result[j]=(double)indices.at<int>(0,count);
            }
            else
            {
                 result[j]=dists.at<double>(0,count);
                 count++;
            }
        }
        //creating the output matrix
        sciErr = createMatrixOfDouble(pvApiCtx,nbInputArgument(pvApiCtx)+1,2,retval,result);
        if (sciErr.iErr)
        {
         printError(&sciErr, 0);
         return sciErr.iErr;
        }
    }
    catch(cv::Exception& e)
    {
        const char* err=e.what();
        sciprint("%s",err);
    }

    //Assigning output variables
    AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;

    //Returning the Output Variables as arguments to the Scilab environment
    ReturnArguments(pvApiCtx);
    return 0;
    }
}
