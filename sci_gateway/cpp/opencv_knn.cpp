//Author- M Avinash Reddy

#include <numeric>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/flann/flann.hpp"
#include "opencv2/imgproc/imgproc.hpp"
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

int opencv_knn(char *fname, unsigned long fname_len){


  SciErr sciErr;
	int intErr=0;
	int *piaddrvar1=NULL;
	int *piaddrvar2=NULL;
	int *piaddrvar3=NULL;
	int *piaddrvar4=NULL;

	double *qpoint=NULL; // contains the query point
	double k=0,algo=0;          // k nearest neighbours
	int r1=0,c1=0;
	int i,j,l,count,n;
	double *result=NULL;  // output matrix containing the indices(of nearest neighbours) and the corresponding distances

    CheckInputArgument(pvApiCtx, 3, 4); // Checking the number of input and output arguments
    CheckOutputArgument(pvApiCtx, 1, 1);
    	/* get Address of inputs */
    n=*getNbInputArgument(pvApiCtx);
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


    try{
    	Mat image,I;

    	Mat image1;
    	retrieveImage(image, 2);  // the input image or hypermatrix by the user is loaded into image(Mat container)


    	if(c1!=image.cols){
			Scierror(999, _("%s: Invalid input argument #%d: The dimensions of query point should be same as that of the data points\n"), fname,1);
        	return 0;
    	}
    	Mat final_gray;
    	if(image.channels()>1){

    	image.convertTo(I,CV_32F);  //cvtColor works only for particular depths
    //sciprint("%d\n",I.type());

    	cvtColor(I,final_gray,COLOR_BGR2GRAY);
    //sciprint("%d\n",final_gray.type());
    	final_gray.convertTo(image1,CV_64F);


    	}
    	else{

    		image.convertTo(image1,CV_64F);

    	}

    	Mat qpoi=Mat(1,image1.cols,CV_64F);
    	for(i=0;i<image1.cols;i++){
    		qpoi.at<double>(0,i)=qpoint[i];   //converting query point into a Mat container
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

	    intErr = getScalarDouble(pvApiCtx, piaddrvar3,&k);
	    if (intErr)
	    {

	        return intErr;
	    }

      if(n==4){

  	    sciErr = getVarAddressFromPosition(pvApiCtx, 4, &piaddrvar4);
  	    if (sciErr.iErr)
  	    {
  	        printError(&sciErr, 0);
  	        return 0;
  	    }
  	    if ( !isDoubleType(pvApiCtx, piaddrvar4))
  	    {
  	        Scierror(999, _("%s: Wrong type for input argument #%d: A scalar expected.\n"), fname, 3);
  	        return 0;
  	    }

  	    intErr = getScalarDouble(pvApiCtx, piaddrvar4,&algo);
  	    if (intErr)
  	    {

  	        return intErr;
  	    }

      }
      else{
        algo=1;
      }

	    if(r1!=1){
			Scierror(999, _("%s: Wrong type for input argument #%d: A vector is expected\n"), fname,1);
	        return 0;
	    }
	    if(algo<1 || algo>4){
			Scierror(999, _("%s: Invalid input argument #%d: A number between 1 and 4 is expected.(set 1 for default linearsearch)\n"), fname,4);
	        return 0;
	    }

	    if(k>image1.rows){
			Scierror(999, _("%s: Invalid input argument #%d: K shouldn't exceed the number of rows in the matrix\n"), fname,3);
	        return 0;
	    }
	    if(k<0){
			Scierror(999, _("%s: Invalid input argument #%d: K cannot be negative\n"), fname,3);
	        return 0;
	    }

   		//Creating a new object of class GenericIndex
   		//the index will perform a linear,brute-force search
   		//the distance type used is euclidian distance(L2)
   		Mat indices=Mat(1,k,CV_32S); //contains the indices of the k nearest neighbours
	    Mat dists=Mat(1,k,CV_64F);

	    if(algo==1){
	    	cv::flann::GenericIndex< cvflann::L2<double> > index(image1,cvflann::LinearIndexParams(),cvflann::L2<double>());
	    	index.knnSearch(qpoi, indices, dists, k,cvflann::SearchParams()); //OpenCV function is called
	    }

	    else if(algo==2){
	    	cv::flann::GenericIndex< cvflann::L2<double> > index(image1,cvflann::KDTreeIndexParams(),cvflann::L2<double>());
	    	index.knnSearch(qpoi, indices, dists, k,cvflann::SearchParams()); //OpenCV function is called
	    }

	    else if(algo==3){
	    	cv::flann::GenericIndex< cvflann::L2<double> > index(image1,cvflann::KMeansIndexParams(),cvflann::L2<double>());
	    	index.knnSearch(qpoi, indices, dists, k,cvflann::SearchParams()); //OpenCV function is called
	    }

	    else{
	    	cv::flann::GenericIndex< cvflann::L2<double> > index(image1,cvflann::CompositeIndexParams(),cvflann::L2<double>());
	    	index.knnSearch(qpoi, indices, dists, k,cvflann::SearchParams()); //OpenCV function is called
	    }

	    result=(double*)malloc(sizeof(double)*2*k);
	    count=0;

	    for (i=0;i<2*k;i++){
	     	if(i%2==0){
	     		result[i]=(double)indices.at<int>(0,count);
	     	}
	     	else{
	     		result[i]=dists.at<double>(0,count);
	     		count=count+1;
	     	}
	    }



	 }
	 catch(cv::Exception& e){
	 	const char* err=e.what();
	 	sciprint("%s",err);
	 }

	sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 1,2,k,result); //creating the output matrix
	if (sciErr.iErr){
	    printError(&sciErr, 0);
	    return sciErr.iErr;
	}

    AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
    ReturnArguments(pvApiCtx);
    return 0;

}


}
