/********************************************************
Author: Manoj Sree Harsha
********************************************************/
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <vector>
using namespace std;
extern "C"
{
	#include "api_scilab.h"
	#include "Scierror.h"
	#include "BOOL.h"
	#include <localization.h>
	#include "sciprint.h"

	int PCL_radsearch(char *fname, unsigned long fname_len)
	{
  	    
  	    // Error management variables
		SciErr sciErr;
		int intErr;

		// variables required to read argument #1
		int *piAddr1 = NULL;
		int rows1 = 0, cols1 = 0;
		double* points = NULL;  // pointCloud datapoints

		// variables required to read argument #2
		int *piAddr2 = NULL;
		int rows2=0,cols2=0;
		double *qpoint=NULL;    // contains the query point

		// variables required to read argument #3
		int *piAddr3 = NULL;
		double r=0;             // radius

        //Output Matrix containing indices and distances.
		double *result=NULL;

		//Checking number of input and output arguments (enviromnet variable, min arguments, max arguments)
		CheckInputArgument(pvApiCtx, 3, 3);
    	CheckOutputArgument(pvApiCtx, 1, 1);

    	// to get the argument #1
		sciErr = getVarAddressFromPosition(pvApiCtx, 1, &piAddr1);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
		// Check that the first input argument is a real matrix (and not complex)
		if( !isDoubleType(pvApiCtx, piAddr1) ||  isVarComplex(pvApiCtx, piAddr1) )
		{
			Scierror(999, "point cloud should consist of real values.");
			return 0;
		}
		sciErr = getMatrixOfDouble(pvApiCtx, piAddr1, &rows1, &cols1, &points);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}

		// to get the argument #2
		sciErr = getVarAddressFromPosition(pvApiCtx, 2, &piAddr2);

		if (sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
		// Check that the first input argument is a real vector (and not complex)
		if ( !isDoubleType(pvApiCtx, piAddr2) ||  isVarComplex(pvApiCtx, piAddr2) )
		{
			Scierror(999, _("%s: Wrong type for input argument #%d: A real vector expected.\n"), fname, 2);
			return 0;
		}
		sciErr = getMatrixOfDouble(pvApiCtx, piAddr2, &rows2, &cols2, &qpoint);
		if (sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}

		// to get the argument #3	
		sciErr = getVarAddressFromPosition(pvApiCtx, 3, &piAddr3);
		if (sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
		if ( !isDoubleType(pvApiCtx, piAddr3))
		{
			Scierror(999, _("%s: Wrong type for input argument #%d: A scalar expected.\n"), fname, 3);
			return 0;
		}
		intErr = getScalarDouble(pvApiCtx, piAddr3,&r);
		if (intErr)
		{
			return intErr;
		}
        
        // Check that the second argument is a vector or not 
		if(rows2!=1)
		{
			Scierror(999, _("%s: Invalid input argument #%d: A vector expected.\n"),fname,2);
			return 0;
		}

		//Application Code
		try
		{
			//Creates and fills a PointCloud with the data given by user.
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
			cloud->width  = rows1;
		  	cloud->height = 1;
		  	cloud->points.resize (cloud->width*cloud->height);
			for(size_t i = 0; i < cloud->points.size (); ++i)
			{
				cloud->points[i].x = points[i];
				cloud->points[i].y = points[i + (int)rows1*1 ];
				cloud->points[i].z = points[i + 2*(int)rows1*1];
			}

            //Creates kdtree object and sets our created cloud as the input
			pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
			kdtree.setInputCloud (cloud);

			//Creates a “searchPoint” which is assigned with values of querypoint.
			pcl::PointXYZ searchPoint;
			searchPoint.x = qpoint[0];
		  	searchPoint.y = qpoint[1];
		  	searchPoint.z = qpoint[2];

		  	//Creates 2 vectors for storing information about our points.
			std::vector<int> pointIdxRadiusSearch;
		  	std::vector<float> pointRadiusSquaredDistance;

		  	//Calling Function 'radiusSearch'
			kdtree.radiusSearch (searchPoint, r, pointIdxRadiusSearch, pointRadiusSquaredDistance);

			//Storing indices and distances in output matrix 'result'
			int count=0;
			result=(double*)malloc(sizeof(double)*2*pointIdxRadiusSearch.size ());
		    for (int i=0;i<2*pointIdxRadiusSearch.size ();i++)
		    {
			    if(i%2==0)
			    {
			     	result[i]=(double)pointIdxRadiusSearch[count];
			    }
			    else
			    {
			     	result[i]=(double)pointRadiusSquaredDistance[count];
			     	count=count+1;
			    }
		    }

		    //creating the output matrix
			sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 1,2,pointIdxRadiusSearch.size (),result);
			if (sciErr.iErr)
			{
				printError(&sciErr, 0);
				return sciErr.iErr;
			}
		}
		catch(cv::Exception& e)
		{
		 	const char* err=e.what();
		 	Scierror(999,("%s"),err);
	 	}

	 	//Assigning output variables
		AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;

		//Returning the Output Variables as arguments to the Scilab environment
		ReturnArguments(pvApiCtx);
		return 0;
	}
}
