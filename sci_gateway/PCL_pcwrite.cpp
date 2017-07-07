#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h>
#include<string>

using namespace std;

extern "C"
{
	#include"api_scilab.h"
	#include"Scierror.h"
	#include"BOOL.h"
	#include<localization.h>
	#include"sciprint.h"

	int PCL_pcwrite(char *fname, unsigned long fname_len)
	{
		// Error management variable
		SciErr sciErr;

		// variables required to read argument #1
		int *piAddr1 = NULL;
		int intErr1 = 0;
		double width = 0;

		// variables required to read argument #2
		int *piAddr2 = NULL;
		int intErr2 = 0;
		double height = 0;

		// variables required to read argument #3
		int *piAddr3 = NULL;
		int intErr3 = 0;
		int dense = false;

		// variables required to read argument #4
		int *piAddr4 = NULL;
		int intErr4 = 0;
		int rows1 = 0, cols1 = 0;
		double* location = NULL;

		// variables required to read argument #5
		int *piAddr5 = NULL;
		int intErr5 = 0;
		double count = 0;
		
		// variables required to read argument #6
		int *piAddr6 = NULL;
		int intErr6 = 0;
		int rows2 = 0, cols2 = 0;
		double* rgb = NULL;

		// variables required to read argument #7
		int *piAddr7 = NULL;
		int intErr7 = 0;
		int rows3 = 0, cols3 = 0;
		double* xlimit = NULL;

		// variables required to read argument #8
		int *piAddr8 = NULL;
		int intErr8 = 0;
		int rows4 = 0, cols4 = 0;
		double* ylimit = NULL;

		// variables required to read argument #9
		int *piAddr9 = NULL;
		int intErr9 = 0;
		int rows5 = 0, cols5 = 0;
		double* zlimit = NULL;

		// variables required to read argument #10
		int *piAddr10 = NULL;
		int intErr10 = 0;
		int rows6 = 0, cols6 = 0;
		int *pilen1 = NULL;
		char **filename = NULL;

		// variables required to read argument #11
		int *piAddr11 = NULL;
		int intErr11 = 0;
		int rows7 = 0, cols7 = 0;
		int *pilen2 = NULL;
		char **fileFormat = NULL;

		// variables required to read argument #12
		int *piAddr12 = NULL;
		int intErr12 = 0;
		int rows8 = 0, cols8 = 0;
		int *pilen3 = NULL;
		char **fileType = NULL;

		int s;
		
		int n= *getNbInputArgument(pvApiCtx);

		// Checking number of input and output arguments (enviromnet variable, min arguments, max arguments) 
		CheckInputArgument(pvApiCtx, 10, 12);
//		CheckOutputArgument(pvApiCtx, 0, 0);
	
		
		// to get the argument #1
		sciErr = getVarAddressFromPosition(pvApiCtx, 1, &piAddr1);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
		intErr1 = getScalarDouble(pvApiCtx, piAddr1, &width);
		if(intErr1)
		{
			Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 1);
			return -1;
		}

		// to get the argument #2
		sciErr = getVarAddressFromPosition(pvApiCtx, 2, &piAddr2);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
		intErr2 = getScalarDouble(pvApiCtx, piAddr2, &height);
		if(intErr2)
		{
			Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 2);
			return -1;
		}

		// to get the argument #3
		sciErr = getVarAddressFromPosition(pvApiCtx, 3, &piAddr3);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
		intErr3 = getScalarBoolean(pvApiCtx, piAddr3, &dense);
		if(intErr3)
		{
			Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 3);
			return -1;
		}

		// to get the argument #4
                sciErr = getVarAddressFromPosition(pvApiCtx, 4, &piAddr4);
                if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
		/* Check that the first input argument is a real matrix (and not complex) */
                if( !isDoubleType(pvApiCtx, piAddr4) ||  isVarComplex(pvApiCtx, piAddr4) )
                {
                        Scierror(999, "point cloud should consist of real values.");
                        return 0;
                }
		sciErr = getMatrixOfDouble(pvApiCtx, piAddr4, &rows1, &cols1, &location);
                if(sciErr.iErr)
                {
                        printError(&sciErr, 0);
                        return 0;
                }
                if(!(rows1==(width*height) && cols1==3))
                {
                    	Scierror(999, "Size error in point-cloud points");
                }

		// to get the argument #5
		sciErr = getVarAddressFromPosition(pvApiCtx, 5, &piAddr5);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
		intErr5 = getScalarDouble(pvApiCtx, piAddr5, &count);
		if(intErr5)
		{
			Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 5);
			return -1;
		}
\
		// to get the argument #6
                sciErr = getVarAddressFromPosition(pvApiCtx, 6, &piAddr6);
                if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
		sciErr = getMatrixOfDouble(pvApiCtx, piAddr6, &rows2, &cols2, &rgb);
                if(sciErr.iErr)
                {
                        Scierror(999, "gateway crashed abruptly while reading input argument #%d.", 6);
			return -1;
                }

		// to get the argument #7
                sciErr = getVarAddressFromPosition(pvApiCtx, 7, &piAddr7);
                if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
		sciErr = getMatrixOfDouble(pvApiCtx, piAddr7, &rows3, &cols3, &xlimit);
                if(sciErr.iErr)
                {
                        printError(&sciErr, 0);
                        return 0;
                }

		// to get the argument #8
                sciErr = getVarAddressFromPosition(pvApiCtx, 8, &piAddr8);
                if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
		sciErr = getMatrixOfDouble(pvApiCtx, piAddr8, &rows4, &cols4, &ylimit);
                if(sciErr.iErr)
                {
                        printError(&sciErr, 0);
                        return 0;
                }

		// to get the argument #9
                sciErr = getVarAddressFromPosition(pvApiCtx, 9, &piAddr9);
                if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
		sciErr = getMatrixOfDouble(pvApiCtx, piAddr9, &rows5, &cols5, &zlimit);
                if(sciErr.iErr)
                {
                        printError(&sciErr, 0);
                        return 0;
                }

		// to get the argument #10
		sciErr = getVarAddressFromPosition(pvApiCtx, 10, &piAddr10);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
		// Check for Argument type
		if( !isStringType(pvApiCtx, piAddr10))
		{
			Scierror(999, "%s: Wrong type of argument #%d. A string is expected.\n");
			return 0;
		}
		// Matrix of Stings
		sciErr = getMatrixOfString(pvApiCtx, piAddr10, &rows6, &cols6, NULL, NULL);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
		pilen1 = (int*)malloc(sizeof(int) * rows6 * cols6);
		// second call to retrieve the length of the string
		sciErr = getMatrixOfString(pvApiCtx, piAddr10, &rows6, &cols6, pilen1, NULL);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}
		filename = (char**)malloc(sizeof(char*) * rows6 * cols6);
		filename[0] = (char*)malloc(sizeof(char) * (*pilen1 + 1));
		// third call to retrieve data
		sciErr = getMatrixOfString(pvApiCtx, piAddr10, &rows6, &cols6, pilen1, filename);
		if(sciErr.iErr)
		{
			printError(&sciErr, 0);
			return 0;
		}

		string name(filename[0]);

		if(n == 10)
		{

			// creating PointCloud object
			pcl::PointCloud<pcl::PointXYZRGB> cloud;

			cloud.width    = width;
	  		cloud.height   = height;
	  		cloud.is_dense = dense;
	  		cloud.points.resize (cloud.width * cloud.height);

			// size of filename string
			s=name.length();

			// checking if the filename suggests .ply or .pcd format
			if(name.substr(s-3,s-1)=="ply")
			{
		                    sciprint("%d\n",*getNbInputArgument(pvApiCtx));
				for(size_t i = 0; i < cloud.points.size (); ++i)
	  			{
	    				cloud.points[i].x = location[i];
	    				cloud.points[i].y = location[i + (int)width*(int)height ];
	    				cloud.points[i].z = location[i + 2*(int)width*(int)height];


					cloud.points[i].r = rgb[i];
	    				cloud.points[i].g = rgb[i + (int)width*(int)height ];
	    				cloud.points[i].b = rgb[i + 2*(int)width*(int)height];
	  			}
	
				// creates ASCII encoded .ply file as default
				pcl::io::savePLYFileASCII(name, cloud);
				
			}
			else if(name.substr(s-3,s-1)=="pcd")
			{
			
				for(size_t i = 0; i < cloud.points.size (); ++i)
	  			{
	    				cloud.points[i].x = location[i];
	    				cloud.points[i].y = location[i + (int)width*(int)height ];
	    				cloud.points[i].z = location[i + 2*(int)width*(int)height];

					cloud.points[i].r = rgb[i];
	    				cloud.points[i].g = rgb[i + (int)width*(int)height ];
	    				cloud.points[i].b = rgb[i + 2*(int)width*(int)height];
	  			}
	
				// creates ASCII encoded .pcd file as default
				pcl::io::savePCDFileASCII(name, cloud);
				
			}
			else
			{
				Scierror(999, "wrong value argument #4 passed.");
				return 0;
			}	
			
		}
		else if(n == 12)
		{
			// to get the argument #11
			sciErr = getVarAddressFromPosition(pvApiCtx, 11, &piAddr11);
			if(sciErr.iErr)
			{
				printError(&sciErr, 0);
				return 0;
			}
			// Check for Argument type
			if( !isStringType(pvApiCtx, piAddr11))
			{
				Scierror(999, "%s: Wrong type of argument #%d. A string is expected.\n");
				return 0;
			}
			// Matrix of Stings
			sciErr = getMatrixOfString(pvApiCtx, piAddr11, &rows7, &cols7, NULL, NULL);
			if(sciErr.iErr)
			{
				printError(&sciErr, 0);
				return 0;
			}
			pilen2 = (int*)malloc(sizeof(int) * rows7 * cols7);
			// second call to retrieve the length of the string
			sciErr = getMatrixOfString(pvApiCtx, piAddr11, &rows7, &cols7, pilen2, NULL);
			if(sciErr.iErr)
			{
				printError(&sciErr, 0);
				return 0;
			}
			fileFormat = (char**)malloc(sizeof(char*) * rows7 * cols7);
			fileFormat[0] = (char*)malloc(sizeof(char) * (*pilen2 + 1));	
			// third call to retrieve data
			sciErr = getMatrixOfString(pvApiCtx, piAddr11, &rows7, &cols7, pilen2, fileFormat);
			if(sciErr.iErr)
			{
				printError(&sciErr, 0);
				free(pilen2);
				free(fileFormat);
					return 0;
			}
	
			// to get the argument #12
			sciErr = getVarAddressFromPosition(pvApiCtx, 12, &piAddr12);
			if(sciErr.iErr)
			{
				printError(&sciErr, 0);
				return 0;
			}
			// Check for Argument type
			if( !isStringType(pvApiCtx, piAddr12))
			{
				Scierror(999, "%s: Wrong type of argument #%d. A string is expected.\n");
				return 0;
			}
			// Matrix of Stings
			sciErr = getMatrixOfString(pvApiCtx, piAddr12, &rows8, &cols8, NULL, NULL);
			if(sciErr.iErr)
			{
				printError(&sciErr, 0);
				return 0;
			}
			pilen3 = (int*)malloc(sizeof(int) * rows8 * cols8);
			// second call to retrieve the length of the string
			sciErr = getMatrixOfString(pvApiCtx, piAddr12, &rows8, &cols8, pilen3, NULL);
			if(sciErr.iErr)
			{
				printError(&sciErr, 0);
				return 0;
			}
			fileType = (char**)malloc(sizeof(char*) * rows8 * cols8);
			fileType[0] = (char*)malloc(sizeof(char) * (*pilen3 + 1));	
			// third call to retrieve data
			sciErr = getMatrixOfString(pvApiCtx, piAddr12, &rows8, &cols8, pilen3, fileType);
			if(sciErr.iErr)
			{
				printError(&sciErr, 0);
				free(pilen3);
				free(fileType);
				return 0;
			}

			// creating PointCloud object
			pcl::PointCloud<pcl::PointXYZRGB> cloud;

			cloud.width    = width;
	  		cloud.height   = height;
	  		cloud.is_dense = dense;
	  		cloud.points.resize (cloud.width * cloud.height);

			// concatenating 
			strcat(filename[0], ".");
			strcat(filename[0], fileFormat[0]);

			if(strcmp(fileFormat[0],"ply") == 0)
			{
		
				for(size_t i = 0; i < cloud.points.size (); ++i)
	  			{
	    				cloud.points[i].x = location[i];
	    				cloud.points[i].y = location[i + (int)width*(int)height ];
	    				cloud.points[i].z = location[i + 2*(int)width*(int)height];

					cloud.points[i].r = rgb[i];
	    				cloud.points[i].g = rgb[i + (int)width*(int)height ];
	    				cloud.points[i].b = rgb[i + 2*(int)width*(int)height];
					
	  			}
	
				if(strcmp(fileType[0],"ASCII") == 0)
				{
					pcl::io::savePLYFileASCII(filename[0], cloud);
				}
				else if(strcmp(fileType[0],"Binary") == 0)
				{
					pcl::io::savePLYFileBinary(filename[0], cloud);
				}
				else
				{
					Scierror(999, "wrong value argument #4 passed.");
					return 0;
				}
			}
			else if(strcmp(fileFormat[0],"pcd") == 0)
			{
			
				for(size_t i = 0; i < cloud.points.size (); ++i)
	  			{
	    				cloud.points[i].x = location[i];
	    				cloud.points[i].y = location[i + (int)width*(int)height ];
	    				cloud.points[i].z = location[i + 2*(int)width*(int)height];

					cloud.points[i].r = rgb[i];
	    				cloud.points[i].g = rgb[i + (int)width*(int)height ];
	    				cloud.points[i].b = rgb[i + 2*(int)width*(int)height];
	  			}
	
				if(strcmp(fileType[0],"ASCII") == 0)
				{
					pcl::io::savePCDFileASCII(filename[0], cloud);
				}
				else if(strcmp(fileType[0],"Binary") == 0)
				{
					pcl::io::savePCDFileBinary(filename[0], cloud);
				}
				else if(strcmp(fileType[0],"Compressed") == 0)
				{
					pcl::io::savePCDFileBinaryCompressed(filename[0], cloud);
				}
				else
				{
					Scierror(999, "wrong value argument #4 passed.");
					return 0;
				}	
			}
			else
			{
				Scierror(999, "wrong value argument #3 passed.");
				return 0;
			}	
			
		}
			
		
	}
}
