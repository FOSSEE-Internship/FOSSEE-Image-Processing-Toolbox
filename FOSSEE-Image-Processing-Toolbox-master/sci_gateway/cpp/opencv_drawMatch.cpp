#include <numeric>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <iostream>
//#include "opencv2/imgcodecs.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/highgui/highgui.hpp"


using namespace cv;
using namespace std;
using namespace cv::xfeatures2d;
extern "C"
{
  #include "api_scilab.h"
  #include "Scierror.h"
  #include "BOOL.h"
  #include <localization.h>
  #include "sciprint.h"
  #include "../common.h"

  int opencv_drawMatch(char *fname, unsigned long fname_len)
  {			
  		//return 0;	
		SciErr sciErr;
    	double *result=NULL;
		double *color =NULL;
		double *indexPairs =NULL;
		double *distances =NULL;
		double *keyPoints1 =NULL;
		double *keyPoints2 =NULL;
		int iRows=0;
		int indRows;
		int iCols=0;
		int intErr = 0;
		int nbInputArguments = 0;
		int *piLen = NULL;
		int *piAddr = NULL;
		bool *providedArgs = NULL; 
		char *currentArg = NULL; 
		char **pstData  = NULL;
		char flag;
		
    	//checking input argument
   	CheckInputArgument(pvApiCtx, 6, 10);
        CheckOutputArgument(pvApiCtx, 1, 1);
        Mat image1;
        retrieveImage(image1, 1);
     	Mat image2;
    	retrieveImage(image2, 2);
		nbInputArguments = *getNbInputArgument(pvApiCtx);

		//sciprint("getting input for loc 50\n");
		if((nbInputArguments)%2!=0)
		{
			Scierror(999, "Incorrect number of arguments provided. Please check the documentation for more information.\n"); 
			return 0; 
		}
                //sciprint("getting input for loc 51\n");
		providedArgs = (bool*)malloc(sizeof(bool) * 2);
		for(int i=0;i<4;i++)
			providedArgs[i] = 0;
                //sciprint("getting input for loc 52\n");
	        sciErr = getVarAddressFromPosition(pvApiCtx,3, &piAddr); 
		if (sciErr.iErr)
		{
			printError(&sciErr, 0); 
			return 0; 
		}
		//sciprint("getting input for loc 53\n");
		sciErr = getMatrixOfDouble(pvApiCtx, piAddr,&iRows,&iCols,&keyPoints1); 
		if (sciErr.iErr)
		{
			printError(&sciErr, 0); 
			return 0; 
		}
		vector <KeyPoint> keypoints1(iRows*2) ;
		
		//sciprint("getting adrress keyPoints1\n");
		for (int i =0 ; i<iRows; i++)
		{
			keypoints1[i].pt.x=float(keyPoints1[i]);
			keypoints1[i].pt.y=float(keyPoints1[i + iRows]);


		}
		
		//sciprint("keyponits rows,cols : %d %d\n",iRows,iCols);
                //////////////////////second keypoints//////////////////////
                //sciprint("getting input for loc 54\n");
                sciErr = getVarAddressFromPosition(pvApiCtx,4, &piAddr); 
		if (sciErr.iErr)
		{
			printError(&sciErr, 0); 
			return 0; 
		}

		sciErr = getMatrixOfDouble(pvApiCtx, piAddr,&iRows,&iCols,&keyPoints2); 
		if (sciErr.iErr)
		{
			printError(&sciErr, 0); 
			return 0; 
		}
		//sciprint("getting input for loc 55\n");
		vector <KeyPoint> keypoints2(iRows*2) ;
		for (int i =0 ; i<iRows; i++)
		{
			keypoints2[i].pt.x=float(keyPoints2[i]);
			keypoints2[i].pt.y=float(keyPoints2[i + iRows]);


		}
		//sciprint("getting adrress for loc 5\n");
		//sciprint("keyponits rows,cols : %d %d\n",keyPoints2.rows,keyPoints2.cols);

                ///////////////////////////index pairs////////////
                sciErr = getVarAddressFromPosition(pvApiCtx,5, &piAddr); 
		if (sciErr.iErr)
		{
			printError(&sciErr, 0); 
			return 0; 
		}
		//sciprint("getting values indexPairs\n");

		sciErr = getMatrixOfDouble(pvApiCtx, piAddr,&iRows,&iCols,&indexPairs); 
		if (sciErr.iErr)
		{
			printError(&sciErr, 0); 
			return 0; 
		}
		indRows=iRows;
		//sciprint("getting double index \n");

		//sciprint("index pairs rows,cols : %d %d\n",indexPairs.rows,indexPairs.cols);
                ////////////////////////////////// distances///////////////////////
                sciErr = getVarAddressFromPosition(pvApiCtx,6, &piAddr); 
		if (sciErr.iErr)
		{
			printError(&sciErr, 0); 
			return 0; 
		}
		//sciprint("getting adrress for distance\n");

		sciErr = getMatrixOfDouble(pvApiCtx, piAddr,&iRows,&iCols,&distances); 
		if (sciErr.iErr)
		{
			printError(&sciErr, 0); 
			return 0; 
		}
		
		//sciprint("getting doubel distances\n");

		//sciprint("distances rows,cols : %d %d\n",distances.rows,distances.cols);
                //////////////////////////////////////////////////////////////////

	        for(int iter=7;iter<=nbInputArguments;iter++)
		{
			// Getting address of next argument
			sciErr = getVarAddressFromPosition(pvApiCtx, iter, &piAddr); 
			if (sciErr.iErr)
			{
				printError(&sciErr, 0); 
				return 0; 
			}

			// Extracting name of next argument takes three calls to getMatrixOfString
			sciErr = getMatrixOfString(pvApiCtx, piAddr, &iRows, &iCols, NULL, NULL); 
			if (sciErr.iErr)
			{
				printError(&sciErr, 0); 
				return 0; 
			}

			piLen = (int*) malloc(sizeof(int) * iRows * iCols); 

			sciErr = getMatrixOfString(pvApiCtx,  piAddr,  &iRows,  &iCols,  piLen,  NULL); 
			if (sciErr.iErr)
			{
				printError(&sciErr, 0); 
				return 0; 
			}

			pstData = (char**) malloc(sizeof(char*) * iRows * iCols); 
			for(int iterPstData = 0; iterPstData < iRows * iCols; iterPstData++)
			{
				pstData[iterPstData] = (char*) malloc(sizeof(char) * piLen[iterPstData] + 1); 
			}

			sciErr = getMatrixOfString(pvApiCtx, piAddr, &iRows, &iCols, piLen, pstData); 
			if (sciErr.iErr)
			{
				printError(&sciErr, 0); 
				return 0; 
			}

			currentArg = pstData[0];
			// providedArgs[] makes sure that no optional argument is provided more than once

			if(strcmp(currentArg, "matchcolor")==0)
			{
				if(!providedArgs[0])
				{
					sciErr = getVarAddressFromPosition(pvApiCtx, ++iter, &piAddr); 
					if (sciErr.iErr)
					{
						printError(&sciErr, 0); 
						return 0; 
					}

					sciErr = getMatrixOfDouble(pvApiCtx, piAddr,&iRows,&iCols,&color); 
					if (sciErr.iErr)
					{
					printError(&sciErr, 0); 
					return 0; 
					}
					// Checking if values are in proper range. Same for all optional arguments
					if (iCols != 3 || iRows != 1)
					{
					
						Scierror(999, "Error: Please provide proper value for \"%s\". input should be of the form [b,g,r].\n", currentArg); 
						return 0; 

					}
					for (int i=0;i<3;i++)
					{
						if (color[i]>255 || color[i]<0)
						{
						Scierror(999, "Error: Please provide proper value for \"%s\". permisible range (0,255).\n", currentArg); 
						return 0; 
	
						}
					}
					providedArgs[0] = 1;

				}
				else if(providedArgs[0])
				{
					Scierror(999, "Please provide optional arguments only once.\n"); 
					return 0; 
				}
			}
			else if(strcmp(currentArg, "flags")==0)
			{
				if(!providedArgs[1])
				{
					sciErr = getVarAddressFromPosition(pvApiCtx, ++iter, &piAddr); 
					if (sciErr.iErr)
					{
						printError(&sciErr, 0); 
						return 0; 
					}

					intErr = getScalarInteger8(pvApiCtx, piAddr, &flag); 
					if(intErr)
					{
						return intErr;  
					}
					// Checking if values are in proper range. Same for all optional arguments
					if(flag !=0 && flag != 1 && flag != 2 && flag != 4)
					{
						Scierror(999, "Error: Please provide proper value for \"%s\". Permissible values are 0,1,2,4.\n", currentArg); 
						return 0; 
					}
					providedArgs[1] = 1;
				}
				else if(providedArgs[1])
				{
					Scierror(999, "Please provide optional arguments only once.\n"); 
					return 0; 
				}
			}
		
		        }	

    	               try
    	               {
                             //Ptr<SURF> surf = SURF::create(5000);
                             //vector<KeyPoint> keypoints1, keypoints2;
                             //Mat descriptors1, descriptors2;
                             //Mat img1,img2;
    		            image1.convertTo(image1,CV_8UC1);
     		            image2.convertTo(image2,CV_8UC1);
     		            //sciprint("converting image \n");

		            // surf->detectAndCompute(img1, Mat(), keypoints1, descriptors1);
                            // surf->detectAndCompute(img2, Mat(), keypoints2, descriptors2);
                            // BFMatcher matcher(surf->defaultNorm());
    		
    		              vector<DMatch> matches;
       		            // double max_dist = 0; double min_dist = 100;
    		            // for(int i=0;i<indRows;i++)
    		            // {
    		            // double dist = matches[i].distance;
	    	            // 	if( dist < min_dist ) min_dist = dist;
    		            // 	if( dist > max_dist ) max_dist = dist;
    		            // }
    			    //sciprint("getting min distance\n");
                            //sciprint("min_dist\n");
    		      for (int i=0;i<indRows;i++)
    		      {
	    		   DMatch temp;
	    		   temp.queryIdx=indexPairs[i];
	    		   temp.distance=distances[i];
	    		   temp.trainIdx=indexPairs[indRows+i];
	    		
	    		    // sciprint("temp distance %d\n",temp.distance);
	    		    // if( temp.distance <= max(2*min_dist, 0.02) )
    			    // 	matches.push_back(temp); 
	    		   matches.push_back(temp);

    		      }
    		            //sciprint("matches\n");
    		            //std::vector< DMatch > good_matches;
                            // for( int i = 0; i < descriptors_1.rows; i++ )
  			    // { 
  			    // 	if( matches[i].distance <= max(2*min_dist, 0.02) )
    		            //good_matches.push_back( matches[i]); 
  			    // }
    		            //matcher.match(descriptors1, descriptors2, matches);
    		            Mat img_matches;
    		            //int b=color[0];
    		            //int g=color[1];
    		            //int r=color[2];
    		           drawMatches(image1, keypoints1, image2, keypoints2, matches, img_matches,Scalar::all(-1),Scalar::all(-1),vector<char>(),int(flag));
			   string tempstring = type2str(img_matches.type());
			   char *checker;
			   checker = (char *)malloc(tempstring.size() + 1);
			   memcpy(checker, tempstring.c_str(), tempstring.size() + 1);
			   returnImage(checker, img_matches, 1);
			   // imshow("opencv",img_matches);
			   // waitKey(0);
			   free(checker);
			
	              }
	              catch(Exception& e)
		      {
			    const char* err=e.what();
			    sciprint("%s",err);
	      	      }
		           AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
			   ReturnArguments(pvApiCtx);	
			   return 0;
	}

		
	
}	

	



