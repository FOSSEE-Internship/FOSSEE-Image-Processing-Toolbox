/********************************************************
Author: Nihar Rao 
*********************************************************
*/

#include <numeric>
#include "opencv2/core/core.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include <sys/stat.h>
#include "opencv2/videoio.hpp"
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

   double *readdoublvec_from_scilab(int cnt)
    {

        SciErr sciErr;
        int *piAddr=NULL;
        double *val=NULL;
        int iRows,iCols;

        sciErr = getVarAddressFromPosition(pvApiCtx,cnt,&piAddr);
        if (sciErr.iErr)
        {
          printError(&sciErr, 0);
          return 0;
        }
        sciErr = getMatrixOfDouble(pvApiCtx,piAddr,&iRows,&iCols,&val);
        if (sciErr.iErr)
        {
          printError(&sciErr, 0); 
          return 0; 
        }
        return val;
    }
 double readdoublec_from_scilab(int cnt)
    {

          SciErr sciErr;
          int *piAddr=NULL;
         double val=0;
         int  intErr;

        sciErr = getVarAddressFromPosition(pvApiCtx,cnt,&piAddr);
        if (sciErr.iErr)
        {
        printError(&sciErr, 0);
        return 0;
        }
         intErr = getScalarDouble(pvApiCtx, piAddr ,&val);

        if(isDoubleType(pvApiCtx, piAddr))
        return val;
        else
        {
          Scierror(999,"Error: the input argument is not of type double");
          return 0;
        }
    }
  inline bool file_exists_check(const std::string& name)
     {
        struct stat buffer;   
        return (stat (name.c_str(), &buffer) == 0); 
     }
  
  int opencv_CascadeObjectDetector(char *fname, unsigned long fname_len)
 {
    SciErr sciErr;
    int intErr = 0;
    int iRows=0,iCols=0;
    int *piAddr = NULL;
    int *piAddrNew = NULL;
    int *piAddr2  = NULL;
    int *piAddr3  = NULL;
    int *piLen = NULL;
    char **object = NULL;
    int i;
    double *bboxes = NULL;
    double *minSize=NULL;
    double *maxSize=NULL;
    Size minsz(40,40);
    Size maxsz(100,100);


    double scale=1.05;//all optional arguments are set to their default values
    int minNeighbors=2,flags=2;
    
    CheckInputArgument(pvApiCtx, 2, 7);
    CheckOutputArgument(pvApiCtx, 1, 2) ;
    Mat image,new_image,image_gray;
    //get no. of input arguments
    int n=*getNbInputArgument(pvApiCtx);
    retrieveImage(image, 1);

    ///resize(image,image,Size(800,600),0,0,cv::INTER_AREA);

    sciErr = getVarAddressFromPosition(pvApiCtx,2, &piAddr2);

    if (sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }

    sciErr = getMatrixOfString(pvApiCtx, piAddr2, &iRows, &iCols, NULL, NULL);
    if(sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }
    piLen = (int*)malloc(sizeof(int) * iRows * iCols);
    //second call to retrieve length of each string
    sciErr = getMatrixOfString(pvApiCtx, piAddr2, &iRows, &iCols, piLen, NULL);
    if(sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }
    object= (char**)malloc(sizeof(char*) * iRows * iCols);
    for(i = 0 ; i < iRows * iCols ; i++)
    {
        object[i] = (char*)malloc(sizeof(char) * (piLen[i] + 1));//+ 1 for null termination
    }
    //third call to retrieve data
    sciErr = getMatrixOfString(pvApiCtx, piAddr2, &iRows, &iCols, piLen,object);
    if(sciErr.iErr)
    {
        printError(&sciErr, 0);
        return 0;
    }
   ///Reading optional arguments
    int cnt=3;
        switch(n)
        {
          case 3:
           
          scale=readdoublec_from_scilab(cnt);
          cnt++;
          break;
          case 4:
           
          scale=readdoublec_from_scilab(cnt);
          cnt++;
          minNeighbors=(int)readdoublec_from_scilab(cnt);
          cnt++;
          break;
          case 5:
           
          scale=readdoublec_from_scilab(cnt);
          cnt++;
          minNeighbors=(int)readdoublec_from_scilab(cnt);
          cnt++;
          flags=(int)readdoublec_from_scilab(cnt);
          if(flags!=2 && flags!=1 && flags!=4 && flags!=8)
            {
               Scierror(999,"Error: the flag should be {1,2,4,8}\n");
                return 0;

            }


          cnt++;
          break;
          case 6:
           
          scale=readdoublec_from_scilab(cnt);
          cnt++;
          minNeighbors=(int)readdoublec_from_scilab(cnt);
          cnt++;
          flags=(int)readdoublec_from_scilab(cnt);
          cnt++;
          if(flags!=2 && flags!=1 && flags!=4 && flags!=8)
            {
               Scierror(999,"Error: the flag should be {1,2,4,8}\n");
                return 0;

            }

          minSize=readdoublvec_from_scilab(cnt);
          cnt++;
          minsz=Size(int(minSize[0]),int(minSize[1]));
          break;
          case 7:
           
          scale=readdoublec_from_scilab(cnt);
          cnt++;
          minNeighbors=(int)readdoublec_from_scilab(cnt);
          cnt++;
          flags=(int)readdoublec_from_scilab(cnt);
          cnt++;
          if(flags!=2 && flags!=1 && flags!=4 && flags!=8)
            {
               Scierror(999,"Error: the flag should be {1,2,4,8}\n");
                return 0;

            }

          minSize=readdoublvec_from_scilab(cnt);
          cnt++;
          minsz=Size(int(minSize[0]),int(minSize[1]));
          maxSize=readdoublvec_from_scilab(cnt);
          cnt++;
          maxsz=Size(int(maxSize[0]),int(maxSize[1]));
          break;

         }


    /* Actual Processing*/
    //Converting image from rgb to gray scale image.
    image.convertTo(image,CV_8U);
    cvtColor( image, image_gray, CV_BGR2GRAY );
    //equalizing the histrogram of gray scale image
    equalizeHist( image_gray, image_gray );

        vector<Rect> found;
        int total_boxes;
        
     
          int j;
          for(j=0;j<iRows * iCols;j++)
          {
            if(!file_exists_check(object[j]))
            {
       
              sciprint("Error: the input File -%s  not found!.Please enter correct path!\n",object[j]);
              return 0;

            }

            CascadeClassifier s;
            s.load(object[j]);
            found.clear();
            try
            {
            s.detectMultiScale( image_gray, found, scale, minNeighbors, 0|flags ,minsz,maxsz) ;
            }
             catch(cv::Exception& e)
            {
              const char* err=e.what();
              sciprint("\n%s\n",err);
            }
            
            bboxes = (double*)malloc(sizeof(double) * (int)found.size() * 4);

            total_boxes=found.size();
            for( int i = 0; i < found.size(); i++ )
            {    

               Point point1( found[i].x + found[i].width, found[i].y + found[i].height );
               Point point2( found[i].x, found[i].y);
               rectangle(image,found[i],cvScalar(0,0,0),2,16,0);
               
               Rect temp = found[i];

              // x coordinate
              bboxes[i + 0 * total_boxes] = temp.x;

              // y coordinate
              bboxes[i + 1 * total_boxes] = temp.y;

              // width
              bboxes[i + 2 * total_boxes] = temp.width;

              // height
              bboxes[i + 3 * total_boxes] = temp.height;

            }

          }
       
      
      
    //sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 1, tbox, 4, bboxes);

    string tempstring = type2str(image.type());
    char *checker;
    checker = (char *)malloc(tempstring.size() + 1);
    memcpy(checker, tempstring.c_str(), tempstring.size() + 1);
    returnImage(checker,image,1);
    free(checker);

    sciErr = createMatrixOfDouble(pvApiCtx, nbInputArgument(pvApiCtx) + 2, total_boxes, 4, bboxes);
    if(sciErr.iErr){
      printError(&sciErr, 0);
      return 0;
    }

    //Assigning the list as the Output Variable
    AssignOutputVariable(pvApiCtx, 1) = nbInputArgument(pvApiCtx) + 1;
    AssignOutputVariable(pvApiCtx, 2) = nbInputArgument(pvApiCtx) + 2;
    //Returning the Output Variables as arguments to the Scilab environment
    ReturnArguments(pvApiCtx); 
    return 0;

  }
  

  
}
