// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Gursimar Singh,Sukul Bagai, Abhilasha Sancheti
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function circles= houghcircles(inputImage , dp , mindist , param1, param2 ,minradius, maxradius)
//Finds circles in a grayscale image using the Hough transform
//
//Calling Sequence
//circles= houghcircles(inputImage , dp , mindist , param1, param2 ,minradius, maxradius);
//
//Parameters
//circles: Output matrix of found circles. A NX3 matrix of the form [x, y, radius],where N represents the number of circles found.
//inputImage:Grayscale input image.
//dp:Inverse ratio of the accumulator resolution to the image resolution. For example, if dp=1, the accumulator has the same resolution as the input image. If dp=2, the accumulator has half as big width and height.
//mindist: Minimum distance between the centers of the detected circles. If the parameter is too small, multiple neighbor circles may be falsely detected in addition to a true one. If it is too large, some circles may be missed.Generally size(image,1)/8.
//param1:First method-specific parameter.It is the higher threshold of the two passed to the cv.Canny edge detector (the lower one is twice smaller). default 100.
//param2:Second method-specific parameter.It is the accumulator threshold for the circle centers at the detection stage. The smaller it is, the more false circles may be detected. Circles, corresponding to the larger accumulator values, will be returned first. default 100.
//minradius:Minimum circle radius. default 0.
//maxradius:Maximum circle radius. default 0.
//
//Description
//The function finds circles in a grayscale image using a modification of the Hough transform. Usually the function detects the centers of circles well. However, it may fail to find correct radii. You can assist to the function by specifying the radius range (MinRadius and MaxRadius) if you know it. Or, you may ignore the returned radius, use only the center, and find the correct radius using an additional procedure.
//
//Examples
//im=imread("images/blob.jpg");
//img=cvtColor(im,CV_BGR2GRAY);
//mindist=size(im,1)/16;
//circles=houghcircles(img,1,mindist,100,50,0,30);
//for i=1:size(circles,1)
//	im=circle(im,circle(i,1),circle(i,2),3,0,0,255,-1,8,0); ///mark centers
//	im=circle(im,circle(i,1),circle(i,2),circle(i,3),0,0,255,2,8,0);//draw circles
//end
//imshow(im);	
//
//Authors
//Sukul Bagai
//Abhilasha Sancheti
//Gursimar Singh
//
//See also
//Canny
//houghlinesp
//houghlines

[lhs rhs] = argn(0);
     if rhs>7 then
         error(msprintf("Too many input arguments"));
     end
     if rhs<7 then
	 error(msprintf("Not enough input arguments"));
	 end
	 if lhs >1
	 error(msprintf("Too many output arguments"));
     end

     if minradius<0 
     	error(msprintf("minradius must be positive"));
     end
     if maxradius<0 
     	error(msprintf("maxradius must be positive"));
     end
     if dp<0 | param1<0 | param2<0 then 
     	error(msprintf("INput arguments must be positive"));
     end

	inputList=mattolist(inputImage);
	sz=size(inputList);
     if sz >=3 then
        error(msprintf("Input image must be grayscale"));
     end 
    circles=raw_houghcircles(inputList , dp , mindist , param1, param2 ,minradius, maxradius)
   
endfunction
