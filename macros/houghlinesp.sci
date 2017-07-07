// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Gursimar Singh
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [lines]=houghlinesp(image, rho, theta, threshold, minLineLength, maxLineGap)
//Extract line segments based on  Probablistic Hough transform.
//
//Calling Sequence
//lines = houghlinesp(image, rho, theta, threshold, minLineLength, maxLineGap)	
//
//Parameters
//image:Input image must be gray scale.
//lines:Output vector of lines. A NX4 matrix ([x1,y1,x2,y2]) where (x1,y1) and (x2,y2) are the ending points of each detected line segment.
//rho:Distance resolution of the accumulator in pixels. default 1.
//theta:Angle resolution of the accumulator in radians. default pi/180.
//threshold:Accumulator threshold parameter. Only those lines are returned that get enough votes (>Threshold).
//MinLineLength:Minimum line length. Line segments shorter than that are rejected.
//MaxLineGap:Maximum allowed gap between points on the same line to link them.
//
//Description
//The function implements the probabilistic Hough transform algorithm for line detection.It is a more efficient implementation of the Hough Line Transform. It gives as output the extremes of the detected lines.
// 
//Examples
//im=imread("images/check.jpg");
//img=canny(im,50,100,3,1);
//l=houghlinesp(img,1,%pi/180,70,40,15);
//sz=size(l);
//sz=sz(1);
//figure("Figure_Name","Original Image")
//imshow(im);
//for i=1:sz
//  im=line(im,l(i,1),l(i,2),l(i,3),l(i,4),0,0,255,2,8);
//end
//figure("Figure_Name","Detected lines")
//imshow(im);
//
//Authors
//Gursimar Singh
//
//See also
//houghlinesp
//line

[lhs rhs] = argn(0);
     if rhs>6 then
         error(msprintf("Too many input arguments"));
     end
     if rhs<6 then
	 error(msprintf("Not enough input arguments"));
	 end
	 if lhs >1
	 error(msprintf("Too many output arguments"));
     end

     if minLineLength<0 
     	error(msprintf("srn must be positive"));
     end
     if maxLineGap<0 
     	error(msprintf("stn must be positive"));
     end
     if threshold<0 
     	error(msprintf("threshold must be positive"));
     end

	image_list = mattolist(image);
     sz=size(image_list);
     if sz >=3 then
        error(msprintf("Input image must be grayscale"));
     end      

	lines = raw_houghlinesp(image_list, rho, theta, threshold, minLineLength, maxLineGap);

endfunction
