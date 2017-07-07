// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Gursimar Singh,Shubheksha Jalan
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [lines]=houghlines(image,rho, theta, threshold, srn, stn)
//Extract line segments based on Standard Hough transform
//
//Calling Sequence
//out=houghlines(image,rho,theta, threshold, srn, stn)
//
//Parameters
//image:Input image.Must be in grayscale.
//lines: Output matrix of lines. A 2 coloumn matrix ([rho,theta]) rho is the distance from the coordinate origin (0,0) (top-left corner of the image). theta is the line rotation angle in radians (0 ~ vertical line, pi/2 ~ horizontal line).
//rho:Distance resolution of the accumulator in pixels. default 1.
//theta:Angle resolution of the accumulator in radians. default pi/180.
//threshold:Accumulator threshold parameter. Only those lines are returned that get enough votes (>Threshold).
//srn:For the multi-scale Hough transform, it is a divisor for the distance resolution Rho. The coarse accumulator distance resolution is Rho and the accurate accumulator resolution is Rho/SRN. If both SRN=0 and STN=0, the classical Hough transform is used. Otherwise, both these parameters should be positive.
//stn:For the multi-scale Hough transform, it is a divisor for the distance resolution Theta.
//
//Description
//The function implements the standard or standard multi-scale Hough transform algorithm for line detection. See homepages.inf.ed.ac.uk/rbf/HIPR2/hough.htm for a good explanation of Hough transform.
//
//Examples
//im=imread("images/check.jpg");
//img=canny(im,50,100,3,1);
//lines=houghlines(img,1,%pi/180,120,0,0);
//sz=size(lines);
//sz=sz(1);
//for i=1:sz 
//      rho = lines(i,1);
//      theta = lines(i,2);
//      a = cos(theta);
//      b = sin(theta);
//      x0 = a*rho; 
//      y0 = b*rho;
//      x1 = round(x0 + 1000*(-b));
//      y1 = round(y0 + 1000*(a));
//      x2 = round(x0 - 1000*(-b));
//      y2 = round(y0 - 1000*(a));
//      line(im,x1,y1,x2,y2,0,0,255,2,8,0);
//    end
//imshow(im);
//
//Authors
//Gursimar Singh
//Shubheksha Jalan
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

     if srn<0 
     	error(msprintf("srn must be positive"));
     end
     if stn<0 
     	error(msprintf("stn must be positive"));
     end
     if threshold<0 
     	error(msprintf("threshold must be positive"));
     end
    image1=mattolist(image);
    sz=size(image1);
    if sz >=3 then
        error(msprintf("Input image must be grayscale"));
     end  
         lines=raw_houghlines(image1,rho, theta, threshold, srn, stn);
endfunction;
