// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Shreyash Sharma
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [out]=multithresh(image,thresh)
// This function is used to get multilevel image thresholds using Otsu's method. 
//
// Calling Sequence
// B = multithres(A,thresh)
//
// Parameters
// A: image matrix of the source image.
// B : Set of threshold values used to quantize an image, returned as a 1-by-N vector, whose data type is the same as image A.
// thresh : Number of threshold values, specified as a positive integer scalar value. 
//
// Description
// thresh = multithresh(A,thresh) returns thresh a 1-by-thresh vector containing N threshold values using Otsu's method..
//
// Examples
// i = imread("lena.jpeg",0);
// m = multithresh(i,67);
//
 
 [lhs rhs] = argn(0);
   
 image1=mattolist(image);
       
       if lhs > 1
		error(msprintf("Too many output arguments"));
       elseif rhs < 1
		error(msprintf("Not enough input arguments"));
       end
       
      
       
         out=raw_multithresh(image1,thresh);
         
endfunction;
