// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Rohan Gurve 
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function depth = getDepth(src)
// This function returns the depth of the input matrix element.  
//
//  Calling Sequence
//  src = imread("image-location-for-src");
//  depth = getDepth(src)
//
//Parameters
//src: Input 8-bit 3-channel image.
//depth: a string which denoted the depth of the src.It identifies the following- 1) CV_8U - 8-bit unsigned integers ( 0..255 ) ; 2) CV_8S - 8-bit signed integers ( -128..127 ); 3) CV_16U - 16-bit unsigned integers ( 0..65535 ); 4) CV_16U - 16-bit unsigned integers ( 0..65535 ); 5) CV_16S - 16-bit signed integers ( -32768..32767 ) ; 6) CV_32S - 32-bit signed integers ( -2147483648..2147483647 ) ; 7) CV_64F - 64-bit floating-point numbers ( -DBL_MAX..DBL_MAX, INF, NAN )
//
//
// Examples
// 
// src = imread("../images/color2.jpeg"); //reading an image
// depth = getDepth(src) ; //get the depth
// disp(depth) ; //view the output
//
// Description
// Note - Scilab does not support CV_32F - it would be considered as CV_64F
//
// Authors
//  Rohan Gurve
	
	[lhs rhs]=argn(0);
    if rhs>1 //max i/p arguments is 1
        error(msprintf(" Too many input arguments"));
    elseif rhs<1 //min i/p argument is 1
        error(msprintf("input arguments missing"));
    end
    
    if lhs>1
	    error(msprintf("Too many output arguments"));
	end
	
	image_list1 = mattolist(src);
	
	d = raw_getDepth(image_list1);	
	depth = d(1);
	
endfunction


