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
function output_image = edgePreservingFilter(src,varargin)
// This function smoothens the given input image while preserving the edges.
//
// Calling Sequence
// src = imread("image-location-for-src");
// outputImage = edgePreservingFilter(src,flag,sigma_s,sigma_r)
//
//Parameters
//outputImage: resultant Output image 
//src: Input 8-bit 3-channel image.
//flag: (default value is 1) (denotes Edge preserving filters). It takes 2 values ->  1) 1 (for RECURS_FILTER) ; 2) 2 (for NORMCONV_FILTER)
//sigma_s: Range between 0 to 200 (float) (default value is 60 )
//sigma_r: Range between 0 to 1. (float) (default value is 0.4f)
//
//Description
//Filtering is the fundamental operation in image and video processing. Edge-preserving smoothing filters 
//are used in many different applications 
//
// Examples
// 
// src = imread("/images/color2.jpeg");
// outputImage1 = edgePreservingFilter(src) //using default values for all optional arguments
// imshow(outputImage1);//view the output
// 
// outputImage2 = edgePreservingFilter(src,2) //using default values for sigma_s & sigma_r
// imshow(outputImage2);//view the output
// 
// outputImage3 = edgePreservingFilter(src,[],[],0.6) //using default values for sigma_s & sigma_r
// imshow(outputImage3);//view the output
//
// Authors
//  Rohan Gurve
	
	[lhs rhs]=argn(0);
    if rhs>4 //max i/p arguments are 4
        error(msprintf(" Too many input arguments"));
    elseif rhs<1//min i/p argument is 1
        error(msprintf("input arguments missing"));
    end
    
    if lhs>1
	    error(msprintf("Too many output arguments"));
	end
	
	image_list1 = mattolist(src);
	flag = argindefault  ( varargin , 1 , 1 ); //default value is 1
	sigma_s = argindefault ( varargin , 2 , 60 );//default value is 60
    sigma_r = argindefault ( varargin , 3 , 0.4 );//default value is 0.4
	
	
	out = raw_edgePreservingFilter(image_list1,flag,sigma_s,sigma_r);
	
	sz = size(out);
	
	for i=1:sz
		output_image(:, :, i) = out(i)
	end
	
endfunction


