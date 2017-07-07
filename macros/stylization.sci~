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
function output_image = stylization(src,varargin)
//This function aims to produce non-photorealistic digital imagery with a wide variety of effects.
//
//Calling Sequence
//src = imread("image-location-for-src");
//outputImage = stylization(src,sigma_s,sigma_r)
//
//Parameters
//outputImage: resultant Output image 
//src: Input 8-bit 3-channel image.
//sigma_s: Range between 0 to 200 (float) (default value is 60)
//sigma_r: Range between 0 to 1. (float)  (default value is 0.45f)
//
//Description
// Edge-aware filters are ideal for stylization, as they can abstract regions of low contrast while preserving, 
// or enhancing, high-contrast features.
//
// Examples
// src = imread("/images/color2.jpeg");
// outputImage1 = stylization(src)  //using default values for optional parameters 
// imshow(outputImage1);//view the output
//
// outputImage2 = stylization(src,121) //using default value for sigma_r
// imshow(outputImage2);//view the output
//
// outputImage3 = stylization(src,[],0.2) //using default value for sigma_s
// imshow(outputImage3);//view the output
//
// Authors
// Rohan Gurve
	
	[lhs rhs]=argn(0);
    if rhs>3 //max i/p arguments are 3
        error(msprintf(" Too many input arguments"));
    elseif rhs<1 //min i/p argument is 1
        error(msprintf("input arguments missing"));
    end
    
    if lhs>1
	    error(msprintf("Too many output arguments"));
	end
	
	image_list1 = mattolist(src);
	sigma_s = argindefault ( varargin , 1 , 60 );//default value is 60
    sigma_r = argindefault ( varargin , 2 , 0.45 );//default value is 0.45f
	
	
	
	out = raw_stylization(image_list1,sigma_s,sigma_r);
	
	
	
	sz = size(out);
	
	for i=1:sz
		output_image(:, :, i) = out(i)
	end
	
endfunction


