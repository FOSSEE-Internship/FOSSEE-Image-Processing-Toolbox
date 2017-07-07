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
function output_image = detailEnhance(src,varargin)
//This filter enhances the details of a the given image.
//
// Calling Sequence
// src = imread("image-location-for-src");
// outputImage = detailEnhance(src,sigma_s,sigma_r)
//
// Parameters
// outputImage: resultant Output image 
// src: Input 8-bit 3-channel image.
// sigma_s: Range between 0 to 200 (float) (default value is 10)
// sigma_r: Range between 0 to 1. (float) (default value is 0.15f)
//
// Examples
// 
// src = imread("/images/color2.jpeg");
// outputImage1 = detailEnhance(src) //using the default values for optional parameter 
// imshow(outputImage1);//view the output
// 
// outputImage2 = detailEnhance(src,[],0.4) //providing value for second optional parameter
// imshow(outputImage2);//view the output
//
// outputImage3 = detailEnhance(src,121,0.4) // providing value all optional parameters
// imshow(outputImage3);//view the output
//
// Authors
//  Rohan Gurve
	
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
	
	sigma_s = argindefault ( varargin , 1 , 10 );//default value is 10
    sigma_r = argindefault ( varargin , 2 , 0.15 );//default value is 0.15
	
	out = raw_detailEnhance(image_list1,sigma_s,sigma_r);
	
	sz = size(out);
	
	for i=1:sz
		output_image(:, :, i) = out(i)
	end
	
endfunction


