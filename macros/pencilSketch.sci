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
function [outputImage_1, outputImage_2 ] = pencilSketch(src,varargin)
// This function is used to give a non-photorealistic pencil-like  line drawing effect to the given input image.
//
// Calling Sequence
// src = imread("image-location-for-src");
// [outputImage_1 outputImage_2 ] = pencilSketch(src,sigma_s,sigma_r,shade_factor)
//
//Parameters
//outputImage_1: resultant Output image with single channel
//outputImage_2: resultant Output image with 3 channels 
//src: Input 8-bit 3-channel image.
//sigma_s: Range between 0 to 200 (float) (default value is 60)
//sigma_r: Range between 0 to 1. (float)  (default value is 0.07f)
//shade_factor: Range between 0 to 0.1 (default value is 0.02f)
//
// Examples
// 
// src = imread("/images/color2.jpeg");
// outputImage1 = pencilSketch(src) //using default values for all optional parameters
// imshow(outputImage1);//view the output
//
// outputImage2 = pencilSketch(src,100,[],0.07) //using default values for sigma_s
// imshow(outputImage2);//view the output
//
// outputImage3 = pencilSketch(src,100,0.4,0.07) //providing values for all optional parameters
// imshow(outputImage3);//view the output
//
// Authors
//  Rohan Gurve
	
	[lhs rhs]=argn(0);
    if rhs>4//max i/p arguments are 4
        error(msprintf(" Too many input arguments"));
    elseif rhs<1//min i/p arguments is 1
        error(msprintf("input arguments missing"));
    end
    
    if lhs>2
	    error(msprintf("Too many output arguments"));
	end
	
	image_list1 = mattolist(src);
	
	sigma_s = argindefault ( varargin , 1 , 60 );//default value is 60
    sigma_r = argindefault ( varargin , 2 , 0.07 );//default value is 0.07f
	shade_factor = argindefault  ( varargin , 3 , 0.02 ); //default value is 0.02f
	
	
	
	[out1 , out2] = raw_pencilSketch(image_list1,sigma_s,sigma_r,shade_factor);
	
	//sz = size(out);
	
	//we know that we'll receive 4 matrices
	//the first matrice will be for the single channel image
	//the rest 3 matrices will be for the 3 channel image 
	
	//retreiving the 1 channel image
    outputImage_1(:, :, 1) = out1(1); 
    //retreiving the 3 channel image
     for i=1:size(out2)
		outputImage_2(:, :, i) = out2(i)
	end
	
endfunction


