// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Priyanka Hiranandani,Rohan Gurve 
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function [outputImg]=rgb2gray(inputImage)
//Given an original color image, this function converts it into a single channel grayscale image and return the same.
//
// Calling Sequence
// inputImage = imread("image-location-for-inputImage");
// outputImg = rgb2gray(inputImage)
//
// Parameters
// outputImage: single channel grayscale version of the inputImage 
// inputImage: Input source image 
// 
// Examples
// 
// src = imread("/images/color2.jpeg");
// gray = rgb2gray(src); 
// imshow(gray);//view the output grayscale image
//
// Authors
// Priyanka Hiranandani
// Rohan Gurve

    [lhs rhs]=argn(0);
    if rhs>1 //max i/p arguments is 1
        error(msprintf(" Too many input arguments"));
    elseif rhs<1 //min i/p argument is 1
        error(msprintf("input arguments missing"));
    end
    
    if lhs>1
	    error(msprintf("Too many output arguments"));
	end
	
    inputList=mattolist(inputImage);
    out=raw_rgb2gray(inputList);
    
    sz = size(out);
	
	for i=1:sz
		outputImg(:, :, i) = out(i)
	end
    
endfunction
