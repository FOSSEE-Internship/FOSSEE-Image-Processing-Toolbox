// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Riddhish Bhalodia
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function [outputImg] = wiener2(inputImage,filtsize,sigma)
// This function is used for 2-D adaptive noise-removal filtering. 
//
// Calling Sequence
// [outputImg] = wiener2(inputImage,filtsize,sigma)
// 
// Parameters
// inputImage : The input image, grayscale only.
// filtsize : The filter size.
// sigma : The additive noise (Gaussian white noise) power is assumed to be noise. if sigma = 0 then the variance is estimated from data
// outputImg : The output image, is of the same size and class as the input image
//
// Description
// It lowpass-filters a grayscale image that has been degraded by constant power additive noise. 
//
// Examples
// // a simple example
// a = imread("/images/m1.jpeg");
// filtsize = 5;
// sigma = 0;
// c = ssim(a,b);
//
// Authors
// Riddhish Bhalodia         	
	inputList = mattolist(inputImage);

	outputList = raw_wiener2(inputList,filtsize,sigma);
    
	for i=1:size(outputList)
	        
		outputImg(:,:,i)=outputList(i)

	end

endfunction
