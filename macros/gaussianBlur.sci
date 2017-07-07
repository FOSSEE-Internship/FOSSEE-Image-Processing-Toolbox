// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Sukul Bagai 
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function [outputImg]= gaussianBlur(inputImage,ksize_height,ksize_width,sigmaX,sigmaY)
// This function blurs the input image using a Gaussian filter.
//
// Calling Sequence
// outputImg = gaussianblur(inputImage,ksize_height,ksize_width,sigmaX,sigmaY)
//
// Parameters
// inputImage : The input source image. 
// ksize_height : It is the gaussian kernel height. It must be positive and odd.
// ksize_width : It is the gaussian kernel width. It must be positive and odd.
// sigmaX : It is the gaussian kernel standard deviation in X direction.
// sigmaY : It is the gaussian kernel standard deviation in Y direction.
// outputImg : The output filtered image is of the same size and type as the input image.
//
// Description
// The function convolves the source image with the specified Gaussian kernel.
//
// Examples
// inputImage = imread('/images/lena.jpg');
// outputImg = gaussianBlur(inputImage,5,5,1,1);
//	
// Authors
// Sukul Bagai	
	inputList=mattolist(inputImage);
    	
	outputList=raw_gaussianBlur(inputList,ksize_height,ksize_width,sigmaX,sigmaY);
    
	for i=1:size(outputList)
       		outputImg(:,:,i)=outputList(i)
   	end

endfunction
