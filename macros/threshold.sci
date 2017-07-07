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

function [outputImg,res]= threshold(inputImage, threshold_value, max_value,thresholdType)
// This function is used to apply an adaptive threshold to an array. 
//
// Calling Sequence
// B = threshold(A, threshold_value, max_value,thresholdType);
//
// Parameters
// A: image matrix of the source image.
// threshold_value: The thresh value with respect to which the thresholding operation is made.
// max_value: The value used with the Binary thresholding operations (to set the chosen pixels).
// thresholdType: One of the 5 thresholding operations. eg,THRESH_BINARY,THRESH_BINARY_INV,THRESH_TRUNC,THRESH_TOZERO,THRESH_TOZERO_INV.
// B : output image by applying the threshold operation. 
//
// Description
// The function transforms a grayscale image to a binary image using a formulae according to the given threshold type.
//
// Examples
// i = imread('lena.jpeg',0);
// [ii ii1] = threshold(i,50,255,"THRESH_BINARY");
// imshow(ii);

	
	inputList=mattolist(inputImage);
        
        [outputList,res]=raw_threshold(inputList, threshold_value, max_value,thresholdType)
     
     for i=1:size(outputList)
        outputImg(:,:,i)=outputList(i)
     end
endfunction
