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

function [outputImg]=imgaussfilt3(inputImage, filter_height, filter_width, sigmaX, sigmaY)
// This function is used for 3-D Gaussian filtering of 3-D images 
//
// Calling Sequence
// B=imgaussfilt3(A, filter_height, filter_width, sigmaX, sigmaY)
// 
// Parameters
// A: image matrix of the source image.
// filter_height: height of the Gaussian filter, specified as a scalar 
// filter_width: width of the Gaussian filter, specified as a scalar 
// sigmax: Standard deviation in x of the Gaussian distribution, specified as a numeric, real, positive scalar. 
// sigmay: Standard deviation in y of the Gaussian distribution, specified as a numeric, real, positive scalar 
// B : output image with it's histogram matching similar to a given reference image. 
//
// Description
// imgaussfilt3(___,Value,...) filters 3-D image A with a 3-D Gaussian smoothing kernel with parameters used to control aspects of the filtering.
//
// Examples
// img = imread("lena.jpeg");
// imshow(img);
// filtered_img = imgaussfilt3(img, 9, 9, 3, 3);
// imshow(filtered_img);
//
	
	inputList=mattolist(inputImage);
        
        outputList=raw_imgaussfilt3(inputList, filter_height, filter_width, sigmaX, sigmaY);
    
        for i=1:size(outputList)
        
         outputImg(:,:,i)=outputList(i)
        
        end
        
endfunction
