// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Shubham Lohakare, Ashish Manatosh Barik 
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function [out] = fastNlMeansDenoising(src, strengthFilter, tempWindowSize, searchWindowSize, choice)
//Removes gaussian white noise from images
//
//Calling Sequence
//outputImage=fastNlMeansDenoising(src, strengthFilter, tempWindowSize, searchWindowSize, choice)
//
//Parameters
//src : Input 8-bit 1-channel, 2-channel, 3-channel or 4-channel image.
//strengthFilter : Parameter regulating filter strength. 
//templateWindowSize : Size in pixels of the template patch that is used to compute weights. Should be odd.
//searchWindowSize : Size in pixels of the window that is used to compute weighted average for given pixel. Should be odd.
//choice : Chooses the method  
//
//Description
//Perform image denoising using Non-local Means Denoising algorithm with several computational optimizations. Noise expected to be a gaussian white noise.
//
//Examples
//a = imread("man.jpg");
//strengthFilter = 30;
//tempWindowSize = 7;
//searchWindowSize = 40;
//choice=2;
//k = fastNlMeansDenoising(a, strengthFilter, tempWindowSize, searchWindowSize,choice);
//
//Examples
//a = imread("noise.jpg");
//strengthFilter = 100;
//tempWindowSize = 7;
//searchWindowSize = 50;
//choice = 3;
//k = fastNlMeansDenoising(a, strengthFilter, tempWindowSize, searchWindowSize,choice);
//
//Authors
//Shubham Lohakare, NITK Surathkal
//Ashish Mantosh, NIT Rourkela	
	srcMat = mattolist(src)

	output = raw_fastNlMeansDenoising(srcMat, strengthFilter, tempWindowSize, searchWindowSize,choice)

	channels = size(output)

	for i = 1 : channels
		out(:,:,i) = (output(i))
	end
	out = double(out)

endfunction

