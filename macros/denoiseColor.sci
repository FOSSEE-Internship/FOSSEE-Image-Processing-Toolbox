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
function [out] = denoiseColor(src, strengthFilter,strengthFilterColor, tempWindowSize, searchWindowSize)
//Removes colored noise from the image
//
//Calling Sequence
//outputImage = denoiseColor(src, strengthFilter,strengthFilterColor, tempWindowSize, searchWindowSize)
//
//Parameters
//src : The input image with colored noise.
//strengthFilter : The noise filter for white gaussian noise.
//strengthFilterColor : The noise filter for colored noise.
//tempWindowSize : Size in pixels of the template patch that is used to compute weights. Should be odd.
//searchWindowSize : Size in pixels of the window that is used to compute weighted average for given pixel. Should be odd.
//
//Description
//The function is used to remove colored noise from an image. It is the modification of fasNlmeansDenoising function
//The output is a denoised image
//
//Examples
//a = imread("taj.jpg");
//strengthFilter = 200;
//strengthFilterColor = 40;
//tempWindowSize = 7;
//searchWindowSize =21;
//p = denoiseColor(a,strengthFilter,strengthFilterColor,tempWindowSize,searchWindowSize);	
//
//Examples
//a = imread("cnoise1.jpg");
//strengthFilter = 20;
//strengthFilterColor = 100;
//tempWindowSize = 7;
//searchWindowSize =21;
//p = denoiseColor(a,strengthFilter,strengthFilterColor,tempWindowSize,searchWindowSize);
//
//Authors
//Shubham Lohakare, NITK Surathkal 
//Ashish Mantosh Barik, NIT Rourkela

	srcMat = mattolist(src)//Reading the image

	output = raw_denoiseColor(srcMat, strengthFilter,strengthFilterColor, tempWindowSize, searchWindowSize)

	channels = size(output)

	for i = 1 : channels
		out(:,:,i) = (output(i))
	end
	out = double(out)

endfunction

