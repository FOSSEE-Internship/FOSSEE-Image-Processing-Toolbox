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

function cropImgs = imcrop(srcImg, xcoor, ycoor, width, height)
// Crops the image.
//
// Calling Sequence
// new_image = imcrop(srcImg, xcoor, ycoor, width, height)
//
// Parameters
// srcImage: Input image.
// xcoor: The x-coordinate of the starting point of the region of interest, ie region to be cropped.
// ycoor: The y-coordinate of the starting point of the region of interest.
// width: The total width of the region of interest wrt to the starting point.
// height: The total height of the region of interest wrt to the starting point.
// 
// Description
// This function can be used to crop the image to a given region of interest.
//
// Examples
// image = imread("images/lena.jpg");
// new_image = imcrop(image, 10, 10, 100, 100);
//
// See also
// imread 
//
// Authors
// Sukul Bagai

	srcMat = mattolist(srcImg)
	cropImg = raw_imcrop(srcMat, xcoor, ycoor, width, height)
        channel = size(cropImg)          // for converting to hyper matrix
	
	for i = 1: channel
		cropImgs(:, :, i) = (cropImg(i))
	end
	cropImgs = double(cropImgs)
endfunction
