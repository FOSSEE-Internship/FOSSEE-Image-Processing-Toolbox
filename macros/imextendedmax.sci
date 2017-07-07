// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Vinay Bhat
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function dstMat = imextendedmax(srcImg, Hmax)
// Extended-maxima transform.
//
// Calling Sequence
// BW = imextendedmax(srcImage, Hmax)
//
// Parameters
// srcImage: Input image.
// Hmax: H-maxima transform, specified as a real, nonnegative scalar.
// 
// Description
// This functoin returns the extended-maxima transform for I, which is the regional maxima of the
// H-maxima transform. Regional maxima are connected components of pixels with a constant intensity
// value, and whose external boundary pixels all have a lower value. H is a nonnegative scalar.
//
// Examples
// image = imread("images/lena.jpg");
// new_image = imextendedmax(image, 80);
//
// See also
// imread 
//
// Authors
// Vinay Bhat

	srcMat = mattolist(srcImg)

	out = raw_imextendedmax(srcMat, Hmax)
	
	channel = size(out)
	
	for i = 1: channel
		dstMat(:, :, i) = out(i)
	end
	
endfunction
