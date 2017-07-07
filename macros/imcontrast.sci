// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Sukul Bagai & Siddhant Narang
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function contrastMat = imcontrast(srcImg, alpha, beta)
// Adjusts image contrast.
//
// Calling Sequence
// new_image = imcontrast(srcImg, aplha, beta)
//
// Parameters
// srcImage: Input image.
// alpha: Pixel multiplier controls the weight of each pixel.
// beta: Added to every pixel to change the range of pixel values.
// 
// Description
// This function is used to change the contrast of the image using
// using the values alpha and beta.
//
// Examples
// image = imread("images/lena.jpg");
// new_image = imcontrast(image, 1.2, 2);
//
// See also
// imread
//
// Authors
// Sukul Bagai
// Siddhant Narang

	srcMat = mattolist(srcImg)

	temp = raw_imcontrast(srcMat, alpha, beta)

	sz = size(temp)
	for i = 1: sz
		contrastMat(:, :, i) = temp(i)
	end
endfunction
