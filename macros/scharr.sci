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

function new_image = scharr(image, ddepth, dx, dy, scale, delta)
// Calculates the first x- or y- image derivative using Scharr operator.
//
// Calling Sequence
// new_image = imcontrast(srcImg, aplha, beta)
//
// Parameters
// srcImg: input image.
// ddepth: output image depth. The possible ddepth values are the following <itemizedlist><listitem> CV_8U </listitem><listitem> CV_16U/CV_16S </listitem><listitem> CV_32F</listitem><listitem> CV_64F </listitem></itemizedlist> 
// dx: order of the derivative x.
// dy: order of the derivative y.
// scale: Scale factor for the computed derivative values.
// delta: Delta value that is added to the results.
// 
// Description
// This function is used to find the derivative of the source image using the
// Scharr operator.
//
// Examples
// image = imread("lena.jpg");
// new_image = scharr(image, "CV_8U", 2, 3, 1.5, 2);
//
// See also
// imread
//
// Authors
// Sukul Bagai

	image_list = mattolist(image)
	
	out = raw_scharr(image_list, ddepth, dx, dy, scale, delta)
	
	sz = size(out)
	
	for i = 1: sz
		new_image(:, :, i) = out(i)
	end
	
endfunction
