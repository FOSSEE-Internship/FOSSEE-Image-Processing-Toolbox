// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Dhruti Shah
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function new_image = decorrstretch(image)
// Applies decorrelation stretch to an image.
//
// Calling Sequence
// outputImage = decorrstretch(image);
//
// Parameters
// image: Input image.
//
// Description
// This function applies decorrelation stretch to an input image which causes the image to enhance
// by way of amplifying image difference.
//
// Examples
// i = imread("images/lena.jpg");
// outputImage = decorrstretch(i);
// imshow(i);
//
// See also
// imread
// imshow
//
// Authors
// Dhruti Shah
	
	[lhs rhs] = argn(0)
	if rhs < 1
		error(msprintf("Not enough input arguments"));
	elseif rhs > 1
		error(msprintf("Too many input arguments"));
	end
	if lhs > 1
		error(msprintf("Too many output arguments"));
	end

	image_list = mattolist(image)
	out = opencv_decorrstretch(image_list)
	
	sz = size(out)
	for i = 1:sz
		new_image(:, :, i) = out(i)
	end
	
endfunction
