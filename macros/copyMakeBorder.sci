// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Ashish Manatosh Barik, Shubheksha Jalan
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function new_image = copyMakeBorder(image, top, bottom, left, right, borderType, value)
// This function forms a border around the input image. 
//
// Calling Sequence
// [new_image] = copyMakeBorder(image, top, bottom, left, right, borderType, value)
// 
// Parameters
// image : The source image. 
// top : No. of pixels in this direction from the source image rectangle to extrapolate.
// bottom : No. of pixels in this direction from the source image rectangle to extrapolate.
// left : No. of pixels in this direction from the source image rectangle to extrapolate.
// right : No. of pixels in this direction from the source image rectangle to extrapolate.
// borderType : Stating the border type.
// value : Border value if borderType==BORDER_CONSTANT.
// new_image : The output image with specified borders.
//
// Description
// This function forms a border around the input image. The areas to the left, to the right, above and below the copied source image are filled with the extrapolated pixels. 
//
// Examples
// // a simple example
// a = imread("/images/lena.jpeg");
// top=1;
// bottom=1;
// left=1;
// right=1; 
// b = copyMakeBorder(a, top, bottom, left, right, "BORDER_CONSTANT", 1);
//
// Authors
// Ashish Manatosh Barik
// Shubheksha Jalan	
	image_list = mattolist(image)
	
	out = raw_copyMakeBorder(image_list, top, bottom, left, right, borderType, value)
		
	sz = size(out)
	
	for i = 1:sz
		new_image(:, :, i) = (out(i))
	end
			
endfunction
