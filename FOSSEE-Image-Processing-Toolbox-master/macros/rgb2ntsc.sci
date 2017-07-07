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

function [output] = rgb2ntsc(img)
// This function is used to convert the range of rgb values to the range ntsc values.
//
// Calling Sequence
// image1 = imread(img);
// image2 = rgb2ntsc(image1);
//
// Parameters
// image1: image matrix of the source image.
// image2: image matrix of the resultant image.
//
// Description
// This function takes an rgb image and transforms the channels of the image in accordance with the ntsc values.
//
// Examples
// i = imread("lena.jpeg");
// rr = rgb2ntsc(i);
// imshow(rr(:,:,1));
// imshow(rr(:,:,2));
// imshow(rr(:,:,3));
//

	image = mattolist(img);
	a = raw_rgb2ntsc(image);
	d = size(a);
	for i=1:d
		output(:,:,i) = a(i);
	end
endfunction
