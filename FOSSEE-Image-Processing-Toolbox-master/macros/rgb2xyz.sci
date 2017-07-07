// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Tess Zacharias
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [output] = rgb2xyz(img)
// This function is used to convert rgb values of an image to their CIE 1931 xyz.
//
// Calling Sequence
// image=imread("lena.jpeg");
// xyz=imread(image);
//
// Parameters
// xyz: the converted image
// image: Input Image.
//
// Description
//
// This function is used to convert rgb values of an image to their CIE 1931 xyz.
//
// Examples
// z=imread("lena.jpeg");
// xyz=rgb2xyz(z);
// imshow(xyz);

	image = mattolist(img);
	a = raw_rgb2xyz(image);
	d = size(a);
	for i=1:d
		output(:,:,i) = a(i);
	end
endfunction
