// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Vinay Bhat,Gursimar Singh
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [dstMat] = imhmin(srcImg, Hmin)
// This fucntion is used to get H-minima transform in the form of an image.
//
// Calling Sequence
// I2 = imhmin(I,h)
//
// Parameters
// I: image matrix of the source image.
// h: h-maxima transform, specified as a nonnegative scalar.
// I2: Transformed image, returned as a nonsparse numeric array of any class, the same size as I. 
//
// Description
// I2 = imhmin(I,h) suppresses all minima in the intensity image I whose depth is less than h, where h is a scalar. Regional minima are connected components of pixels with a constant intensity value, t, whose external boundary pixels all have a value greater than t 
//
// Examples
// i = imread('images/lena.jpeg');
// i2 = imhmin(i,200);
// imshow(i2);
//
//Authors
//Vinay Bhat
//Gursimar Singh
//
//See also
//imhmax
//imhistmax

	
	[lhs, rhs] = argn(0);

	 if rhs>2 then
     error(msprintf("Too many input arguments"));
     end
     if rhs<2 then
	 error(msprintf("Not enough input arguments"));
	 end
	 if lhs >1
	 error(msprintf("Too many output arguments"));
     end
     
	srcMat = mattolist(srcImg)


	out = raw_imhmin(srcMat, Hmin)
	
	channel = size(out)
	
	for i = 1: channel
		dstMat(:,:,i) = out(i)
	end
	
endfunction
