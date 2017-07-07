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

function [dstMat] = imhmax(srcImg,threshold, conn)
// This fucntion is used to get H-maxima transform in the form of an image.
//
// Calling Sequence
// I2 = imhmax(I,h,conn)
//
// Parameters
// I: image matrix of the source image.
// h: h-maxima transform, specified as a nonnegative scalar.
// conn: Connectivity, specified as a one of the scalar values in the following table. By default, imhmax uses 8-connected neighborhoods for 2-D images and 26-connected neighborhoods for 3-D images.
//         For higher dimensions, imhmax uses conndef(numel(size(I)),'maximal'). Connectivity can be defined in a more general way for any dimension by using for conn a 3-by-3-by- ...-by-3 matrix of 0s
//         and 1s. The 1-valued elements define neighborhood locations relative to the center element of conn. Note that conn must be symmetric around its center element.
// I2: Transformed image, returned as a nonsparse numeric array of any class, the same size as I. 
//
// Description
// I2 = imhmax(I,h) suppresses all maxima in the intensity image I whose height is less than h, where h is a scalar. Regional maxima are connected components of pixels with a constant intensity value,
//      and whose external boundary pixels all have a lower value. By default, imhmax uses 8-connected neighborhoods for 2-D images, and 26-connected neighborhoods for 3-D images. For higher
//      dimensions, imhmax uses conndef(ndims(I),'maximal').
//
// Examples
// i = imread('lena.jpeg',0);
// i4 = imhmax(i,300,8);
// imshow(i4);
//	
	[lhs, rhs] = argn(0)
	
	srcMat = mattolist(srcImg)

	out = raw_imhmax(srcMat, threshold, conn)
	
	channel = size(out)
	
	for i = 1: channel
		dstMat(:,:,i) = out(i)
	end
	
endfunction
