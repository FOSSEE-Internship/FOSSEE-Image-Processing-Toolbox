// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Siddhant Narang
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function image = imabsdiff(img1, img2)
// Calculates the per-element absolute difference between two images.
//
// Calling Sequence
// image = imabsdiff(img1, img2)
//
// Parameters
// img1: First input image.
// img2: Second input image.
//
// Description
// The function absdiff calculates- Absolute difference between two arrays when they have the same size and type-
// dst(I) = saturate(|src1(I) − src2(I)|)
// Absolute difference between an array and a scalar when the second array is constructed
// from Scalar or has as many elements as the number of channels in src1-
// dst(I) = saturate(|src1(I) − src2|)
// Absolute difference between a scalar and an array when the first array is constructed from Scalar or has
// as many elements as the number of channels in src2-
// dst(I) = saturate(|src1 − src2(I)|)
// where I is a multi-dimensional index of array elements. In case of multi-channel arrays, each channel is processed // independently.
//
// Examples
// img1 = imread("images/left1.jpg", 0);
// img2 = imread("images/right1.jpg", 0);
// image = imabsdiff(img1, img2);
//
// See also
// imread
//
// Authors
// Siddhant Narang
	
	[lhs rhs] = argn(0);
	if lhs > 1
        error(msprintf("Too many output arguments.\n"));
    end
	
	if rhs > 2 
        error(msprintf("Too many input arguments, maximum number of arguments is 2.\n"));
    elseif rhs < 2
        error(msprintf("The function needs atleast 2 arguments.\n"));
	end

	img_list1 = mattolist(img1);
	img_list2 = mattolist(img2);

	temp = raw_imabsdiff(img_list1, img_list2);

	channel = size(temp)
		
	for i = 1: channel
		image(:, :, i) = temp(i)
	end
endfunction