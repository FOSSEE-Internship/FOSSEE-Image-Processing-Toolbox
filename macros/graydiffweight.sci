// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Yash S. Bhalgat
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [out] = graydiffweight(image, refgrayval)
// Calculate weights for image pixels based on grayscale intensity difference.
//
// Calling Sequence
// W = graydiffweight(srcImage, refGrayVal)
//
// Parameters
// srcImage: Input image.
// refGrayVal: Reference grayscale intensity value, specified as a scalar.
// 
// Description
// This function computes the pixel weight for each pixel in the grayscale image I. The weight is the absolute
// value of the difference between the intensity of the pixel and the reference grayscale intensity
// specified by the scalar refGrayVal. Pick a reference grayscale intensity value that is representative
// of the object you want to segment. The weights are returned in the array W, which is the same size as
// input image I. The weight of a pixel is inversely related to the absolute value of the grayscale intensity 
// difference at the pixel location. If the difference is small (intensity value close to refGrayVal), the
// weight value is large. If the difference is large (intensity value very different from refGrayVal), the
// weight value is small.
//
// Examples
// image = imread("images/lena.jpg");
// W = graydiffweight(image, 120);
//
// See also
// imread 
//
// Authors
// Dhruti Shah

	image1 = mattolist(image);

    a = raw_graydiffweight(image1, refgrayval);

    dimension = size(a)
	
	for i = 1:dimension
		out(:,:,i)=a(i);
	end
endfunction;
