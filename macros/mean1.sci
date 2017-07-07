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

function [out]=mean1(image)
// This function is used to calculate average or mean of matrix elements. 
//
// Calling Sequence
// B = mean1(A)
//
// Parameters
// A: image matrix of the source image.
// B : output mean value of the matrix elements of the input image. 
//
// Description
// B = mean1(A) computes the mean of the values in A.
//
// Examples
// i1 = imread('lena.jpeg');
// i2 = mean(i);
// imshow(i2);
//
	 image1=mattolist(image);
         out=raw_mean1(image1);
endfunction;
