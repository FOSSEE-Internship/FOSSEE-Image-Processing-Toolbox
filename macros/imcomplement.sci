// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Tess Zacharias, Shubham Lohakare 
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function [compMat] = imcomplement(srcImg)
//Computes the complement of the image
//
//Calling Sequence
//compMat = imcomplement(srcImg)
//
//Parameters
//srcImg : Input image
//
//Description
//In the complement of a binary image, zeros become ones and ones become zeros; black and white are reversed. In the complement of an intensity or RGB image, each pixel value is subtracted from the maximum pixel value supported by the class (or 1.0 for double-precision images) and the difference is used as the pixel value in the output image. In the output image, dark areas become lighter and light areas become darker.
//
//Examples
//a = imread("lena.jpeg");
//k = imcomplement(a);
//imshow(k)
//
//Examples
//a = imread("photo1.jpg");
//k = imcomplement(a);
//imshow(k)
//
//Authors
//Tess Zacharias
//Shubham Lohakare	
	srcMat = mattolist(srcImg)

	out = raw_imcomplement(srcMat)

	ch = size(out)
	
	for i=1:ch
		compMat(:,:,i) = out(i)
	end
	
endfunction
