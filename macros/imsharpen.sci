// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Shreyash Sharma,Sukul Bagai
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [dstMat]=imsharpen(SrcImg)
// This function is used to sharpen an image. 
//
// Calling Sequence
// B = imsharpen(A)
//
// Parameters
// A: image matrix of the source image.
// B : output image with it's sharpened features. 
//
// Description
// B = imsharpen(A) returns an enhanced version of the grayscale or truecolor (RGB) input image A, where the image features, such as edges, have been sharpened.
//
// Examples
// i = imread('lena.jpeg');
// i1 = imsharpen(i);
// imshow(i1);
//
     
	
	srcMat = mattolist(SrcImg)

	out = raw_imsharpen(srcMat)
	[channel] = size(out)
	
	for i = 1: channel
		dstMat(:,:,i) = out(i)
	end
	
endfunction
