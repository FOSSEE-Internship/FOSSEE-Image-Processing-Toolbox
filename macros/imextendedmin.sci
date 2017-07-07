// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Vinay Bhat
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [dstMat] = imextendedmin(srcImg, Hmin)
//Extended-minima transform
//
//Calling Sequence
//dstMat=imextendedmin(srcImg, Hmin);
//
//Parameters
//dstMat:Output binary image
//srcImg:Input image
//Hmin:h-maxima transform, specified as a positive scalar.
//
//Description
//The function computes the extended-minima transform, which is the regional minima of the H-minima transform.h is a nonnegative scalar.
//
//Examples
//im=imread('images/lena.jpeg');
//img=imextendedmin(im,50);
//imshow(img);
//
//Authors
//Vinay Bhat

	srcMat = mattolist(srcImg)

	out = raw_imextendedmin(srcMat, Hmin)
	
	channel = size(out)
	
	for i = 1: channel
		dstMat(:,:,i) = out(i)
	end
	
endfunction
