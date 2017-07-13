// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Samiran Roy, Shubham Lohakare 
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function [dstMat,squaredImage,rotatedImage] = integralImage(srcImg, varargin)
//The function calculates the integral of the source image passed.
//
//Calling Sequence
//[outputImage,squaredImage,rotatedImage] = integralImage(srcImg)
//[outputImage,squaredImage,rotatedImage] = integralImage(srcImg,method)
//
//Parameters
//srcImg : The input image
//method : The type of output frame in which the image is to be returned. 'upright' gives the original output whereas 'rotated' gives the output rotated by 45 degrees.
//
//Description
//The function can be used to calculate integral images for the provided input image. These integrals can be used to calculate the sum or mean or even standard deviation. It makes it feasible to apply fast blurring or fast block co-relation.
//
//Examples
//a = imread("lena.jpeg");
//[p, q, r] = integralImage(a);
//imshow(p);
//imshow(q);
//imshow(r);
//
//Examples
//a = imread("lena1.jpeg");
//[p, q, r] = integralImage(a);
//imshow(p);
//imshow(q);
//imshow(r);
//
//Authors
//Samiran Roy
//Shubham Lohakare 
	[lhs, rhs] = argn(0)
	
	srcMat = mattolist(srcImg)

	select rhs
		case 1 then
			[out1,out2,out3] = raw_integralImage(srcMat)
		case 3 then
			[out1,out2,out3] = raw_integralImage(srcMat, varargin(1))
	end
	
	channels1 = size(out1)
	channels2 = size(out2)
	channels3 = size(out3)
	
	for i = 1: channels1
		dstMat(:,:,i) = (out1(i))
	end
	
	for j = 1:channels2
		squaredImage(:,:,j) = (out2(j))
	end
	for k = 1:channels3
		rotatedImage(:,:,k) = (out3(k))
	end
endfunction
