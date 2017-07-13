// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Vinay Bhat, Shubham Lohakare 
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function [dstMat] = imfill(srcImg, varargin)
//Fills in the regions and holes in an image.
//
//Calling Sequence
//[outputImage] = imfill(srcImg)
//[outputImage] = imfill(srcImg,locations)
//
//Parameters
//srcImg : It is the input image
//locations :  If locations is a P-by-1 vector, it contains the linear indices of starting locations. If locations is P-by-ndims(BW) matrix, each row contains the array indices of one of the starting locations.
//
//Description
//The function performs a flood-fill operation on the pixels of the input image.
//
//Examples
//k = imread("lena.jpeg");
//im = imfill(k, [1 1; 10 10; 11 12]);
//imshow(im);
//
//Examples
//k = imread("lena1.jpeg");
//im = imfill(k, [1 1; 20 20; 11 12;50 50;]);
//imshow(im);
//
//Examples
//k = imread("penguin.jpg");
//p = imfill(k);
//imshow(p);
//
//Authors
//Vinay Bhat
//Shubham Lohakare 	
	[lhs, rhs] = argn(0)
	
	srcMat = mattolist(srcImg)

	select rhs
		case 1 then
			out = raw_imfill(srcMat)
		case 2 then
			out = raw_imfill(srcMat, varargin(1))
	end
	
	channel = size(out)
	
	for i = 1: channel
		dstMat(:,:,i) = out(i)
	end
	
endfunction
