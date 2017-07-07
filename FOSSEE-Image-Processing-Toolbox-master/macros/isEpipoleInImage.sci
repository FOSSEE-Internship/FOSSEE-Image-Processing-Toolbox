// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Shreyash Sharma,Suraj Prakash
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [isepi, varargout ] = isEpipoleInImage(fundamental_matrix, imagesize)
// Find whether image contains epipole.
//
// Calling Sequence
// isepi = isEpipoleInImage(F, imagesize)
// [isepi, epipole] = isEpipoleInImage(F, imagesize)
//
// Parameters
// F : A 3 * 3 fundamental matrix computed from stereo images. It should be double or single
// imagesize : The size of the image
// isepi : Logical value true / false denoting whether the image contains epipole
// epipole : Location of the epipole. It is 1 * 2 vector.
// 
// Description
// The function determines whether the image with fundamental matrix F contains the epipole or not. It also gives the position of the epipole. 
//
// Examples
// i = imread('left11.jpg',0);
// i1 = imread('right11.jpg',0);
// new1 = detectCheckerboardCorner(i1,[7,10]);
// new1 = detectCheckerboardCorner(i,[7,10]);
// new2 = detectCheckerboardCorner(i1,[7,10]);
// f1 = estimateFundamentalMat(new1,new2);
// [isep isep2] = isEpipoleInImage(f1,[360 640]);
//
	
	[ lhs, rhs ] = argn(0)
	
	if lhs > 2 then
		error(msprintf("Too many output arguments"));	
	end
	/// If there is more than one output parameter
	[rows cols] = size(fundamental_matrix)
	if rows ~= 3 | cols ~=3 then
		error(msprintf("Invalid size of fundamental matrix\n"));
	end
	//	[rows1 col2] = size(imagesize)
	//if rows1 ~=1 | cols ~= 2 then
	//	error(msprintf("Invalid image size matrix\n"));
	//end
	if lhs == 2 then
		[isepi, temp ] = raw_isEpipoleInImage(fundamental_matrix, imagesize);
		varargout(1) = temp;
	/// if there is only one output parameter
	else
		isepi = raw_isEpipoleInImage(fundamental_matrix, imagesize);
	end
	
endfunction
