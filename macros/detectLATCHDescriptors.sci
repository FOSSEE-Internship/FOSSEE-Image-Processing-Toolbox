// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Ashish Manatosh Barik & Shubham Lohakare
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function [varargout] = detectLATCHDescriptors(srcImg, varargin)	
// This function is used for computing the LATCH descriptors using Star keypoints.
// 
// Calling Sequence
// [ a ] = detectLATCHDescriptors(srcImg)
// [ a ] = detectLATCHDescriptors(srcImg, maxSize, responseThreshold, lineThresholdProjected, lineThresholdBinarized, suppressNonmaxSize, bytes, rotationInvariance, half_ssd_size)
//
// Parameters
// srcImg : Hyper of input image
// maxSize : Choose the number of filters to be applied, the parameter value set the maximum size.
// responseThreshold : To eliminate weak corners.
// lineThresholdProjected : Harris of responses.
// lineThresholdBinarized : Harris of sizes.
// suppressNonmaxSize : Window size (n-by-n) to apply the non-maximal suppression.
// bytes : It is the size of the descriptor - can be 64, 32, 16, 8, 4, 2 or 1. 
// rotationInvariance : whether or not the descriptor should compansate for orientation changes.
// half_ssd_size) : the size of half of the mini-patches size. For example, if we would like to compare triplets of patches of size 7x7x then the half_ssd_size should be (7-1)/2 = 3.
// a : It is a struct consisting of 'Type'(Type of Feature) , 'Features'(descriptors) , 'NumBits', 'NumFeatures', 'KeyPoints', 'keypointsCount'.
// 
// Description
// For extracting keypoints(using StarDetectors) and computing descriptors(LATCH). 
//
// Examples
// // with default values
// [ a ] = imread("/images/b1.jpeg");
// [ b ] = imread("/images/b2.jpeg");
// stacksize('max')
// [ c ] = detectLATCHdescriptors(a);
// [ d ] = detectLATCHDescriptors(b);
// [ e f ] = matchFeatures(c.Features, d.Features);
// out = drawMatch(a, b, c.KeyPoints, d.KeyPoints, e, f);
//
// Examples
// // user assigned values
// [ a ] = imread("/images/b1.jpeg");
// [ b ] = imread("/images/b2.jpeg");
// stacksize('max')
// [ c ] = detectLATCHdescriptors(a, 45, 30, 10, 8, 5, 32, %t, 3);
// [ d ] = detectLATCHDEscriptors(b, 45, 30, 10, 8, 5, 32, %t, 3);
// [ e f ] = matchFeatures(c.Features, d.Features);
// out = drawMatch(a, b, c.KeyPoints, d.KeyPoints, e, f);
//
// Authors
// Ashish Manatosh Barik, NIT Rourkela
// Shubham Lohakare, NITK Surathkal
	srcMat = mattolist(srcImg)

	[lhs, rhs] = argn(0)

	if rhs > 9 then
		error(msprintf("Too many input arguments"))
	end	
	if lhs > 1 then
		error(msprintf("Too many output arguments"))
	end		


	select rhs
		case 1 then
			[a b c d e] = raw_detectLATCHDescriptors(srcMat)
		case 9 then
			[a b c d e] = raw_detectLATCHDescriptors(srcMat, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(8))
	end

	varargout(1) = struct('Type','Brief features','Features',a,'NumBits',b,'NumFeatures',c,'KeyPoints',d,'keypointsCount',e);	

endfunction
