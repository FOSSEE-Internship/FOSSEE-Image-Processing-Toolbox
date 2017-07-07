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
function [varargout] = detectBRIEFDescriptors(srcImg, varargin)	
// This function is used for computing BRIEF descriptors using Star keypoints.
// 
// Calling Sequence
// [ a ] = detectBRIEFDescriptors(srcImg)
// [ a ] = detectVRIEFDescriptors(srcImg, maxSize, responseThreshold, lineThresholdProjected, lineThresholdBinarized, suppressNonmaxSize, bytes, use_orientation	)
//
// Parameters
// srcImg : Hyper of input image
// maxSize : Choose the number of filters to be applied, the parameter value set the maximum size.
// responseThreshold : To eliminate weak corners.
// lineThresholdProjected : Harris of responses.
// lineThresholdBinarized : Harris of sizes.
// suppressNonmaxSize : Window size (n-by-n) to apply the non-maximal suppression.
// bytes : legth of the descriptor in bytes, valid values are: 16, 32 (default) or 64.
// use_orientation : sample patterns using keypoints orientation, disabled by default. 
// a : It is a struct consisting of 'Type'(Type of Feature) , 'Features'(descriptors) , 'NumBits', 'NumFeatures', 'KeyPoints', 'keypointsCount'.
// 
// Description
// For extracting keypoints(StarDetector) and computing descriptors. BRIEF which gives the shortcut to find binary descriptors with less memory, faster matching, still higher recognition rate.
//
// Examples
// // with default values
// [ a ] = imread("/images/b1.jpeg");
// [ b ] = imread("/images/b2.jpeg");
// stacksize("max);
// [ c ] = detectBRIEFDescriptors(a);
// [ d ] = detectBRIEFDescriptors(b);
// [ e f ] = matchFeatures(c.Features, d.Features);
// out = drawMatch(a, b, c.KeyPoints, d.KeyPoints, e, f);
//
// Examples
// // user assigned values
// [ a ] = imread("/images/b1.jpeg");
// [ b ] = imread("/images/b2.jpeg");
// stacksize("max);
// [ c ] = detectBRIEFDescriptors(a, 45, 30, 10, 8, 5, 32, %f);
// [ d ] = detectBRIEFDEscriptors(b, 45, 30, 10, 8, 5, 32, %f);
// [ e f ] = matchFeatures(c.Features, d.Features);
// out = drawMatch(a, b, c.KeyPoints, d.KeyPoints, e, f);
//
// Authors
// Ashish Manatosh Barik, NIT Rourkela
// Shubham Lohakare, NITK Surathkal 
	srcMat = mattolist(srcImg)

	[lhs, rhs] = argn(0)

	if rhs > 8 then
		error(msprintf("Too many input arguments"))
	end	
	if lhs > 1 then
		error(msprintf("Too many output arguments"))
	end		


	select rhs
		case 1 then
			[a b c d e] = raw_detectBRIEFDescriptors(srcMat)
		case 8 then
			[a b c d e] = raw_detectBRIEFDescriptors(srcMat, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7))
	end

	varargout(1) = struct('Type','Brief features','Features',a,'NumBits',b,'NumFeatures',c,'KeyPoints',d,'keypointsCount',e);	

endfunction
