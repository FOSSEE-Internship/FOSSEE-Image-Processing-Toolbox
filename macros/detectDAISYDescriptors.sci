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
function [varargout] = detectDAISYDescriptors(srcImg, varargin)	
// This function is used for computing DAISY descriptors using Star keypoints.
// 
// Calling Sequence
// [ a ] = detectDAISYDescriptors(srcImg)
// [ a ] = detectDAISYDescriptors(srcImg, maxSize, responseThreshold, lineThresholdProjected, lineThresholdBinarized, suppressNonmaxSize, radius, q_radius, q_theta, q_hist, norm, interpolation, use_orientation)
// [ a ] = detectDAISYDescriptors(srcImg, maxSize, responseThreshold, lineThresholdProjected, lineThresholdBinarized, suppressNonmaxSize, radius, q_radius, q_theta, q_hist, norm, homography, interpolation, use_orientation)
//
// Parameters
// srcImg : Hyper of input image
// maxSize : Choose the number of filters to be applied, the parameter value set the maximum size.
// responseThreshold : To eliminate weak corners.
// lineThresholdProjected : Harris of responses.
// lineThresholdBinarized : Harris of sizes.
// suppressNonmaxSize : Window size (n-by-n) to apply the non-maximal suppression.
// radius : radius of the descriptor at the initial scale.
// q_radius : amount of radial range division quantity.
// q_theta : amount of angular range division quantity.
// q_hist : amount of gradient orientations range division quantity. 
// norm	: choose descriptors normalization type, where DAISY::NRM_NONE will not do any normalization (default), DAISY::NRM_PARTIAL mean that histograms are normalized independently for L2 norm equal to 1.0, DAISY::NRM_FULL mean that descriptors are normalized for L2 norm equal to 1.0, DAISY::NRM_SIFT mean that descriptors are normalized for L2 norm equal to 1.0 but no individual one is bigger than 0.154 as in SIFT   
// homography :	optional 3x3 homography matrix used to warp the grid of daisy but sampling keypoints remains unwarped on image.
// interpolation : switch to disable interpolation for speed improvement at minor quality loss.
// use_orientation : sample patterns using keypoints orientation, disabled by default.   
// a : It is a struct consisting of 'Type'(Type of Feature) , 'Features'(descriptors) , 'NumBits', 'NumFeatures', 'KeyPoints', 'keypointsCount'.
// 
// Description
// For extracting keypoints(using StarDetector) and computing descriptors(DAISY).
//
// Examples
// // with default values
// [ a ] = imread("/images/b1.jpeg");
// [ b ] = imread("/images/b2.jpeg");
// stacksize('max')
// [ c ] = detectDAISYDescriptors(a);
// [ d ] = detectDAISYDescriptors(b);
// [ e f ] = matchFeatures(c.Features, d.Features);
// out = drawMatch(a, b, c.KeyPoints, d.KeyPoints, e, f);
//
// Examples
// // user assigned values
// [ a ] = imread("/images/b1.jpeg");
// [ b ] = imread("/images/b2.jpeg");
// stacksize('max')
// [ c ] = detectDAISYDescriptors(a, 45, 30, 10, 8, 5, 15, 3, 8, 8, 100, %t, %f);
// [ d ] = detectDAISYDEscriptors(b, 45, 30, 10, 8, 5, 15, 3, 8, 8, 100, %t, %f);
// [ e f ] = matchFeatures(c.Features, d.Features);
// out = drawMatch(a, b, c.KeyPoints, d.KeyPoints, e, f);
//
// Authors
// Ashish Manatosh Barik NIT Rourkela
// Shubham Lohakare, NITK Surathkal 
	srcMat = mattolist(srcImg)

	[lhs, rhs] = argn(0)

	if rhs > 14 then
		error(msprintf("Too many input arguments"))
	end	
	if lhs > 1 then
		error(msprintf("Too many output arguments"))
	end		


	select rhs
		case 1 then
			[a b c d e] = raw_detectDAISYDescriptors(srcMat)
		case 13 then
			[a b c d e] = raw_detectDAISYDescriptors(srcMat, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(8), varargin(9), varargin(10), varargin(11), varargin(12))
		case 14 then
			[a b c d e] = raw_detectDAISYDescriptors(srcMat, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(8), varargin(9), varargin(10), varargin(11), varargin(12), varargin(13))
	end

	varargout(1) = struct('Type','Brief features','Features',a,'NumBits',b,'NumFeatures',c,'KeyPoints',d,'keypointsCount',e);	

endfunction
