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
function [varargout] = detectSIFTFeatures(srcImg, varargin)	
// This function is used to find scale-invariant features.
// 
// Calling Sequence
// [ a ] = detectSIFTFeatures(srcImg)
// [ a ] = detectSIFTFeatures(srcImg, features, nOctaveLayers, contrastThreshold, edgeThreshold, sigma)
//
// Parameters
// srcImg : Hyper of input image.
// nfeatures : The number of best features to retain. The features are ranked by their scores (measured in SIFT algorithm as the local contrast). If valued as 0, uses all detected keypoints.
// nOctaveLayers : The number of layers in each octave. 3 is the value used in D. Lowe paper. The number of octaves is computed automatically from the image resolution.
// contrastThreshold : The contrast threshold used to filter out weak features in semi-uniform (low-contrast) regions. The larger the threshold, the less features are produced by the detector.
// edgeThreshold : The threshold used to filter out edge-like features. Note that the its meaning is different from the contrastThreshold, i.e. the larger the edgeThreshold, the less features are filtered out (more features are retained).
// sigma : The sigma of the Gaussian applied to the input image at the octave #0. If your image is captured with a weak camera with soft lenses, you might want to reduce the number.
// a : It is a struct consisting of 'Type'(Type of Feature) , 'Features'(descriptors) , 'NumBits', 'NumFeatures', 'KeyPoints', 'keypointsCount'.
// 
// Description
// For extracting keypoints and computing descriptors using the Scale Invariant Feature Transform. RGB images are converted to Grayscale images before processing. 
//
// Examples
// // with default values
// a = imread("/images/photo1.jpeg");
// b = imread("/images/photo2.jpeg");
// stacksize("max");
// c = detectSIFTFeatures(a);
// d = detectSIFTFeatures(b);
// [ e f ] = matchFeatures(c.Features, d.Features);
// out = drawMatch(a, b, c.KeyPoints, d.KeyPoints, e, f);
//
// Examples
// // user assigned values
// a = imread("/images/photo1.jpeg");
// b = imread("/images/photo2.jpeg");
// stacksize("max");
// c = detectSIFTFeatures(a, 0, 3, 0.05, 11, 1.6);
// d = detectSIFTFeatures(b, 0, 3, 0.05, 11, 1.6);
// [ e f ] = matchFeatures(c.Features, d.Features);
// out = drawMatch(a, b, c.KeyPoints, d.KeyPoints, e, f);
//
// Authors
// Ashish Manatosh Barik, NIT Rourkela  
	srcMat = mattolist(srcImg)

	[lhs, rhs] = argn(0)

	if rhs > 6 then
		error(msprintf("Too many input arguments"))
	end	
	if lhs > 1 then
		error(msprintf("Too many output arguments"))
	end		


	select rhs
		case 1 then
			[a b c d e] = raw_detectSIFTFeatures(srcMat)
		case 6 then
			[a b c d e] = raw_detectSIFTFeatures(srcMat, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5))
	end

	varargout(1) = struct('Type','Scale-Invariant-Features','Features',a,'NumBits',b,'NumFeatures',c,'KeyPoints',d,'keypointsCount',e);	

endfunction
