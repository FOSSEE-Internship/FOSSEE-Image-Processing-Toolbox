// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Shreyash Sharma
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [varargout] = detectAkazeFeatures(image, varargin)	
// This function is used to detect Akaze(accelerated Kaze) Features in a grayscale Image.
//
// Calling Sequence
//   result = detectAkazeFeatures(Image);
//   result = detectAkazeFeatures(Image, Name, Value, ...)
//
// Parameters
// result: BRISKPoints struct which contains Location of KeyPoints, Orientation, Metric, SignOfLaplacian, Scale and Count of the features.
// Image :  Input image, specified as a A-by-N 2D grayscale.
// MinContrast : (Optional) The minimum difference in intensity between a corner and its surrounding region. (Default: 0.2). The value must be between 0 and 1.
// NumOctaves : (Optional)The number of Octaves that the detector uses. (Default - 3) The value must be an integer scalar in between 1 and 4.
// MinQuality : (Optional) This specifies the minimum quality accepted for corners. (Default - 0.1) The value must be between 0 and 1.
// ROI : (Optional) Region Of Interest. This is taken as a vector [u v width height]. When specified, the function detects the key points within region of area width*height with u and v being the top
//       left corner coordinates.
//
// Description
// This function returns the Akaze features detected in a 2D grayscale image. 
//
// Examples
// image = imread('sample.jpg');
// results = detectAkazeFeatures(image);
//

	
	image_list = mattolist(image);
	[ lhs, rhs ] = argn(0)
	if rhs > 17 then
		error(msprintf("Too many input arguments"))
	end	
	if lhs > 1 then
		error(msprintf("Too many output arguments"))
	end		
	select rhs 
		case 1 then
			[a b c d e ]= ocv_detectAkazeFeatures(image_list)		
		case 3 then
			[a b c d e ]= ocv_detectAkazeFeatures(image_list, varargin(1), varargin(2))
		case 5 then
			[a b c d e ]= ocv_detectAkazeFeatures(image_list, varargin(1), varargin(2), varargin(3), varargin(4))		
		case 7 then
			[a b c d e ]= ocv_detectAkazeFeatures(image_list, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6))			
		case 9 then
			[a b c d e ]= ocv_detectAkazeFeatures(image_list, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(8))
		case 11 then
			[a b c d e ]= ocv_detectAkazeFeatures(image_list, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(8),varargin(9), varargin(10))
                case 13 then
			[a b c d e ]= ocv_detectAkazeFeatures(image_list, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(8),varargin(9), varargin(10),varargin(11), varargin(12))
                case 15 then
			[a b c d e ]= ocv_detectAkazeFeatures(image_list, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(8),varargin(9), varargin(10),varargin(11), varargin(12),varargin(13), varargin(14))
                case 17 then
			[a b c d e ]= ocv_detectAkazeFeatures(image_list, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(8),varargin(9), varargin(10),varargin(11), varargin(12),varargin(13), varargin(14),varargin(15), varargin(16))
                
	end
	varargout(1) = struct("Type","binaryFeatures",'Features',a,'NumBits',b,'NumFeatures',c,'KeyPoints',d,'keypointsCount',e);	
endfunction
