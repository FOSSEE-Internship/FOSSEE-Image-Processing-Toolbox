function [varargout] = detectBRISKFeatures(image, varargin)	
// This function is used to detect BRISK(Binary Robust Invariant Scalable Keypoints) Features in a grayscale Image.
//
// Calling Sequence
//   result = detectBRISKFeatures(Image);
//	 result = detectBRISKFeatures(Image, Name, Value, ...)
//
// Parameters
// result: BRISKPoints struct which contains Location of KeyPoints, Orientation, Metric, SignOfLaplacian, Scale and Count of the features.
// Image :  Input image, specified as a A-by-N 2D grayscale.
// MinContrast : (Optional) The minimum difference in intensity between a corner and its surrounding region. (Default: 0.2). The value must be between 0 and 1.
// NumOctaves : (Optional)The number of Octaves that the detector uses. (Default - 3) The value must be an integer scalar in between 1 and 4.
// MinQuality : (Optional) This specifies the minimum quality accepted for corners. (Default - 0.1) The value must be between 0 and 1.
// ROI : (Optional) Region Of Interest. This is taken as a vector [u v width height]. When specified, the function detects the key points within region of area width*height with u and v being the top left corner coordinates.
//
// Description
// This function returns the BRISK features detected in a 2D grayscale image. 
//
// Examples
// image = imread('sample.jpg');
// results = detectBRISKFeatures(image);
//
// Authors
//  Shashank Shekhar	
	
	image_list = mattolist(image);
	[ lhs, rhs ] = argn(0)
	if rhs > 9 then
		error(msprintf("Too many input arguments"))
	end	
	if lhs > 1 then
		error(msprintf("Too many output arguments"))
	end		
	select rhs 
		case 1 then
			[a b c d e]= ocv_detectBRISKFeatures(image_list)		
		case 3 then
			[a b c d e]= ocv_detectBRISKFeatures(image_list, varargin(1), varargin(2))
		case 5 then
			[a b c d e]= ocv_detectBRISKFeatures(image_list, varargin(1), varargin(2), varargin(3), varargin(4))		
		case 7 then
			[a b c d e]= ocv_detectBRISKFeatures(image_list, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6))			
		case 9 then
			[a b c d e]= ocv_detectBRISKFeatures(image_list, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(8))
	end
	varargout(1) = struct('Keypoints', a, 'Orientation', b, 'Metric', c ,'Scale', d, 'Count', e);	
endfunction
