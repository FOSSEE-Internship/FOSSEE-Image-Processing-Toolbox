function [varargout] = ForegroundDetector(varargin)	
// This function creates a ForeggroundDetector struct which is used to detect Foreground with Gaussian Mixture Models.
//
// Calling Sequence
//	 result = ForegroundDetector()
//	 result = ForegroundDetector(Name, Value, ..)
//
// Parameters
// results: ForegroundDetector struct containing AdaptLearningRate, NumTrainingFrames, MinimumBackgroundRatio, NumGaussians and LearningRate.
// AdaptLearningRate : (optional) If true, it enables the adaptation of Learning Rate during the period specified by number of training frames.
// NumTrainingFrames : (optional) The number of initial video frames employed to train the Background Model.
// MinimumBackgroundRatio : (optional) 
// NumGaussians : (optional) 
// LearningRate : (optional) This comes into action when AdaptLearningRate is enabled. 
//
// Description
// This function returns a ForegreoundDetector Struct which basically sets a background model and tracks changes in an input video frame with respect to the background and then computes the Foreground Mask. 
//
// Examples
// myStr = ForegroundDetector("NumGaussians", 4)
// 
// Authors
//  Shashank Shekhar
	[ lhs, rhs ] = argn(0)	
	if modulo(rhs,2)~=0 then
		error(msprintf("Incorrect number of input arguments"))
	end

	if rhs > 10 then
		error(msprintf("Too many input arguments"))
	end		
	if lhs > 1 then
		error(msprintf("Too many output arguments"))
	end
	select rhs 
		case 0 then	
			[fdAdaptLearningRate, fdNumTrainingFrames,fdMinimumBackgroundRatio, fdNumGaussians, fdLearningRate ]= ocv_ForegroundDetector()
			
		case 2 then		
			[fdAdaptLearningRate, fdNumTrainingFrames,fdMinimumBackgroundRatio, fdNumGaussians, fdLearningRate ] = ocv_ForegroundDetector(varargin(1), varargin(2))
		
		case 4 then
			[fdAdaptLearningRate, fdNumTrainingFrames,fdMinimumBackgroundRatio, fdNumGaussians, fdLearningRate ] = ocv_ForegroundDetector(varargin(1), varargin(2), varargin(3), varargin(4))
				
		case 6 then
			[fdAdaptLearningRate, fdNumTrainingFrames,fdMinimumBackgroundRatio, fdNumGaussians, fdLearningRate ] = ocv_ForegroundDetector(varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6))
				
		case 8 then
			[fdAdaptLearningRate, fdNumTrainingFrames,fdMinimumBackgroundRatio, fdNumGaussians, fdLearningRate ] = ocv_ForegroundDetector(varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(8))
			
		case 10 then
			[fdAdaptLearningRate, fdNumTrainingFrames,fdMinimumBackgroundRatio, fdNumGaussians, fdLearningRate ] = ocv_ForegroundDetector(varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(8), varargin(9), varargin(10))				
	end
		varargout(1) = struct("AdaptLearningRate", fdAdaptLearningRate, "NumTrainingFrames", fdNumTrainingFrames, "MinimumBackgroundRatio", fdMinimumBackgroundRatio, "NumGaussians", fdNumGaussians, "LearningRate", fdLearningRate);	
endfunction
