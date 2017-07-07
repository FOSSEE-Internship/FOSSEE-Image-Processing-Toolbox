function[varargout] = VideoReader(varargin)
// This function creates a SURFPoints struct, needed to read Video Files.
//
// Calling Sequence
//	 result = VideoReader(filename)
//	 result = VideoReader(filename, Name, Value)
//
// Parameters
// results: VideoReader Struct containing Name, Duration, FramesPerSecond, CurrentTime, FramesCount, Width, Height and Path.
// filename: name of the video file
// CurrentTime : (optional) The timestamp of video in seconds from the beginning of the video.
//
// Description
// This function returns a VideoReader struct with the properties of the video
//
// Examples
// videoStruct = VideoReader('sample.mp4');
// 
// Authors
//  Shashank Shekhar
//	Tanmay Chaudhari
	[lhs, rhs] = argn(0)
	if rhs>3 then
		error(msprintf(" Too many input arguments. One expected"))
	end
	if lhs>1 then
		error(msprintf(" Too many output arguments. One expected"))
	end

	if rhs==3 then 
		[FileName, duration, fps, currentTime, frames, width, height, fullPath] = ocv_VideoReader(varargin(1), varargin(2), varargin(3));
	else
		[FileName, duration, fps, currentTime, frames, width, height, fullPath] = ocv_VideoReader(varargin(1));	
	end	
		
	varargout(1) = struct('Filename', FileName, 'Duration', duration, 'FramesPerSecond', fps, 'CurrentTime', currentTime, 'FramesCount', frames, 'Width', width, 'Height', height, "Path", fullPath);		
endfunction
