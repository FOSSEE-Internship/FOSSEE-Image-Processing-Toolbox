function [out]=hasFrame()
// This function is used to check whether there is a frame to read in the current video, read through VideoReader.
//
// Calling Sequence
//   results = hasFrame();
//
// Parameters
// results: A boolean. True if there is a frame to read and false if not.
//
// Description
// This is a helper function for VideoReader and readFrame() function. It returns true if the video has a 
// frame to read, else it returns false.
//
// Examples
// videoStr = VideoReader('sample.mp4');
// hasFrame();
//
// Authors
//  Shashank Shekhar
//	Tanmay Chaudhari
         out=ocv_hasFrame();
endfunction;
