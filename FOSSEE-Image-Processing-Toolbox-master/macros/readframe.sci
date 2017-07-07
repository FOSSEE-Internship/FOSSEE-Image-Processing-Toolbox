function [out]=readFrame(input)
// This function is used to return the next frame pointed to by the CurrentTime Property of A VideoReader Struct.
//
// Calling Sequence
//   results = readFrame();
//
// Parameters
// results: Frame of video.
//
// Description
// This function checks whether there is a next frame to grab in the video read through VideoReader and returns the frame.
//
// Examples
// videoStr = VideoReader('sample.mp4');
// if(hasFrame()) then
// frame = readFrame()
// end
//
// Authors
//  Shashank Shekhar
//	Tanmay Chaudhari
         out=ocv_readFrame();
endfunction;
