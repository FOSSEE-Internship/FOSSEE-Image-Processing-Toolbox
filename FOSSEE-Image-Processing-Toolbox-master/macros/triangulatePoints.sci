function [worldPoints] = triangulatePoints(projMat1, projMat2, points1, points2)
// Returns the worldPoint coordinates of the feature points. 
//
// Calling Sequence
// [worldPoints] = triangulatePoints(projMat1, projMat2, points1, points2)
//
// Parameters
// projMat1: 3X4 projection matrix for the first Camera matrix
// projMat2: 3X4 projection matrix for the second Camera matrix
// points1: 2xN array of feature points in the 1st image. In case of c++ version it can be also a vector of feature points or two-channel matrix of size 1xN or Nx1
// points2: 2xN array of feature points in the 2nd image. In case of c++ version it can be also a vector of feature points or two-channel matrix of size 1xN or Nx1 
// worldPoints: 4XN array or matrix of reconstructed points
//
// Description
// Returns 4D location of the matched feature points from the two projected matrices. The 4D location of the feature points is reconstructed using triangulation.
//
// Examples
// [projMat1] = [12 21 21 19; 34 12 0 2; 112 431 890 32.1]
// [projMat2] = [16 17 32 1; 64 90 12 11; 123 43.5 895 9.8]
// [points1] = [1 2 3 4;5 6 7 8]
// [points2] = [32 1 3 5; 9 8 3 4]
// [worldPoints] = triangulatePoints(projMat1, projMat2, points1, points2) 
//
//
// Authors
//   Deepshikha

// code for the above function
//
	
	output = opencv_triangulatePoints(projMat1, projMat2, points1, points2)
	
	channels = size(output)
	
	for i = 1:channels
		worldPoints(:,:,1) = output(1)
	end
	
endfunction

