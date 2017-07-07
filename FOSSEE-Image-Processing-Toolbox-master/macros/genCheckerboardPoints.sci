function [ worldPoints	 ] = genCheckerboardPoints(boardSize, squareSize)
// This function returns list of checkerboard corner points.
//
// Calling Sequence
//   worldPoints = genCheckerboardPoints(boardSize, squareSize);
//
// Parameters
// worldPoints: A list of size N-by-2 matrix containing x and y co-ordinates of the checkerboard corner points. The origin is set to lower-right corner of the top-left square.
// boardSize: Size of the checkerboard.
// squareSize: Size of each of square in the checkerboard.
//
// Description
// Return a list of size N-by-2, which contains the corner points of all squares of a checkerboard.
//
// Examples
// boardSize = [10 10];
// 	
// worldPoints = genCheckerboardPoints(boardSize, squareSize);
//
// Authors
// Tanmay Chaudhari

    a= raw_genCheckerboardPoints(boardSize, squareSize);
    worldPoints(:,:,1)=a(1);
	
endfunction
