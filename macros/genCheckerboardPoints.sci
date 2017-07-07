// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Tanmay Chaudhari & Gursimar Singh
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [worldPoints] = genCheckerboardPoints(boardSize, squareSize)
// This function returns list of checkerboard corner points.
//
// Calling Sequence
// worldPoints = genCheckerboardPoints(boardSize, squareSize);
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
// squareSize = 8;
// worldPoints = genCheckerboardPoints(boardSize, squareSize);
//
// See also
// detectCheckerboardCorner
//
// Authors
// Tanmay Chaudhari
// Gursimar Singh

    a= raw_genCheckerboardPoints(boardSize, squareSize);
    worldPoints(:,:,1)=a(1);
	
endfunction
