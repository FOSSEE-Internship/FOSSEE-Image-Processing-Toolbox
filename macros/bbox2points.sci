// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author:Tanmay Chaudhari
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [ out ] = bbox2points(rectangle)
// Returns list of corner points of a rectangle.
//
// Calling Sequence
// bbox = bbox2points(rectangle);
//
// Parameters
// rectangle: A Nx4 matrix where each row is a rectangle of the form [x, y, width, height];
// points: Returns 4x2xN size matrix which contains all the 4 co-ordinates of each of the N bounding boxes.  
//
// Description
// List of corner points of a rectangle.
//
// Examples
// bbox = [1 2 3 4; 5 6 7 8];
// results = bbox2points(bbox);
//
// Authors
// Tanmay Chaudhari

    out=raw_bbox2points(rectangle);

endfunction
