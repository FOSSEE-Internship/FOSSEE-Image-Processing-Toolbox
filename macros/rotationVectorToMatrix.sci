// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Tanmay Chaudhari
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [ out ] = rotationVectorToMatrix(vector)
	// Returns rotation matrix.
	//
	// Calling Sequence
	//   matrix = rotationVectorToMatrix(vector);
	//
	// Parameters
	// matrix: rotation matrix
	// vector: 3-D rotation vector
	//
	// Description
	// Converts rotation vector to rotation matrix.
	//
	// Examples
	// vector = pi/4 * [1, 2, 3];
	// matrix = rotationVectorToMatrix(vector);
	//
	// Authors
	//  	Tanmay Chaudhari

         out=raw_rotationVectorToMatrix(vector);
	
endfunction
