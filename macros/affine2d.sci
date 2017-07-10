// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Yash S. Bhalgat
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function tform = affine2d(image)
	//Creates an affine2d object for input 3x3 matrix.
	//
	//Calling Sequence
	//y = affine2d(mat)
	//
	//Parameters
	//mat : It is a 3x3 matrix which specifies forward affine2d transformation.
	//y : an affine2d object with similar properties as the input.
	//
	//Description
	//y = affine2d(mat) returns the affine2d object where a 3x3 numeric matrix is given as input  
	//It encapsulates 2d affine geometri transformation.
	//
	//Examples
	//a=[1 2 0;3 4 0;5 1 1];
	//y=affine2d(a);
	//disp(y);
	//Authors
	//    Yash S. Bhalgat


	image_list = mattolist(image)

	out = raw_affine2d(image)

	sz = size(out)
	for i=1:sz
		tform(:, :, i) = out(i)
	end

endfunction
