// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Sukul Bagai
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function imwrite(img, pstData)
// Writes to a specified location.
//
// Calling Sequence
// new_image = imwrite(srcImg, location)
//
// Parameters
// srcImg: input image.
// location: The relative path of where you want to write the image.
// 
// Description
// This function is used write the image to a specified path
//
// Examples
// image = imread("images/lena.jpg");
// imwrite(image, "./imageName.jpg"); // imageName -> name of the image
//
// See also
// imread
//
// Authors
// Sukul Bagai
	
	[lhs rhs] = argn(0)
	if rhs < 2
		error(msprintf("Not enough input arguments"));
	elseif rhs > 2
		error(msprintf("Too many input arguments"));
	end
	image = mattolist(img)
	raw_imwrite(image, pstData)
endfunction
