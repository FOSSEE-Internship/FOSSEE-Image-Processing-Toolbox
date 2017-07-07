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
//
function structuring_element = getStructuringElement(gettype, cols, rows, anchorX, anchorY)
// This function returns a structuring element required for the morphological operations.
// Calling Sequence
// se = raw_getStructuringElement(gettype, cols, rows, anchorX, anchorY)
//
// Parameters
// se: output structuring element matrix
// shape: element shape that could be one of the following-
//
//MORPH_RECT - a rectangular structuring element
//MORPH_ELLIPSE - an elliptic structuring element
//MORPH_CROSS - a cross-shaped structuring element
//
//cols: Width of the structuring element
//rows: Height of the structuring element
//anchor: Anchor position within the element. The value  (-1, -1) means that the anchor is at the center. 
//Only the shape of a cross-shaped element depends on the anchor position. In other cases the anchor just regulates 
//how much the result of the morphological operation is shifted.
//anchor_x: x-coordinate of the anchor
//anchor_y: y-coordinate of the anchor
//
// Description
// The function constructs and returns the structuring element that can be further passed to 
// function that perform morphological operations like erode or dilate. 
//
// Examples
// src = imread("/images/color2.jpeg");
// se1=getStructuringElement('MORPH_RECT',5,7,3,4); //make a rectangular structuring element
// out  = dilate(src,se1,3,4,1); //perform dilate morphological operation
// imshow(out); //view the output image
// 
// Examples
// src = imread("../images/color2.jpeg");
// se2=getStructuringElement('MORPH_ELLIPSE',10,15,2,2);  //make an elliptical structuring element
// out  = dilate(src,se2,2,2,2); //perform dilate morphological operation
// imshow(out); //view the output image
//
// Authors
// Sukul Bagai
	
    	[ lhs rhs ] = argn(0)
		if lhs > 1 then
			error(msprintf("Too many output argument"))
		end
		
		if rhs > 5 then
			error(msprintf("Too many input arguments"))
		elseif rhs < 5 then
		    error(msprintf("input arguments missing"))
		end	

	
	
	structuring_element = raw_getStructuringElement(gettype, cols, rows, anchorX, anchorY)
	
endfunction
