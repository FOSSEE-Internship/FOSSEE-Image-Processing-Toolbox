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

function new_image = sepFilter2D(image, ddepth, kernel_x, kernel_y, anchor_x, anchor_y, delta, border)
// Applies a separable linear filter to an image.
//
// Calling Sequence
// new_image = sepFilter2D(image, ddepth, kernel_x, kernel_y, anchor_x, anchor_y, delta, border);
//
// Parameters
// image: Input image.
// ddepth: Destination image depth, given below are the possible values<itemizedlist><listitem><para> CV_8U </para></listitem><listitem><para> CV_16U/CV_16S </para></listitem><listitem><para> CV_32F</para></listitem><listitem><para> CV_64F </para></listitem></itemizedlist>
// kernel_x: Coefficients for filtering each row.
// kernel_y: Coefficients for filtering each column.
// anchor_x: X-coordinate of the anchor.
// anchor_y: Y-coordinate of the anchor.
// delta: Value added to the filtered results before storing them.
// borderType: Pixel extrapolation method, given below are the possible argument values<itemizedlist><listitem><para> BORDER_CONSTANT 0 </para></listitem><listitem><para> BORDER_REPLICATE 1 </para></listitem><listitem><para> BORDER_REFLECT 2 </para></listitem><listitem><para> BORDER_WRAP 3 </para></listitem><listitem><para> BORDER_REFLECT_101 4 </para></listitem><listitem><para> BORDER_TRANSPARENT 5 </para></listitem> </itemizedlist>
// 
// Description
// The function applies a separable linear filter to the image. That is, first, every row of src
// is filtered with the 1D kernel kernelX.
// Then, every column of the result is filtered with the 1D kernel kernelY.
//
// Examples
// image = imread("images/lena.jpg");
// new_image = sepFilter2D(image, CV_8U, [1, 2, 1], [2, 1, 2], -1, -1, 2, 1)
//
// See also
// imread 
//
// Authors
// Sukul Bagai
	
	[lhs, rhs] = argn(0)
	if rhs > 8 then
		error(msprintf("Too many input arguments"))
	end
	if rhs < 8 then
		error(msprintf("Too less input arguments"))
	end	
	if lhs > 1 then
		error(msprintf("Too many output arguments"))
	end		

	image_list = mattolist(image)
	
	out = raw_sepFilter2D(image_list, ddepth, kernel_x, kernel_y, anchor_x, anchor_y, delta, border)
	
	sz = size(out)
	
	for i = 1 : sz
		new_image(:, :, i) = out(i)
	end
	
endfunction
