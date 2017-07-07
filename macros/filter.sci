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

function out = filter(input_image, depth, kernelMatrix, anchor_x, anchor_y, delta)
// Convolves an image with the kernel.
//
// Calling Sequence
//  img = filter(input_image, depth, kernelMatrix, -1, -1, 1);
//
// Parameters
// input_image: Input image.
// depth: Destination image depth, given below are the possible values-<itemizedlist><para><listitem> CV_8U </listitem></para><para><listitem> CV_16U/CV_16S </listitem></para><para><listitem> CV_32F </listitem></para><para><listitem> CV_64F </listitem></para></itemizedlist>
// kernelMatrix: The matrix which defines the kernel of the filter to be applied on the input image.
// anchor_x: X-coordinate of the anchor.
// anchor_y: Y-coordinate of the anchor.
// delta: Value added to the filtered results before storing them.
//
// Description
// The function applies an arbitrary linear filter to an image. In-place operation is supported.
// When the aperture is partially outside the image, the function interpolates outlier pixel values
// according to the specified border mode.
// The function does actually compute correlation, not the convolution:
// <latex>
//	dst(x, y) = $$\sum_{0 < x' < kernel.cols}_{0 < y' < kernel.rows} kernel(x', y') * src(x + x' - anchor.y + y'- anchor.y)
// </latex>
//
// Examples
// img1 = imread("images/right1.jpg", 0);
// img2 = imread("images/left1.jpg", 0);
// image = imabsdiff(img1, img2);
//
// See also
// imread
//
// Authors
// Sukul Bagai

	input_image1 = mattolist(input_image);
	a = raw_filter2D(input_image1, depth, kernel_matrix, anchor_x, anchor_y, delta);
	dimension = size(a)

	for i = 1: dimension
	  out(:, :, i) = a(i);
	end
endfunction
