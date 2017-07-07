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

function dstMat = imboxfilt3(srcImg, varargin)
// Blurs an image using the box filter.
//
// Calling Sequence
// blurredImg = imboxfilt3(srcImg)
// blurredImg = imboxfilt3(srcImg, filterWidth, filterHeight)
//
// Parameters
// srcImage: Input image.
// filterWidth: The width of the filter to be applied.
// filterHeight: The height of the filter to be applied.
// 
// Description
// The function smoothes an image using the kernel:
//
// <latex>
//  	K = $$\alpha $$\begin{bmatrix}
//   			1 & 1 & 1 & \dots & 1 & 1 \\
//    			1 & 1 & 1 & \dots & 1 & 1 \\
//    			$$\hdots                  \\
//    			1 & 1 & 1 & \dots & 1 & 1
//				$$\end{bmatrix}
// </latex>
//
//
// where
//
// <latex>
// 		$$\alpha = \begin{cases} $$\frac{1}{ksize.width * ksize.height} & \hspace{2mm} when\hspace{2mm}normalize = true \\ 1 & {otherwise}
//					\end{cases}$ 
// </latex>
//
//
// Unnormalized box filter is useful for computing various integral characteristics over each pixel 
// neighborhood, such as covariance matrices of image derivatives (used in dense optical flow algorithms,
// and so on). If you need to compute pixel sums over variable-size windows.
//
// Examples
// image = imread("images/lena.jpg");
// blurredImage = imboxfilt3(input_img);
//
// Examples
// image = imread("images/blob.jpg");
// blurredImage = imboxfilt3(input_img, 5, 5);
//
// See also
// imread 
//
// Authors
// Yash S. Bhalgat

	[lhs, rhs] = argn(0)
	
	srcMat = mattolist(srcImg)

	select rhs
		case 1 then
			out = raw_imboxfilt3(srcMat)
	
		case 2 then
			out = raw_imboxfilt3(srcMat, varargin(1))
			
		case 3 then
			out = raw_imboxfilt3(srcMat, varargin(1), varargin(2))
		end
	channel = size(out)
	
	for i = 1: channel
		dstMat(:,:,i) = out(i)
	end
	
endfunction
