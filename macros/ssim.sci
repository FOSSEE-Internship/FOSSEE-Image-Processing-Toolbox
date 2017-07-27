// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Dhruti Shah
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function [ssim_val] = ssim(srcImg, reference)
// This function is used to compute the Structural Similarity Index (SSIM) for measuring image quality. 
//
// Calling Sequence
// [ssim_val] = ssim(srcImg, reference)
// 
// Parameters
// srcImg : The input image whose quality is to be measured. Must be the same size and class as reference.
// reference : Reference image against which quality if measured.
// ssim_val : Structural Similarity (SSIM) Index.
//
// Description
// Computes the Structural Similarity Index (SSIM) value.
//
// Examples
// // a simple example
// a = imread("/images/m1.jpeg");
// b = imread("/images/m2.jpeg");
// c = ssim(a,b);
//
// Authors
// Dhruti Shah          
	srcMat1 = mattolist(srcImg)
	srcMat2 = mattolist(reference)

	ssim_val = raw_ssim(srcMat1, srcMat2)
	 
endfunction
