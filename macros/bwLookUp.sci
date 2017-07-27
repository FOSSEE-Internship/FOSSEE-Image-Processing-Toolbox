// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Samiran Roy
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function [out] = bwLookUp(image,lut)
// This function performs 2*2 and 3*3 nonlinear filtering using a lookup table.
//
// Calling Sequence
// [out] = bwLookUp(image,lut)
// 
// Parameters
// image : The input is a grayscale image. If the image is not binary, it is converted to one.
// lut : The lut is a 1*16 double vector [2*2 filtering], or a [1*512] double vector [3*3 filtering].
// out : The output image is the same size as image, same data type as lut.
//
// Description
// The function performs a 2-by-2 or 3-by-3 nonlinear neighborhood filtering operation on a grayscale image and returns the results in the output image. The neighborhood processing determines an integer index value used to access values in a lookup table 'lut'. The fetched lut value becomes the pixel value in the output image at the targeted position.
//
// Examples
// // a simple example
// a = imread("/images/lena.jpeg", 0);
// lut = [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
// b = bwLookUp(a,lut);
//
// Authors
// Samiran Roy          
	image1 = mattolist(image);
        
	a = raw_bwLookUp(image1,lut);
         
	dimension = size(a)
         
	for i = 1:dimension
        	out(:,:,i)=a(i);
        end

endfunction;
