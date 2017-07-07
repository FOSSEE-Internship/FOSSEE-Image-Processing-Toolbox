// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Shreyash Sharma
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [out]=impixel(image,value1,value2)
// This function is used to extract pixel color values. 
//
// Calling Sequence
// B = impixel(A,value1,value2)
//
// Parameters
// A: image matrix of the source image.
// B : The pixel values in the specified coordinates in the form of a matrix of double type. 
// value1 :  The column c indices of the pixels to extract.
// value2 :  The row r indices of the pixels to extract.
//
// Description
// impixel(I) returns the value of pixels in the specified image I, where I can be a grayscale, binary, or RGB image. impixel displays the image specified and waits for you to select the pixels in the image using the mouse. If you omit the input arguments, impixel operates on the image in the current axes.P = impixel(I,c,r) returns the values of pixels specified by the row and column vectors r and c. r and c must be equal-length vectors. The kth row of P contains the RGB values for the pixel (r(k),c(k)).
//
// Examples
// i1 = imread('lena.jpeg');
// a = [1 2 3]
// b = [1 2 3] 
// C=impixel(A,a,b)
// imshow(i2);
//

        
         image1=mattolist(image);
         out=raw_impixel(image1,value1,value2);
         
endfunction;
