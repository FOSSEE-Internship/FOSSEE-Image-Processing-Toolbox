// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author:Diwakar Bhardwaj 
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [out]=IFFT(inputimage)
// Computes the inverse descrete fourier transform of image
//
// Calling Sequence
// output_image = IFFT(inputimage);
//
// Parameters
//
// inputimage : Grayscale image
// out_image  : IFFT of input image
//
// Description
// This function computes the inverse descrete fourier transform of input image.The image should be grayscale.
//
// Examples
// a = imread('images/lena.jpg',0);
// b = IFFT(a);
// imshow(b)
//
//Authors
//
//Diwakar Bhardwaj

[lhs, rhs] = argn(0);

 if rhs>1 then
 error(msprintf("Too many input arguments"));
 end
 if rhs<1 then
 error(msprintf("Not enough input arguments"));
 end
 if lhs >1
 error(msprintf("Too many output arguments"));
 end		


  inputimage1=mattolist(inputimage);
  a=raw_IFFT(inputimage1);
  dimension=size(a)
  for i = 1:dimension
      out(:,:,i)=a(i);
  end
endfunction;
