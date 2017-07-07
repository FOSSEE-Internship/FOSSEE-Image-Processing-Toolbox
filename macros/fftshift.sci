// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Tess  Zacharias,Gursimar Singh
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [out]=fftshift(image)
//The function shifts the fourier transform of an image to its center.
//
//Calling Sequence
//[out]=fftshift(image)
//
//Parameters
//out - Output image same as size and depth of src image.
//image - Input image 
//
//Description
//The function shifts the fourier transform of an image to its center.The image is split into 4 parts and rearranged.
//
//Examples
//im=imread("images/lena.jpeg");
//img_out=fftshift(im);
//
//Authors
//Tess Zacharias,Gursimar Singh
//
//See also
//IDCT
//IFFT


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
     
     image1=mattolist(image);
     a=raw_fftshift(image1);
     dimension=size(a)
     for i = 1:dimension
          out(:,:,i)=a(i);
     end
     
endfunction;
