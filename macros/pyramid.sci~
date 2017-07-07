// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Diwakar Bhardwaj 
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function [out]=pyramid(inputimage,direction,level)
// This function computes the pyramid reduction or expansion of input_image.
//
// Calling Sequence
// output_image=pyramid(inputimage,direction,level);
//
// Parameters
// inputimage : image matrix on which pyramid reduction or expansion has to be applied
// Direction  : An input string 'expand' for expand or 'reduce' for reduce the image matrix
// level      : It is specify number of times want to expand or reduce the dimension of an image matrix.
// output_image : Expanded or reduced form of image after pyramid reduction or expansio
//
// Description
// This function computes the pyramid reduction or expansion of input image.It uses  Gaussian pyramid to downsamples 
// and upsamples the image pixels.In reduction it     reduces the size of image (if m*n matrix image) by m/pow(2,level) 
// and n/pow(2,level).In expansion it expands the size of image (if m*n matrix image) by m*pow(2,level) 
// and n*pow(2,level) where pow(x,y)=x^y.
//
// Examples
// inputimage1 = imread('/images/color2.jpeg'); //read an image
// a=pyramid(inputimage1,'expand',2); //perform 'expand' operation
// imshow(a); //view the output
// b=pyramid(inputimage1,'reduce',4);
// imshow(b); //view the output
//
// Authors
// Diwakar Bhardwaj

        [ lhs rhs ] = argn(0)
		if lhs > 1 then
			error(msprintf("Too many output argument"))
		end
		
		if rhs > 3 then
			error(msprintf("Too many input arguments"))
		elseif rhs < 3
		    error(msprintf("Input arguments missing"))
		end	  

          inputimage1=mattolist(inputimage);
         a=raw_pyramid(inputimage1,direction,level);
         dimension=size(a)
         for i = 1:dimension
              out(:,:,i)=a(i);
         end
     
endfunction;
