// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Gursimar Singh
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function outImg= imlincomb(x1,A1,varargin)
//Blend two or more images 
//
//Calling Sequence
//outImg= imlincomb(x1,A1);
//outImg= imlincomb(x1,A1,A2,A2,x3,A3,x4,A4........,xN,AN);
//
//Parameters
//outImg:Output combined image.
//xN:Input multiplication factor.The multiplication factor and the sum of all the mutiplication factors should be less than 1.  
//AN:Input image
//
//Description
//This function returns a linear combination of the input images.
//
//Examples
//im1=imread('images/balls.jpg');
//im2=imread('images/lena.jpeg');
//img=imlincomb(0.5,im1,05,im2);
//
//Authors
//Gursimar Singh
//
//See also
//imimposemin
//imadd

[lhs rhs] = argn(0);
    
   if rhs<1 then
	   error(msprintf("Not enough input arguments"));
	 end
	 if lhs >1
	   error(msprintf("Too many output arguments"));
   end
     
     if modulo(rhs-1,2) == 0 then
      	error(msprintf("Number of input arguments must be even"));
     end
     out=x1*A1;     
     if rhs>2
        for i=1:rhs/2 -1          
           A=varargin(2*i); 
           x=varargin(2*i-1);    
           B=x*A;
           out=imadd(out,B);
        end
     end

     outImg=out;
endfunction               


