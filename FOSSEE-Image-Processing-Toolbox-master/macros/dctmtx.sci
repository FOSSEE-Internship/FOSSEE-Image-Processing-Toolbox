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

function [out]=dctmtx(n)
// This function is used to calculate Discrete cosine transform matrix. 
//
// Calling Sequence
// D = dctmtx(n)
//
// Parameters
// A: The input scalar of double type.
// D : The n-by-n DCT (discrete cosine transform) matrix.. 
//
// Description
// D = dctmtx(n) returns the n-by-n DCT (discrete cosine transform) matrix. D*A is the DCT of the columns of A and D'*A is the inverse DCT of the columns of A (when A is n-by-n).


//
// Examples
// i = imread("rice.png",0); 
// mm = dctmtx(size(i,1));
// ir = mm*i*mm';
// imshow(ir);
// 


         out=raw_dctmtx(n);
     
endfunction;
