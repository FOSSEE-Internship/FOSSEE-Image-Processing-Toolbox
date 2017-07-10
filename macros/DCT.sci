// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Deepshikha
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [dstImg] =	DCT(srcImg)
	  //Performs forward Discrete Cosine Transform of the 1D or 2D array. 
	  //
	  //Calling Sequence
	  //dstMat = DCT(srcMat)
	  //
	  //Parameters
	  //srcMat : 1D or 2D floating type array
	  //dstMat : The output matrix 
	  //
	  //Description
	  //dstMat = DCT(srcMat)
	  //Returns the DCT of the input matrix.
	  //
	  //Examples
	  //srcMat = [230.3 23.1 432.5; 321 543.1 89.5]
	  //dstMAt = DCT(srcMat)
	  //disp(dstMAt)
	  //Authors
	  //    Deepshikha

	srcMat = mattolist(srcImg)
	output = raw_DCT(srcMat)
	
	channels = size(output)
	
	for i = 1:channels 		// for i channel image
		dstImg(:,:,i) = output(i)
	end	
endfunction