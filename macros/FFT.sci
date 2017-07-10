// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Sridhar Reddy
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function fourierTransform=FFT(inputMatrix)
	// This function returns discrete Fourier Transform of 2D input matrix
	//
	// Calling Sequence
	// fourierTransform=FFT(inputMatrix)
	//
	// Parameters
	// inputMatrix: Input matrix must be 2-D.
	//
	// Description
	// It returns the 2D discrete Fourier transform of two dimensional input matrix.
	//
	// Examples
	// I=imread('images/lena.jpeg',0);
	// I=double(I);
	// fourier=FFT(I)
	// imshow(fourier)
	//Authors
	//    Sridhar Reddy

	[rows cols channels]=size(inputMatrix);
	if channels <> 1 then
		error(msprintf("Wrong input, input must be 2-D matrix"));
	end
	fourierTransform=raw_FFT(inputMatrix);
endfunction
