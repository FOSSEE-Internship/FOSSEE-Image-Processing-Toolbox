function fourierTransform=FFT(inputMatrix)
// This funtions returns discrete Fourier Transform of 2D input matrix
//
// Calling Sequence
// fourierTransform=FFT(inputMatrix);
//
// Parameters
// inputMatrix: Input matrix must be 2-D.
//
// Description
// It returns the 2D discrete Fourier transform of two dimensional input matrix.
//
// Examples
// I=imread("cameraman.tif");
// fourier=FFT(I);

	[rows cols channels]=size(inputMatrix);
	if channels <> 1 then
		error(msprintf("Wrong input, input must be 2-D matrix"));
	end
	fourierTransform=opencv_FFT(inputMatrix);
endfunction
