function [dstImg] =	DCT(srcImg)
// Performs forward Discrete Cosine Transform of the 1D or 2D array. 
//
// Calling Sequence
// [dstMat] = DCT(srcMat)
//
// Parameters
// srcMat = 1D or 2D floating type array
// dstMat = The output matrix
//
// Description
// Returns the forward transform of the input matrix. It uses the flag to perform the transform. The flag is 0 by default so as to perform forward transform.
//
// Examples
// [srcMat] = [230.3 23.1 432.5; 321 543.1 89.5]
// [dstMAt] = DCT(srcMat)
//
// Bibliography
//   Literature references one pr. line
//
// code for the above function
//

	srcMat = mattolist(srcImg)
	output = opencv_DCT(srcMat)
	
	channels = size(output)
	
	for i = 1:channels 		// for i channel image
		output_image(:,:,i) = output(i)
	end	
endfunction

