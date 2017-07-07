function [ matrix ] = rotationVectorToMatrix(vector)
// Returns rotation matrix.
//
// Calling Sequence
//   matrix = rotationVectorToMatrix(vector);
//
// Parameters
// matrix: rotation matrix
// vector: 3-D rotation vector
//
// Description
// Converts rotation vector to rotation matrix.
//
// Examples
// vector = pi/4 * [1, 2, 3];
// matrix = rotationVectorToMatrix(vector);
//
// Authors
// Tanmay Chaudhari

         a=opencv_rotationVectorToMatrix(vector);
         matrix(:,:,1)=a(1);
	
endfunction
