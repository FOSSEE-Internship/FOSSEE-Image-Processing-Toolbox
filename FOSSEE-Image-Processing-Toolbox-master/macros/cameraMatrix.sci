function [ camMatrix ] = cameraMatrix(instrinsicMatrix,rotationMatrix,translationVector)
// Returns camera projection matrix.
//
// Calling Sequence
//   camMatrix = cameraMatrix(instrinsicMatrix,rotationMatrix,translationVector);
//
// Parameters
// camMatrix: A 4x3 camera projection matrix, which can be used to project 3-D world points in homogenous co-ordinates into an image.
// instrinsicMatrix: Projection matrix of size 3x3.
// rotationMatrix: A 3x3 matrix which specifies the rotation of a camera.
// translationVector: A 1x3 vector which specifies the translation of a camera.
//
// Description
// Return a matrix of size 4-by-3, which can be used to project 3-D world points in homogenous co-ordinates into an image.
//
// Examples
// instrinsicMatrix = [1 0 0; 1 2 0; 3 4 0];
// rotationMatrix = [   0.1417   -0.7409    0.6565; 0.9661   -0.0410   -0.2548;    0.2157    0.6703    0.7100];
// translationVector = [ -29.2584   35.7824  725.5824];
// camMatrix = cameraMatrix(instrinsicMatrix,rotationMatrix,translationVector)
//
// Authors
// Tanmay Chaudhari

        a=opencv_cameraMatrix(instrinsicMatrix,rotationMatrix,translationVector);
        camMatrix(:,:,1)=a(1);
	
endfunction
