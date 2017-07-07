function xyzrefinedPoints = bundleAdjustment(_3dpoints, imagepoints, visibility, cameramatrix, rotation, translation, distcoeffs)
// Refine camera poses and 3-d points
//
// Calling Sequence
// xyzrefinedPoints = bundleAdjustment(_3dpoints, imagepoints, visibility, cameramatrix, rotation, translation, distcoeffs)
//
// Parameters
// _3dpoints : N * 3 object points
// imagepoints : M * N * 2 image points for each camera and each points
// visibility : M * N * 1 visibility matrix, element[i][j] = 1 when object point i is visible from camera j and 0 if not
// cameramatrix : M * 3 * 3 camera matrix(intrinsic parameters) 3 * 3 camera matrix for each image
// rotation : M * 3 * 3 rotation matrix for each image
// translation : M * 3 * 1 translation matrix for each image
// distcoeffs : M * (4 * 5 * 8) * 1 distortion coefficient for each image
// xyzrefinedPoints : Refined N * 3 object points 
//
// Description
// The function returns the refined 3-D points. The refinement procedure uses Levenberg-Marquardt algorithm.
//
// Examples
// xyzrefinedPoints = bundleAdjustment(_3dpoints, imagepoints, visibility, cameramatrix, rotation, translation, distcoeffs)
//
// Authors
// Suraj Prakash

	
	xyzrefinedPoints = opencv_bundleAdjustment(_3dpoints, imagepoints, visibility, cameramatrix, rotation, translation, distcoeffs)
	
endfunction
