function [idealPoints] = distortPoints(observedPoints, camMat, disCoefMat, alpha)
// Returns the ideal point coordinates from the observed point coordinates
//
// Calling Sequence
//   [idealPoints] = distortPoints(observedPoints, camMat, disCoefMat,alpha)
//
// Parameters
// observedPoints: 1xN or Nx1 2-channel (CV_32FC2 or CV_64FC2).
// camMat: \vecthreethree{f_x}{0}{c_x}{0}{f_y}{c_y}{0}{0}{1}
// disCoefMat: Input vector of distortion coefficients  (k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6]]) of 4, 5, or 8 elements. If the vector is NULL/empty, the zero  distortion coefficients are assumed.
// idealPoints: ideal point coordinates matrix. If matrix newCamMat is identity or omitted, idealPoints will contain normalized point coordinates.
//
// Description
// Returns the ideal points coordinates from the observed point coordinates after distortion and reverse perpective transformation. 
//
// Examples
// [observedPoints] = [1 2 3 4; 4 3 2 1]
// [camMat] = [450 0 231; 0 876.3 87.1; 0 0 1]
// [disCoefMat] = [3 2 0 9]
// alpha = 2
// [idealPoints] = distortPoints(observedPoints, camMat, disCoefMat,alpha)
//
// Authors
// Shreyash Sharma

	output = raw_distortPoints(observedPoints, camMat, disCoefMat, alpha)
    
         idealPoints = output
    		
    
endfunction
