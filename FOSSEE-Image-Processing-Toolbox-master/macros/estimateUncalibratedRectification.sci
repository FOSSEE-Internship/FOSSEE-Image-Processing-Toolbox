function [T1,T2]=estimateUncalibratedRectification(fundamentalMatrix,inlierPoints1,inlierPoints2,imageSize)
// It returns projective transformation for rectifying stereo images 
//
// Calling Sequence
//   [T1,T2]=estimateUncalibratedRectification((fundamentalMatrix,inlierPoints1,inlierPoints2,imageSize);
//
// Parameters
// fundamentalMatrix: 3-by-3 fundamental matrix.
// inlierPoints1: M-by-2 matrix 
// inlierPoints2: M-by-2 matrix
// imageSize: 2 element vector, indicating size of the second image.
//
// Description
// estimateUncalibratedRectification gives transformation of both images as 3-by-3 matrices
//
// Examples
// I=imread("lena_left.jpeg");
// cornerPoints1=detectMinEigenFeatures(I);
// I2=imread("lena_right.jpeg");
// cornerPoints2=detectMinEigenFeatures(I2);
// fundamentalMat=(cornerPoints1.Location,cornerPoints2.Location);
// [T1, T2]=estimateUncalibratedRectification(fundamentalMat,cornerPoints1.Location,cornerPoints2.Location,size(I2));

    [lhs rhs]=argn(0);
    if lhs>2
         error(msprintf(" Too many output arguments"));
    elseif rhs<2
        error(msprintf(" More input arguments required"));
    elseif rhs>4
        error(msprintf(" Too many input arguments"));
    elseif modulo(rhs,2)==0
       error(msprintf("Either Argument Name or its Value missing"));
    end
    [T1 T2]=opencv_estimateUncalibratedRectification(fundamentalMatrix,inlierPoints1,inlierPoints2,imageSize);
endfunction
