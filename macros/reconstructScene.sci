// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the BSD.
// This source file is licensed as described in the file LICENSE, which
// you should have received as part of this distribution.  The terms
// are also available at
// https://opensource.org/licenses/BSD-3-Clause
// Author: Siddhant Narang
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function image = reconstructScene(Q, disp_mat, handleMissingValues)
// This function converts a 2-D image to a 3-D image.
//
// Calling Sequence
// reconstructScene(Q, disp_mat, handleMissingValues)
//
// Parameters
// Q(Depth Map): It is a matrix whose values define the different characteristics of a camera image.
// Disparity Map: Refers to the apparent pixel difference or motion between a pair of stereo image.
// handleMissingValues: Flag whose default value is false, which controls the handling of missing values.
// Returns: 3D image
//
// Description
// The function transforms a single-channel disparity map to a 3-channel image representing a 3D surface. 
// That is, for each pixel (x,y) and the corresponding disparity d = disparity(x,y).
// The matrix Q can be an arbitrary 4×4 matrix (for example, the one computed by stereoRectify). To reproject
// a sparse set of points {(x,y,d),...} to 3D space, use perspectiveTransform.
//
// Examples
// stacksize("max")
// img_1 = imread("images/left1.jpg", 0);
// img_2 = imread("images/right1.jpg", 0);
// w1 = genCheckerboardPoints([10, 7], 8);
// ip1 = detectCheckerboardCorner(img_1, [7, 10]);
// ip2 = detectCheckerboardCorner(img_2, [7, 10]);
// ip1l = list(ip1);
// ip2l = list(ip2);
// op = stereoCalibrateAndRect(w1, ip1l, ip2l, size(img_1));
// [map map1] = disparity(img_1, img_2);
// img = reconstructScene(op.DepthMap, map1, 1);
//
// See also
// imread
// genCheckerboardPoints
// detectCheckerboardCorne
// stereoCalibrateAndRect
// disparity
//
// Authors
// Siddhant Narang

[lhs rhs] = argn(0);

if rhs > 3 then
	error(msprintf("Too many input arguments"));
end
if rhs < 3 then
	error(msprintf("Not enough input arguments"));
end
if lhs > 1 then
	error(msprintf("Too many output arguments"));
end

_disp_mat = mattolist(disp_mat);

timage = raw_reconstructScene(Q, _disp_mat, handleMissingValues);

channel = size(timage);

for i = 1 : channel
	image(:, :, i) = timage(i);
end

endfunction
