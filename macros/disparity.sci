// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Siddhant Narang
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [map, map1] = disparity(img1, img2, varargin)
// This function returns the disparity map between two images which
// are photos of the same image taken at two different angles.
// 
// Calling Sequence
// map, map1 = disparity(img1, img2)
// map, map1 = disparity(img1, img2, Name(same as the ones given under parameters), Value, ....)
//
// Parameters
// method:  Disparity estimation algorithm, specified as the comma-separated pair consisting of 'Method' 
// and character vector 'blockMatching' or 'semi-blockMatching'.
// numDisparities: Maximum disparity minus minimum disparity. This parameter must be divisible by 16.
// SADWindowSize: Matched block size. It must be an odd number >=1 
// uniquenessRatio: Margin in percentage by which the best (minimum) computed cost function value should 
// “win” the second best value to consider the found match correct. Normally, a value within the 5-15 
// range is good enough.
// textureThreshold: This controls the detailing in the disparity map.
// disp12MaxDiff:  Maximum allowed difference (in integer pixel units) in the left-right disparity check.
// Returns: disparity map
//
// Description
// Validates disparity using the left-right check. The matrix "cost" should be computed by the 
// stereo correspondence algorithm.
//
// Examples
// stacksize("max");
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
// reconstructScene
//
// Authors
// Siddhant Narang

	img_list1 = mattolist(img1);
	img_list2 = mattolist(img2);

	[lhs rhs] = argn(0);

	if rhs > 14 then
		error(msprintf("Too many input arguments"));
	end
	if rhs < 2 then
		error(msprintf("Not enough input arguments"));
	end
	if lhs > 2 then
		error(msprintf("Too many output arguments"));
	end

	if rhs == 2 then
		[tmap, tmap1] = raw_disparity(img_list1, img_list2);
	end
	if rhs == 4 then
		[tmap, tmap1] = raw_disparity(img_list1, img_list2, varargin(1), varargin(2));
	end
	if rhs == 6 then
		[tmap, tmap1] = raw_disparity(img_list1, img_list2, varargin(1), varargin(2), varargin(3), varargin(4));
	end
	if rhs == 8 then
		[tmap, tmap1] = raw_disparity(img_list1, img_list2, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6));
	end
	if rhs == 10 then
		[tmap, tmap1] = raw_disparity(img_list1, img_list2, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(8));
	end
	if rhs == 12 then
		[tmap, tmap1] = raw_disparity(img_list1, img_list2, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(8), varargin(9), varargin(10));
	end
	if rhs == 14 then
		[tmap, tmap1] = raw_disparity(img_list1, img_list2, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(8), varargin(9), varargin(10), varargin(11), varargin(12));
	end

	channel = size(tmap);

	for i = 1 : channel
		map(:, :, i) = tmap(i);
	end

	channel = size(tmap1);

	for i = 1 : channel
		map1(:, :, i) = tmap1(i);
	end
endfunction