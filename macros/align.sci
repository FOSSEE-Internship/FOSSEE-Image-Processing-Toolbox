// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Ashish Manatosh Barik 
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
//
function [outImg1, outImg2, outImg3, varargout] = align(maxBits, excludeRange, cut, num, srcImg1, srcImg2, srcImg3, varargin)
// This function aligns the set of input images for HDR image creation.
//
// Calling Sequence 
// [out1, out2, out3] = align(maxBits, excludeRange, cut, num, srcImg_1, srcImg_2, srcImg_3)
// [out1, out2, out3, out4] = align(maxBits, excludeRange, cut, num, srcImg1, srcImg_2, srcImg_3, srcImg_4)
// [out1, out2, out3, out4, out5] = align(maxBits, excludeRange, cut, num, srcImg_1, srcImg_2, srcImg_3, srcImg_4, srcImg_5)
//
// Parameters 
// maxBits : Logarithm to the base 2 of maximal shift in each dimension. Values of 5 and 6 are usually good enough (31 and 63 pixels shift respectively). Value should not exceed 6. It is of Double type.
// excludeRange : Range for exclusion bitmap that is constructed to suppress noise around the median value. It is of Double type.
// cut : If true, cuts images. Otherwise fills the new regions with zeros. It is of Boolean type.
// num : Number of images given as input source images(3 - 5). It is of double type.
// srcImg_i : Hypermat of image_i.
//
// Description
// This function uses AlignMTB algorithm which converts images to median threshold bitmaps (1 for pixels brighter than median luminance and 0 otherwise) and than aligns the resulting bitmaps using bit operations.
//
// Examples
// a = imread("/images/m1.jpeg");
// b = imread("/images/m2.jpeg");
// c = imread("/images/m3.jpeg");
// num = 3;
// maxBits= 6;
// excludeRange = 4;
// cut = %t;
// [x, y, z] = align(maxBits, excludeRange, cut, num, a, b, c);
//
// Examples
// a = imread("/images/t1.jpeg");
// b = imread("/images/t2.jpeg");
// c = imread("/images/t3.jpeg");
// d = imread("/images/t4.jpeg");
// num = 4;
// maxBits= 6;
// excludeRange = 4;
// cut = %f;
// [x, y, z, p] = align(maxBits, excludeRange, cut, num, a, b, c, d);
//
// Examples
// // error cause maxBits value is greater than 6
// a = imread("/images/m1.jpeg");
// b = imread("/images/m2.jpeg");
// c = imread("/images/m3.jpeg");
// d = imread("/images/m4.jpeg");
// num = 4;
// maxBits= 7;
// excludeRange = 4;
// cut = %t;
// [x, y, z, p] = align(maxBits, excludeRange, cut, num, a, b, c, d);
//
// Examples
// a = imread("/images/m1.jpeg");
// b = imread("/images/m2.jpeg");
// c = imread("/images/m3.jpeg");
// d = imread("/images/m4.jpeg");
// e = imread("/images/m5.jpeg");
// num = 5;
// maxBits= 6;
// excludeRange = 4;
// cut = %t;
// [x, y, z, p, q] = align(maxBits, excludeRange, cut, num, a, b, c, d, e);
//
// Examples
// // cut is set false here (if true cuts images, otherwise fills the new regions with zeros. )
// a = imread("/images/t1.jpeg");
// b = imread("/images/t2.jpeg");
// c = imread("/images/t3.jpeg");
// num = 3;
// maxBits= 1;
// excludeRange = 4;
// cut = %t;
// [x, y, z] = align(maxBits, excludeRange, cut, num, a, b, c);
// 
// Examples
// // aligns the images for the making of 1 HDR image
// a = imread("/images/m1.jpeg");
// b = imread("/images/m2.jpeg");
// c = imread("/images/m3.jpeg");
// num = 3;
// maxBits= 5;
// excludeRange = 4;
// cut = %t;
//[x, y, z] = align(maxBits, excludeRange, cut, num, a, b, c);
//
// Examples
// // aligns the images for the making of 1 HDR image
// a = imread("/images/t1.jpeg");
// b = imread("/images/t2.jpeg");
// c = imread("/images/t3.jpeg");
// num = 3;
// maxBits= 5;
// excludeRange = 4;
// cut = %t;
// [x, y, z] = align(maxBits, excludeRange, cut, num, a, b, c);
//
// Examples
// // aligns the images for the making of 1 HDR image
// a = imread("/images/m1.jpeg");
// b = imread("/images/m2.jpeg");
// c = imread("/images/m3.jpeg");
// num = 3;
// maxBits= 5;
// excludeRange = 3;
// cut = %f;
// [x, y, z] = align(maxBits, excludeRange, cut, num, a, b, c);
//
// Examples
// // maxBits = 6, leads to noticeable pixel shift
// a = imread("/images/t1.jpeg");
// b = imread("/images/t2.jpeg");
// c = imread("/images/t3.jpeg");
// d = imread("/images/t4.jpeg");
// num = 4;
// maxBits= 5;
// excludeRange = 5;
// cut = %t;
// [x, y, z, p] = align(maxBits, excludeRange, cut, num, a, b, c, d);
//
// Examples
// // maxBits = 6, leads to noticeable pixel shift
// a = imread("/images/t1.jpeg");
// b = imread("/images/t2.jpeg");
// c = imread("/images/t3.jpeg");
// num = 3;
// maxBits= 5;
// excludeRange = 6;
// cut = %f;
// [x, y, z, p] = align(maxBits, excludeRange, cut, num, a, b, c, d);
//
// Authors
// Ashish Manatosh Barik, NIT Rourkela
//
	srcMat1 = mattolist(srcImg1)
	srcMat2 = mattolist(srcImg2)
	srcMat3 = mattolist(srcImg3)

	[lhs, rhs] = argn(0)

	select rhs
		case 7 then
			[out1, out2, out3] = raw_align(maxBits, excludeRange, cut, num, srcMat1, srcMat2, srcMat3)

			channels1 = size(out1)
			channels2 = size(out2)
			channels3 = size(out3)

			for i = 1:channels1
				outImg1(:, :, i) = (out1(i))
			end

			for j = 1:channels2
				outImg2(:, :, j) = (out2(j))
			end

			for k = 1:channels3
				outImg3(:, :, k) = (out3(k))
			end
			
		case 8 then
			srcMat4 = mattolist(varargin(1))

			[out1, out2, out3, out4] = raw_align(maxBits, excludeRange, cut, num, srcMat1, srcMat2, srcMat3, srcMat4)

			channels1 = size(out1)
			channels2 = size(out2)
			channels3 = size(out3)
			channels4 = size(out4)

			for i = 1:channels1
				outImg1(:, :, i) = (out1(i))
			end

			for j = 1:channels2
				outImg2(:, :, j) = (out2(j))
			end

			for k = 1:channels3
				outImg3(:, :, k) = (out3(k))
			end

			for l = 1:channels4
				outImg4(:, :, l) = (out4(l))
			end

			varargout(1) = outImg4
	


		case 9 then
			srcMat4 = mattolist(varargin(1))
			srcMat5 = mattolist(varargin(2))

			[out1, out2, out3, out4, out5] = raw_align(maxBits, excludeRange, cut, num, srcMat1, srcMat2, srcMat3, srcMat4, srcMat5)
			
			channels1 = size(out1)
			channels2 = size(out2)
			channels3 = size(out3)
			channels4 = size(out4)
			channels5 = size(out5)

			for i = 1:channels1
				outImg1(:, :, i) = (out1(i))
			end

			for j = 1:channels2
				outImg2(:, :, j) = (out2(j))
			end

			for k = 1:channels3
				outImg3(:, :, k) = (out3(k))
			end

			for l = 1:channels4
				outImg4(:, :, l) = (out4(l))
			end		
			
			varargout(1) = outImg4

			for m = 1:channels5
				outImg5(:, :, m) = (out5(m))
			end

			varargout(2) = outImg5

		case 10 then
			srcMat4 = mattolist(varargin(1))
			srcMat5 = mattolist(varargin(2))
			srcMat6 = mattolist(varargin(3))

			[out1, out2, out3, out4, out5, out6] = raw_align(maxBits, excludeRange, cut, num, srcMat1, srcMat2, srcMat3, srcMat4, srcMat5, srcMat6)
			
			channels1 = size(out1)
			channels2 = size(out2)
			channels3 = size(out3)
			channels4 = size(out4)
			channels5 = size(out5)
			channels6 - size(out6)

			for i = 1:channels1
				outImg1(:, :, i) = (out1(i))
			end

			for j = 1:channels2
				outImg2(:, :, j) = (out2(j))
			end

			for k = 1:channels3
				outImg3(:, :, k) = (out3(k))
			end

			for l = 1:channels4
				outImg4(:, :, l) = (out4(l))
			end		
			
			varargout(1) = outImg4

			for m = 1:channels5
				outImg5(:, :, m) = (out5(m))
			end

			varargout(2) = outImg5

			for n = 1:channels6
				outImg6(:, :, n) = (out6(n))
			end

			varargout(3) = outImg6
	end

endfunction
