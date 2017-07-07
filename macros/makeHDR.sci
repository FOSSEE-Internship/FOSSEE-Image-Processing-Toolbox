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
function [outHDR, outLDR] = makeHDR(typeOfMethod, num, varargin)
// This function is used to create HDR image.
//
// Calling Sequence
// [out1, out2] = makeHDR(typeOfMethod=1, num=3, srcMat_1, srcMat_2, srcMat_3, ex_1, ex_2, ex_3, max_iter, threshold) // Robertson merging
// [out1, out2] = makeHDR(typeOfMethod=2, num=3, srcMat_1, srcMat_2, srcMat_3, ex_1, ex_2, ex_3, samples, lambda, random) // Debevec merging
// [out1, out2] = makeHDR(typeOfMethod=3, num=3, srcMat_1, srcMat_2, srcMat_3, ex_1, ex_2, ex_3, contrast_weight, saturation_weight, exposure_weight) // Mertens merging
// [out1, out2] = makeHDR(typeOfMethod=1, num=4, srcMat_1, srcMat_2, srcMat_3, srcMat_4, ex_1, ex_2, ex_3, ex_4, max_iter, threshold) // Robertson merging
// [out1, out2] = makeHDR(typeOfMethod=2, num=4, srcMat_1, srcMat_2, srcMat_3, srcMat_4, ex_1, ex_2, ex_3, ex_4, samples, lambda, random) // Debevec merging
// [out1, out2] = makeHDR(typeOfMethod=3, num=4, srcMat_1, srcMat_2, srcMat_3, srcMat_4, ex_1, ex_2, ex_3, ex_4, contrast_weight, saturation_weight, exposure_weight) // Mertens merging
// [out1, out2] = makeHDR(typeOfMethod=1, num=5, srcMat_1, srcMat_2, srcMat_3, srcMat_4, srcMat_5, ex_1, ex_2, ex_3, ex_4, ex_5, max_iter, threshold) // Robertson merging
// [out1, out2] = makeHDR(typeOfMethod=2, num=5, srcMat_1, srcMat_2, srcMat_3, srcMat_4, srcMat_5, ex_1, ex_2, ex_3, ex_4, ex_5, samples, lambda, random) // Debevec merging
// [out1, out2] = makeHDR(typeOfMethod=3, num=5, srcMat_1, srcMat_2, srcMat_3, srcMat_4, srcMat_5, ex_1, ex_2, ex_3, ex_4, ex_5, contrast_weight, saturation_weight, exposure_weight) // Mertens merging
// [out1, out2] = makeHDR(typeOfMethod=1, num=6, srcMat_1, srcMat_2, srcMat_3, srcMat_4, srcMat_5, srcMat_6, ex_1, ex_2, ex_3, ex_4, ex_5, ex_6, max_iter, threshold) // Robertson merging
// [out1, out2] = makeHDR(typeOfMethod=2, num=6, srcMat_1, srcMat_2, srcMat_3, srcMat_4, srcMat_5, srcMat_6, ex_1, ex_2, ex_3, ex_4, ex_5, ex_6, samples, lambda, random) // Debevec merging
// [out1, out2] = makeHDR(typeOfMethod=3, num=6, srcMat_1, srcMat_2, srcMat_3, srcMat_4, srcMat_5, srcMat_6, ex_1, ex_2, ex_3, ex_4, ex_5, ex_6, contrast_weight, saturation_weight, exposure_weight) // Mertens merging
//
// Parameters 
// typeOfMethod : Use '1' for 'Robertson', '2' for 'Debevec', or '3' for 'Mertens'.
// num : It is the number of images being fed as input. It is of Double type.
// srcMat_i : It is the hypermat of input source image.
// ex_i : It is the exposure value of the corresponding image_i. It is of double type.
// max_iter : (Robertson) maximal number of Gauss-Seidel solver iterations. It is of Double type.
// threshold : (Robertson) target difference between results of two successive steps of the minimization. It is of Double type.
// samples : (Debevec) number of pixel locations to use. It is of Double type.
// lambda : (Debevec) smoothness term weight. Greater values produce smoother results, but can alter the response. It is of Double type.
// random : (Debevec) if true sample pixel locations are chosen at random, otherwise they form a rectangular grid. It is of Boolean type.
// contrast_weight : (Mertens) contrast measure weight. It is of Double type.
// saturation_weight : (Mertens) saturation measure weight. It is of Double type.
// exposure_weight : (Mertens) well-exposedness measure weight. It is of Double type.
// out1 : HDR image 
// out2 : LDR image    
//
// Description
// This function takes a set of images of the same scene in different exposures which have been aligned accordingly and outputs the HDR image.
//
// Examples
// // input of 3 images(min), using Robertson merging technique
// a = imread("/images/t1.jpeg");
// b = imread("/images/t2.jpeg");
// c = imread("/images/t3.jpeg");
// num = 3;
// typeOfMethod = 1;
// ex1 = 15;
// ex2 = 2.5;
// ex3 = 0.25;
// maxIter = 30;
// thres = 0.01;
//[hdr, ldr] = makeHDR(typeOfMethod, num, a, b, c, ex1, ex2, ex3, maxIter, thres);
//
// Examples
// // Use of Debevec merging technique
// a = imread("/images/m1.jpeg");
// b = imread("/images/m2.jpeg");
// c = imread("/images/m3.jpeg");
// d = imread("/images/m4.jpeg");
// e = imread("/images/m5.jpeg");
// f = imread("/images/m6.jpeg");
// num = 6;
// typeOfMethod = 2;
// ex1 = 0.0167;
// ex2 = 0.034;
// ex3 = 0.067;
// ex4 = 0.125;
// ex5 = 0.25;
// ex6 = 0.5;
// samples = 70;
// lambda = 10.0;
// random = %f; 
// [hdr, ldr] = makeHDR(typeOfMethod, num, a, b, c, d, e, f, ex1, ex2, ex3, ex4, ex5, ex6, samples, lambda, random);
// 
// Examples
// // use of Robertson merging technique
// a = imread("/images/m1.jpeg");
// b = imread("/images/m2.jpeg");
// c = imread("/images/m3.jpeg");
// d = imread("/images/m4.jpeg");
// e = imread("/images/m5.jpeg");
// f = imread("/images/m6.jpeg");
// num = 6;
// typeOfMethod = 1;
// ex1 = 0.0167;
// ex2 = 0.034;
// ex3 = 0.067;
// ex4 = 0.125;
// ex5 = 0.25;
// ex6 = 0.5;
// maxIter = 30;
// thres = 0.01;
// [hdr, ldr] = makeHDR(typeOfMethod, num, a, b, c, d, e, f, ex1, ex2, ex3, ex4, ex5, ex6, maxIter, thres); 
//
// Examples
// // alternative to creating an HDR image, resulting image is of average exposure. Faster compared to rendering a HDR image.
// a = imread("/images/m1.jpeg");
// b = imread("/images/m2.jpeg");
// c = imread("/images/m3.jpeg");
// d = imread("/images/m4.jpeg");
// e = imread("/images/m5.jpeg");
// f = imread("/images/m6.jpeg");
// num = 6;
// typeOfMethod = 3;
// ex1 = 0.0167;
// ex2 = 0.034;
// ex3 = 0.067;
// ex4 = 0.125;
// ex5 = 0.25;
// ex6 = 0.5;
// contrastWeight = 1.0;
// saturationWeight = 1.0;
//exposureWeight = 0.0; 
//[hdr, ldr] = makeHDR(typeOfMethod, num, a, b, c, d, e, f, ex1, ex2, ex3, ex4, ex5, ex6, contrastWeight, saturationWeight, exposureWeight);
//
// Examples
// a = imread("/images/i1.jpeg");
// b = imread("/images/i2.jpeg");
// c = imread("/images/i3.jpeg");
// d = imread("/images/i4.jpeg");
// num = 4;
// typeOfMethod = 2;
// ex1 = 0.034;
// ex2 = 0.008;
// ex3 = 0.0034;
// ex4 = 0.00073;
// samples = 70;
// lambda = 10.0;
// random = %f; 
//[hdr, ldr] = makeHDR(typeOfMethod, num, a, b, c, d, ex1, ex2, ex3, ex4, samples, lambda, random);
//
// Examples
// a = imread("/images/i1.jpeg");
// b = imread("/images/i2.jpeg");
// c = imread("/images/i3.jpeg");
// d = imread("/images/i4.jpeg");
// num = 4;
// typeOfMethod = 1;
// ex1 = 0.034;
// ex2 = 0.008;
// ex3 = 0.0034;
// ex4 = 0.00073;
// maxIter = 30;
// thres = 0.01;
// [hdr, ldr] = makeHDR(typeOfMethod, num, a, b, c, d, ex1, ex2, ex3, ex4, maxIter, thres);
//
// Examples
// a = imread("/images/i1.jpeg");
// b = imread("/images/i2.jpeg");
// c = imread("/images/i3.jpeg");
// d = imread("/images/i4.jpeg");
// num = 4;
// typeOfMethod = 3;
// ex1 = 0.034;
// ex2 = 0.008;
// ex3 = 0.0034;
// ex4 = 0.00073;
// maxIter = 30;
// contrastWeight = 1.0;
// saturationWeight = 1.0;
//exposureWeight = 0.0; 
// [hdr, ldr] = makeHDR(typeOfMethod, num, a, b, c, d, ex1, ex2, ex3, ex4, contrastWeight, saturationWeight, exposureWeight);
//
// Authors 
// Ashish Manatosh Barik, NIT Rourkela
//
	[lhs, rhs] = argn(0)

	select rhs
		case 10 then
			for i = 1:num
				varargin(i) = mattolist(varargin(i))
			end

			[out1, out2] = raw_makeHDR(typeOfMethod, num, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(8))

		case 11 then
			for i = 1:num
				varargin(i) = mattolist(varargin(i))
			end

			[out1, out2] = raw_makeHDR(typeOfMethod, num, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(8), varargin(9))

		case 12 then
			for i = 1:num
				varargin(i) = mattolist(varargin(i))
			end

			[out1, out2] = raw_makeHDR(typeOfMethod, num, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(8), varargin(9), varargin(10))

		case 13 then
			for i = 1:num
				varargin(i) = mattolist(varargin(i))
			end

			[out1, out2] = raw_makeHDR(typeOfMethod, num, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(8), varargin(9), varargin(10), varargin(11))
		
		case 14 then 
			for i = 1:num
				varargin(i) = mattolist(varargin(i))
			end
			
			[out1, out2] = raw_makeHDR(typeOfMethod, num, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(8), varargin(9), varargin(10), varargin(11), varargin(12))

		case 15 then 
			for i = 1:num
				varargin(i) = mattolist(varargin(i))
			end
			
			[out1, out2] = raw_makeHDR(typeOfMethod, num, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(8), varargin(9), varargin(10), varargin(11), varargin(12), varargin(13))

		case 16 then 
			for i = 1:num
				varargin(i) = mattolist(varargin(i))
			end
			
			[out1, out2] = raw_makeHDR(typeOfMethod, num, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(8), varargin(9), varargin(10), varargin(11), varargin(12), varargin(13), varargin(14))
		
		case 17 then 
			for i = 1:num
				varargin(i) = mattolist(varargin(i))
			end
			
			[out1, out2] = raw_makeHDR(typeOfMethod, num, varargin(1), varargin(2), varargin(3), varargin(4), varargin(5), varargin(6), varargin(7), varargin(8), varargin(9), varargin(10), varargin(11), varargin(12), varargin(13), varargin(14), varargin(15))

	end
		

	channels1 = size(out1)
	channels2 = size(out2)

	for i = 1:channels1	
		outHDR(:, :, i) = out1(i)
	end

	for i = 1:channels2	
		outLDR(:, :, i) = out2(i)
	end

endfunction
