function [out]=integralFilter(intimage,bbox,weights,filterSize)
// Integral Image based Filter.
//
// Calling Sequence
//   filter = integralFilter(intimage,bbox,weights,filterSize);
//
// Parameters
// intimage: Integral Image, which can be obtained from the function integralImage.
// bbox: Bounding box of the filter object, which can be obtained from integralKernel function.
// weight: Weights of the bounding box, which can be obtained from integralKernel function.
// filterSize: Size of the filter, which can be obtained from integralKernel function.
//
// Description
// This function filters image using box filters and integral images.
//
// Examples
// i = imread("lena.jpg");
// intImg = integralImage(i);
// kernel = integralKernel([2 2 11 11], 1/51);
// filter = integralKernel(intImg, kernel.bbox, kernel.weights, kernel.filterSize);
//
// Authors
//  Tanmay Chaudhari

     	inputimage1=mattolist(inputimage);
     	a=opencv_integralFilter(inputimage1,bbox,weights,filterSize);
     	out(:,:,1)=a(1);
	
endfunction
