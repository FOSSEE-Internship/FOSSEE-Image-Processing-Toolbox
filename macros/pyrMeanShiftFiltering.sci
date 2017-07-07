// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Priyanka Hiranandani,Gursimar Singh
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [outputImg]=pyrMeanShiftFiltering(inputImage,sp,sr,varargin)
//Performs initial step of meanshift segmentation of an image
//
//Calling Sequence
//[outputImg]=pyrMeanShiftFiltering(inputImage,sp,sr)
//[outputImg]=pyrMeanShiftFiltering(inputImage,sp,sr,maxLevel)
//
//Parameters
//outputImg:The destination image of the same format and the same size as the source inputImage.
//inputImage:The source image.
//SP:The spatial window radius.
//SR:The color window radius.
//maxLevel:Maximum level of the pyramid for the segmentation. default 1
//
//Description
//The function implements the filtering stage of meanshift segmentation, that is, the output of the function is the filtered "posterized" image with color gradients and fine-grain texture flattened. At every pixel (X,Y) of the input image (or down-sized input image, see below) the function executes meanshift iterations, that is, the pixel (X,Y) neighborhood in the joint space-color hyperspace is considered.
//When MaxLevel > 0, the gaussian pyramid of MaxLevel+1 levels is built, and the above procedure is run on the smallest layer first. After that, the results are propagated to the larger layer and the iterations are run again only on those pixels where the layer colors differ by more than SR from the lower-resolution layer of the pyramid. That makes boundaries of color regions sharper. Note that the results will be actually different from the ones obtained by running the meanshift procedure on the whole original image (i.e. when MaxLevel = 0).
//
//Examples
//im=imread("images/lena.jpg");
//img=pyrMeanShiftFiltering(im,100,200);
//figure("Figure_name","originalImage");
//imshow(im);
//figure("Figure_name","ProcessedImage");
//imshow(img);
//
//Authors
//Priyanka Hiranandani
//Gursimar Singh


	[lhs,rhs]=argn(0);

	if rhs>4 then
        error(msprintf("Too many input arguments"));
     end
     if rhs<3 then
	   error(msprintf("Not enough input arguments"));
	 end
	 if lhs >1 then
	   error(msprintf("Too many output arguments"));
     end


    inputList=mattolist(inputImage);
    if rhs==3 then
   	  outputList=raw_pyrMeanShiftFiltering(inputList,sp,sr);
   	elseif rhs==4 then
   	  outputList=raw_pyrMeanShiftFiltering(inputList,sp,sr,varargin(1)); 
	  end 

   for i=1:size(outputList)
       outputImg(:,:,i)=outputList(i)
   end
endfunction
