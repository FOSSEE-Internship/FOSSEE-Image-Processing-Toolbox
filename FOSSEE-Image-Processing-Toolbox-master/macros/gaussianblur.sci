function [outputImg]= gaussianblur(inputImage,ksize_height,ksize_width,sigmaX,sigmaY)
	inputList=mattolist(inputImage);
    outputList=opencv_gaussianblur(inputList,ksize_height,ksize_width,sigmaX,sigmaY);
    for i=1:size(outputList)
       outputImg(:,:,i)=outputList(i)
   end
endfunction
