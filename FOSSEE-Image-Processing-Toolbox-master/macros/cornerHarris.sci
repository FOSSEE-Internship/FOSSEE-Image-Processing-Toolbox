function [outputImg]= cornerHarris(inputImage, blockSize, kSize, k, borderType)
	inputList=mattolist(inputImage);
    outputList=opencv_cornerHarris(inputList, blockSize, kSize, k, borderType);
    for i=1:size(outputList)
       outputImg(:,:,i)=outputList(i)
   end
endfunction
