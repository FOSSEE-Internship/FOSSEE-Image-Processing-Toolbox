function [outputImg]= cornerMinEigenVal(inputImage, blockSize, kSize, borderType)
	inputList=mattolist(inputImage);
    outputList=opencv_cornerMinEigenVal(inputList, blockSize, kSize, borderType);
    for i=1:size(outputList)
       outputImg(:,:,i)=outputList(i)
   end
endfunction
