function [outputImg]= cornerEigenValsAndVecs(inputImage, blockSize, kSize, borderType)
	inputList=mattolist(inputImage);
    outputList=opencv_cornerEigenValsAndVecs(inputList, blockSize, kSize, borderType);
    for i=1:size(outputList)
       outputImg(:,:,i)=outputList(i)
   end
endfunction
