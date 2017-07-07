function [outputImg]=wiener2(inputImage,filtsize,sigma)
	inputList=mattolist(inputImage);
    outputList=opencv_wiener2(inputList,filtsize,sigma);
    for i=1:size(outputList)
        outputImg(:,:,i)=outputList(i)
    end
endfunction
