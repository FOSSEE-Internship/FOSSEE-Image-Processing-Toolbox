function outputImg= medianblur(inputImage,ksize)
	inputList=mattolist(inputImage);
    outputList=opencv_medianblur(inputList,ksize)
    for i=1:size(outputList)
        outputImg(:,:,i)=outputList(i)
    end
endfunction
