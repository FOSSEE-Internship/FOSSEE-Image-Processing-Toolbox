function outputImg= laplacian(inputImage , depth , ksize , scale, delta)
	inputList=mattolist(inputImage);
    outputList=opencv_laplacian(inputList , depth , ksize , scale, delta)
    for i=1:size(outputList)
        outputImg(:,:,i)=outputList(i)
    end
endfunction
