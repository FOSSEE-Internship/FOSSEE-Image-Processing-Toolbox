function outputImg= houghcircles(inputImage , dp , mindist , param1, param2 ,minradius, maxradius)
	inputList=mattolist(inputImage)
    outputList=opencv_houghcircles(inputList , dp , mindist , param1, param2 ,minradius, maxradius)
    for i=1:size(outputList)
        outputImg(:,:,i)=outputList(i)
    end
endfunction
