function [outputMat]= findcontours(inputImage, Mode, method, point_x, point_y)
	inputList=mattolist(inputImage);
    outputList=opencv_findcontours(inputList,Mode, method, point_x, point_y)
    for i=1:size(outputList)
       outputMat(:,:,i)=outputList(i)
   end
endfunction
