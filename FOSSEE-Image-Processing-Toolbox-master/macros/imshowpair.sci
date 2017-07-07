function [outputImg]=imshowpair(inputImage1,inputImage2,method,scale)
	inputList1=mattolist(inputImage1);
    inputList2=mattolist(inputImage2);
    outputList=opencv_imshowpair(inputList1,inputList2,method,scale);
    for i=1:size(outputList)
        outputImg(:,:,i)=outputList(i)
    end
endfunction
