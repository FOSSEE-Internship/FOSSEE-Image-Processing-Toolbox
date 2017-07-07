//Author- Sridhar Reddy
//inputImage should be matrix
function [outputImg]=rgb2lab(inputImage)
    inputList=mattolist(inputImage);
    outputList=opencv_rgb2lab(inputList);
    disp(outputList(2));
    for i=1:size(outputList)
        outputImg(:,:,i)=outputList(i)
    end
endfunction
