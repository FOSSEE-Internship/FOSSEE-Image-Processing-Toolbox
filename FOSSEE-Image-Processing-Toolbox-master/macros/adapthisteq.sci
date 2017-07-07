function [outputImg]=adapthisteq(inputImage,varargin)
     [lhs rhs]=argn(0);
    if rhs>2
        error(msprintf(" Too many input arguments"));
    end
    inputList=mattolist(inputImage);
    select rhs
        case 1 then
            outputList=opencv_adapthisteq(inputList);
        case 2 then
            outputList=opencv_adapthisteq(inputList,varargin(1));
   end
    for i=1:size(outputList)
        outputImg(:,:,i)=outputList(i)
    end
endfunction
