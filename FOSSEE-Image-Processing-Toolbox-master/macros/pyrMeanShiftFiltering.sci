function [outputImg]=pyrMeanShiftFiltering(inputImage,sp,sr)
    inputList=mattolist(inputImage);
   outputList=opencv_pyrMeanShiftFiltering(inputList,sp,sr);
   
   for i=1:size(outputList)
       outputImg(:,:,i)=outputList(i)
   end
endfunction
