function [outputImg]=gabor(wavelength,orientation)
    outputList=opencv_gabor(wavelength,orientation);
    for i=1:size(outputList)
        outputImg(:,:,i)=outputList(i)
    end
endfunction
