function [textSize,baseLine]= getTextSize(inputtext,fontface,fontscale,thickness)
    [textSize,baseLine]=opencv_getTextSize(inputtext,fontface,fontscale,thickness)
endfunction
