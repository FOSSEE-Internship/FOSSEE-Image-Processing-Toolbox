function varargout=readForegroundMask(varargin)
// This function computes the foreground mask for the input image.
//
// Calling Sequence
//   mask = readForegroundMask(image)
//   
// Parameters
// mask: the computed foreground mask.
// image : the input image
//  
//
// Description
// This function returns the foreground mask for an image using Gaussian Mixed Models.
//
// Examples
// myStr = ForegroundDetector()
// myVideo = VideoReader('sample.mp4');
// while hasFrame()
// frame = readFrame();
// mask = readForegroundMask(frame);
// end
// 
//
// Authors
//  Shashank Shekhar
[lhs,rhs]=argn(0)
    if rhs<1 then
         error(msprintf(" Not enough input arguments"))
    end
    if rhs>1 then
          error(msprintf(" Too many input arguments to the function"))     
    end
    if lhs>1 then
         error(msprintf(" Too many output arguments"))
    end
    varargout(1) = ocv_readForegroundMask(varargin(1));
endfunction
