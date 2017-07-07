function [out]=houghlines(image,rho, theta, threshold, srn, stn)
         image1=mattolist(image);
         out=opencv_houghlines(image1,rho, theta, threshold, srn, stn);
endfunction;
