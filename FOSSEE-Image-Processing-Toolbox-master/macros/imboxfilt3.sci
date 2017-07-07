//1) box_filtered_image = imboxfilt3(input_img);
		//In this usage, the default filter size of 3x3 is used.
	
//	2) box_filtered_image = imboxfilt3(input_img, filter_height, filter_width);

function[dstMat] = imboxfilt3(srcImg, varargin)

	[lhs, rhs] = argn(0)
	
	srcMat = mattolist(srcImg)

	select rhs
		case 1 then
			out = opencv_imboxfilt3(srcMat)
	
		case 2 then
			out = opencv_imboxfilt3(srcMat, varargin(1))
			
		case 3 then
			out = opencv_imboxfilt3(srcMat, varargin(1), varargin(2))
		end
	channel = size(out)
	
	for i = 1: channel
		dstMat(:,:,i) = out(i)
	end
	
endfunction
