function[dstMat] = imfill(srcImg, varargin)
	
	[lhs, rhs] = argn(0)
	
	srcMat = mattolist(srcImg)

	select rhs
		case 1 then
			out = opencv_imfill(srcMat)
		case 2 then
			out = opencv_imfill(srcMat, varargin(1))
	end
	
	channel = size(out)
	
	for i = 1: channel
		dstMat(:,:,i) = out(i)
	end
	
endfunction
