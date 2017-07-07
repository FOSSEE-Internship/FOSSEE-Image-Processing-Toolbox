function[dstImg] = regionfill(srcImg, varargin)
	
	[lhs, rhs] = argn(0)
	
	srcMat = mattolist(srcImg)

	if rhs == 2 then
		out = opencv_regionfill(srcMat, varargin(1))
	elseif rhs == 2 then
		out = opencv_regionfill(srcMat, varargin(1), varargin(2))
	end
	
	channels = size(out)
	
	for i = 1:channels
		dstImg(:,:,i) = out(i)
	end
endfunction
