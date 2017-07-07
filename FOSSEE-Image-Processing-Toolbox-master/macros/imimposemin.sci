function[dstMat] = imimposemin(maskImg, markerImg)
	
	out = opencv_imimposemin(maskImg, markerImg)
	
	channel = size(out)
	
	for i = 1: channel
		dstMat(:,:,i) = out(i)
	end
	
endfunction
