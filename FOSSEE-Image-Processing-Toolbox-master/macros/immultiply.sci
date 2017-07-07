function[multiplyMat] = immultiply(matA, matB)
	
	out = opencv_immultiplyt(matA, matB)
	
	channel = size(out)
	
	for i = 1: channel
		multiplyMat(:,:,i) = out(i)
	end
endfunction
