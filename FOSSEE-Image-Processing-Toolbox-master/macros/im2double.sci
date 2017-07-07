function[doubleMat] = im2double(matA)
	
	srcMat = mattolist(matA)

	out = opencv_im2double(srcMat)

	ch = size(out)

	for i = 1:ch
		doubleMat(:,:,i) = out(i)
	end
	
endfunction
