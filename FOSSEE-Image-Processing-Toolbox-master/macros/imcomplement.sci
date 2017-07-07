function[compMat] = imcomplement(srcImg)
	
	srcMat = mattolist(srcImg)

	out = opencv_imcomplement(srcMat)

	ch = size(out)
	
	for i=1:ch
		compMat(:,:,i) = out(i)
	end
	
endfunction
