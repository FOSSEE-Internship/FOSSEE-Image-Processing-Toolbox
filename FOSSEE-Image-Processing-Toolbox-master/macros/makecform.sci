function[out_string] = makecform(srcImg)
	srcMat = mattolist(srcImg)
	out_string = opencv_makecform(srcMat)
endfunction
