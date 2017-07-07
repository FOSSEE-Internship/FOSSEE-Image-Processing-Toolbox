function[sum] = imadd(matA, matB)

	[lhs, rhs] = argn(0)
	
	sum = opencv_imadd(matA, matB)
	
endfunction
