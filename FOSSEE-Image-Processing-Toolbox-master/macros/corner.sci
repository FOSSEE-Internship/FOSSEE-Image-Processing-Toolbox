// This function detects corner in image I and returns it in the matrix C

function[C] = corner(I, varargin)

	[lhs, rhs] = argn(0)
	
	if rhs == 2 then
		error(msprintf("No of input cannot be 2"))
	end
	
	srcMat = mattolist(I)
	select rhs 
		case 1 then
			out = opencv_corner(srcMat)
			
		case 3 then 
			out = opencv_corner(srcMat, varargin(1), varargin(2))
	end
	
	channels = size(out)
	
	for i = 1:channels
		C(:,:,i) = out(i)
	end
	
endfunction
