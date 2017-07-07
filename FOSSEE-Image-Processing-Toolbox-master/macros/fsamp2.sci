// Function Name = Frequency sampling

// Inputs = Frequency response Hd; frequency vectors f1, f2; dimension [m n]

// Output = matrix h of dimension m x n, stores the filter coeff

function[h] = fsamp2(Hd, varargin)

	[lhs, rhs] = argn(0)
	
	select rhs
		case 1 then
			out = opencv_fsamp2(Hd)
	
		case 2 then
			out = opencv_fsamp2(Hd, varargin(1))
			
		case 3 then
			out = opencv_fsamp2(Hd, varargin(1), varargin(2))
			
		case 4 then
			out = opencv_fsamp2(Hd, varargin(1), varargin(2), varargin(3))
		end
		
	channel = size(out)
	
	for i = 1: channel
		h(:,:,i) = out(i)
	end
	
endfunction
