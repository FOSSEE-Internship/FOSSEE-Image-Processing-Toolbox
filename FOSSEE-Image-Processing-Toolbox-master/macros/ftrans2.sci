// Function Name = Frequency Transformation

// Inputs = 1-D FIR filter b; Transform t(varargin(1))

// Output = 2-D FIR filter h

function[h] = ftrans2(b, t)

	[lhs, rhs] = argn(0)
	
	select rhs
		case 1 then
			out = opencv_ftrans2(b)
	
		case 2 then
			out = opencv_ftrans2(b, varargin(1))
	end
	
	channel = size(out)
	
	for i = 1: channel
		h(:,:,i) = out(i)
	end
	
endfunction
