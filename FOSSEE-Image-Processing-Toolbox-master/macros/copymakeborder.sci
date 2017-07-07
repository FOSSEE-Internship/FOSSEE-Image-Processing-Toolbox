function new_image = copymakeborder(image, top, bottom, left, right, borderType, varargin)
	
	[lhs, rhs] = argn(0)
	
	image_list = mattolist(image)
	
	select rhs
		case 6 then
			out = opencv_copymakeborder(image_list, top, bottom, left, right, borderType)
		
		case 7 then
			out = opencv_copymakeborder(image_list, top, bottom, left, right, borderType, varargin(1))
	end
	
	sz = size(out)
	for i = 1 : sz
		new_image(:, :, i) = out(i)
	end
			
endfunction
