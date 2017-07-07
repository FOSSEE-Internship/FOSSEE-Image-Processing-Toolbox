function [points, varargout] = imfindcircles(image, Rmin, Rmax)
	
	[lhs, rhs] = argn(0)
	
	image_list = mattolist(image)
	
	select lhs
		case 1 then
			out_centres = opencv_imfindcircles(image_list, Rmin, Rmax)
		
		case 2 then
			[out_centres radii] = opencv_imfindcircles(image_list, Rmin, Rmax)
			
			varargout(1) = radii
	end
	
	total_points = size(out_centres(1),'c')
	
	
	for i=1:total_points
		points(i, 1) = out_centres(1)(1, i)
		points(i, 2) = out_centres(2)(1, i)	
	end

endfunction
