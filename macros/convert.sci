function dst = convert(src,rtype, varargin)
// Converts an array to another data type with optional scaling.
//
// Calling Sequence
// src = imread('location-of-src');
// dst = convert(src,rtype, alpha,beta);
// 
// Parameters
// src : the source matrix/image
// alpha – optional scale factor (default value is 1) (type: double)
// beta – optional delta added to the scaled values (default value is 0) (type: double)
// rtype - desired output matrix type/depth (type: string) 
// 
// It supports the following types: 
//
//  CV_8U    //the no of channel remains the same
//  CV_8S    //the no of channel remains the same
//  CV_16U   //the no of channel remains the same 
//  CV_16S   //the no of channel remains the same 
//  CV_32S   //the no of channel remains the same
//  CV_32F   //the no of channel remains the same
//  CV_64F   //the no of channel remains the same 
//  
//  CV_8UC1
//  CV_8UC2  
//  CV_8UC3
//  CV_8UC4
//  
//  CV_8SC1
//  CV_8SC2
//  CV_8SC3
//  CV_8SC4
//
//  CV_16UC1
//  CV_16UC2
//  CV_16UC3
//  CV_16UC4
//
//  CV_16SC1
//  CV_16SC2
//  CV_16SC3
//  CV_16SC4
//
//  CV_32SC1
//  CV_32SC2
//  CV_32SC3 
//  CV_32SC4
//
//  CV_64FC1
//  CV_64FC2
//  CV_64FC3 
//  CV_64FC4



// Description
// The method converts source pixel values to the target data type. saturate_cast<> is applied at the end to avoid possible overflows:
//
// m(x,y) = saturate_cast<rType>( alpha (*this)(x,y) + beta )
//
// Examples
// src = imread('images/1.jpg');
// dst = convert(src,'CV_8UC1') //convert to single channel 8 bit insigned int using default values of alpha and beta 
// dst //viewing the content of dst
//
// dst2 = convert(src,'CV_32FC1') //convert to single channel 32 bit floating point using default values of alpha and beta
// dst2 //viewing the content of dst2
//
// Authors
// Rohan Gurve


		[ lhs rhs ] = argn(0)
		if lhs > 1 then
			error(msprintf("Too many output argument"))
		end
		
		if rhs > 4 then
			error(msprintf("Too many input arguments"))
		elseif rhs < 2 then
			error(msprintf("Input arguments missing"))
		end	
		
	    image_list = mattolist(src)
		
		alpha = argindefault ( varargin , 1 , 1 );//default value is 1
        beta = argindefault ( varargin , 2 , 0 );//default value is 0	
	    
	    temp = raw_convert(image_list, rtype, alpha,beta)
		
		
		sz = size(temp)
		
		for i=1 : sz
			dst(:, :, i) = temp(i)
		end
		
endfunction
