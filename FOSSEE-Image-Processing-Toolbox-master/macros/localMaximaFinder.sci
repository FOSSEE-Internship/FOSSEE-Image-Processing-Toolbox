function [location]=localMaximaFinder(inputMatrix,neighborhood,maxNoOfMaxima,threshold)
// Returns local maxima in input matrices
//
// Calling Sequence
//   location=localMaximaFinder(inputMatrix,neighborhood,maxNoOfMaxima,threshold)
//
// Parameters
// inputMatrix: 2-D input matrix 
// neighborhood: It specifies the neighborhood around maxima, function zeros out the values inside the block. Neighborhood is a two element vector,first one indicates number of rows in neighborhood,second one indicates columns, of odd intergers. 
// maxNoOfMaxima: maximum number of local maxima you want to find.
// threshold: It specifies minimum value of the local maxima.
//
// Description
// localMaximaFinder returns the M-by-2 matrix where M is the number of local maxima and each row represents one local maximum
//
// Examples
// I=imread("lena.jpeg");
// grayImage=rgb2gray(I);
// location=localMaximaFinder(grayImage);
// mat=[1 6 3;4 8 2;9 7 5];
// loc=localMaximaFinder(mat,[1 3],4,3);
  
    [lhs,rhs]=argn(0)
    if rhs<4 then
         error(msprintf(" Not enough input arguments"))
    elseif rhs>4 then
         error(msprintf(" Too many input arguments to the function"))
    elseif lhs>1 then
         error(msprintf(" Too many output arguments"))
    end
    
    [iRows iCols]=size(inputMatrix)
    [nRows nCols]=size(neighborhood)
    if ~isreal(inputMatrix) | ~isreal(neighborhood)
        error(msprintf(" Wrong input argument,complex matrix is not expected"))
    elseif ~nRows==1 | ~nCols==2
        error(msprintf("Neighborhood property must be a vector with 2 columns"))
    end
    
    nh_x=neighborhood(1,1);
	nh_y=neighborhood(1,2);
    if modulo(nh_x,2)==0 | modulo(nh_y,2)==0
        error(msprintf("-Neighborhood property must be odd"))
    elseif ~nh_x<=iRows | ~nh_y<=iCols
        error(msprintf("Each dimension of the NeighborhoodSize property value must be less than or equal to the corresponding dimension of the input matrix"))
    end
    
    nh_x=floor(nh_x/2);
	nh_y=floor(nh_y/2);
    count=0;
    index=1;
    k=1;
    while count<maxNoOfMaxima
		max_data=-1;
        //to find maximum in the input matrix
		for i=1:iRows	
			for j=1:iCols
				if inputMatrix(i,j)>max_data
					max_data=inputMatrix(i,j);
					max_xIn=i;
					max_yIn=j;
                end
            end
        end
        if threshold<=max_data
            location(k,1)=max_yIn;
            location(k,2)=max_xIn;
            k=k+1;
            
            //to zero-out the values around the neighborhood
            if max_xIn-nh_x<1 then
                start_x=1
            else
                start_x=max_xIn-nh_x
            end
            if max_yIn-nh_y<1 then
                start_y=1
            else
                start_y=max_yIn-nh_y
            end
            if max_xIn+nh_x>iRows then
                end_x=iRows
            else
                end_x=max_xIn+nh_x
            end
            if max_yIn+nh_y>iCols then
                end_y=iCols
            else
                end_y=max_yIn+nh_y
            end
            for i=start_x:end_x
                for j=start_y:end_y
                    inputMatrix(i,j)=0;
                end
            end
            //to check the existence of non-zero element
            nonZeroCount=0;
            for i=1:iRows
                for j=1:iCols
                    if ~inputMatrix(i,j)==0 then
                        nonZeroCount=nonZeroCount+1;
                        break;
                    end
                end
                if nonZeroCount==1 then
                    break;
                end
            end
            if nonZeroCount==0 then
                    break;
            end
            count=count+1;
        
        else
            break;  //if maximum number in input matrix is less than threshold
        end
    end    
endfunction
