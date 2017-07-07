function [varargout]=partition(imageSet,groupSizesOrPercentages,varargin)
// This function is used to split an imageSet.
//
// Calling Sequence
// [set1 set2 ... setN] = partition(imgSet, groupSizes)
// [set1 set2 ... setN] = partition(imgSet, groupPercentages)
// [set1 set2 ... setN] = partition(..., method)
//
// Parameters
// set: Output imageSet structure
// imgSet: Input imageSet to be split
// groupSizes: Matrix that specifies size of output sets
// groupPercentages: Matrix that specifies percentage content of each output set
// method: (Optional) Determines whether split is 'sequential' or 'randomized' (Default: 'sequential')
//
// Description
// This function splits an imageSet into two or more imageSets on the basis of the groupSizes or groupPercentages provided.
//
// Examples
// 
// imgSet = imageSet(directory,'recursive');
// [set1 set2 set3] = partition(imgSet,[5 , 3], 'randomized');
//
//   imgSet = imageSet(directory,'recursive');
// [trainingSet testSet] = partition(imgSet,[0.8]);
//
// imgSet = imageSet(directory,'recursive');
// [trainingSet testSet] = partition(imgSet,[5], 'randomized');
//
// Authors
//  Rohit Suri

    [lhs rhs]=argn(0);
    if rhs>3
        error(msprintf(" Too many input arguments"));
    end
    cols=size(groupSizesOrPercentages,'c');
    if lhs>cols+1 then
        error(msprintf(" Too many output arguments"));
    end
    imgSetList=imageSetToList(imageSet);
    //sets=list();`
    select rhs
        case 2 then
            sets=raw_partition(imgSetList,groupSizesOrPercentages);
        case 3 then
            sets=raw_partition(imgSetList,groupSizesOrPercentages,varargin(1));
    end
    for i=1:lhs
        for j=1:length(sets(i)(3))
            for k=1:sets(i)(3)(j)
                imgLocations(k)=sets(i)(4)(j)(1,k);
            end    
         imgSet(1,j)=struct('Description',sets(i)(2)(j),'ImageLocation',imgLocations,'Count',int32(sets(i)(3)(j)) );
	     imgLocations=[]
        end    
     varargout(i) = imgSet;
    end
endfunction


