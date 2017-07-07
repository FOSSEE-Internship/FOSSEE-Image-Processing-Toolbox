function imgSetList=imageSetToList(imageSet)
    if isstruct(imageSet)<> %T  then
            error(msprintf("Error: The input argument is not of type imageSet."));
        end
        fieldNamesActual=["Description";"ImageLocation";"Count"];
        fieldNamesInput=fieldnames(imageSet);
        if fieldNamesActual <> fieldNamesInput then
            error(msprintf("Error: The input argument is not of type imageSet, Wrong field names"));
        end
        if size(imageSet) == [1 1] then
            if type(imageSet.Description)<> 10 | size(imageSet.Description) <> [1 1] then
            error(msprintf("Error: The input argument is not of type imageSet, Description field must be string"));
            end
            if type(imageSet.Count)<> 1 | size(imageSet.Count) <> [1 1] then
                error(msprintf("Error: The input argument is not of type imageSet, Count field must be integer"));
            end
            if type(imageSet.ImageLocation)<> 10 & size(imageSet.ImageLocation,'c') <> 1 then
                error(msprintf("Error: The input argument is not of type imageSet, ImageLocation must be strings"));
            end
            locationList=list();
                Desc=imageSet.Description;
                Count=int32(imageSet.Count);
                for j=1:imageSet.Count
                    tempMat(1,j)=imageSet.ImageLocation(j);
                end
                locationList($+1)=tempMat;
        else
            for i=1:size(imageSet.Count)
                if type(imageSet(i).Description)<> 10 | size(imageSet(i).Description) <> [1 1] then
                    error(msprintf("Error: The input argument is not of type imageSet, Description field must be string"));
                end
                if type(imageSet(i).Count)<> 1 | size(imageSet(i).Count) <> [1 1] then
                    error(msprintf("Error: The input argument is not of type imageSet, Count field must be integer"));
                end
                if type(imageSet(i).ImageLocation)<> 10 & size(imageSet(i).ImageLocation,'c') <> 1 then
                    error(msprintf("Error: The input argument is not of type imageSet, ImageLocation must be strings"));
                end
            end
            locationList=list();
            for i=1:size(imageSet.Count)
                Desc(i)=imageSet(i).Description;
                Count(i)=int32(imageSet(i).Count);
                for j=1:imageSet(i).Count
                    tempMat(1,j)=imageSet(i).ImageLocation(j);
                end
                locationList($+1)=tempMat;
            end
        end
        imgSetList=list("imageSet",Desc,Count,locationList);
endfunction
