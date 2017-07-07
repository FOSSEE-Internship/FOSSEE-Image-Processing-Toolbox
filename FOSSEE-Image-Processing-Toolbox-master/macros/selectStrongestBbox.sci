function [selectedBbox,selectedScore,varargout]=selectStrongestBbox(bBox,score,varargin)
// Selecting strongest bounding boxes
//
// Calling Sequence
//   [selectedBbox, selectedScore]= selectStrongestBbox(bBox,score);
//   [selectedBbox, selectedScore]= selectStrongestBbox(bBox,score,Name,Value);
//   [selectedBbox, selectedScore, selectionIndex]= selectStrongestBbox(bBox,score,Name,Value);
//
// Parameters
// bBox: Each row represents one bounding box specified as [x y width height];
// score: Confidence Score of bounding box
// varargin: optinal (Name,Value) pair arguments
// Optional arguments can be
// <itemizedlist>
// <listitem><para>RatioType: method to compute the ratio of the intersection area between two boxes and it is specified as string, possile values are 'Union', 'Min' </para></listitem>
// <listitem><para>overlapThreshold: It specifies maximum overlap ratio, if overlap ration is more than this corresponding bounding box will  be removed, possible range is 0 to 1</para></listitem>
// <itemizedlist>
//
// Description
// Returns strongest bounding boxes as per the given RatioType and OverlapThreshold and additionally it returns the index of the selected boxes
//
// Examples
// box(1)=[100 200 150  140];
// box(2)=[100 180 140 160];
// score(1)=0.03;
// score(2)=0.05;
// [sb ss]=selectStrongestBbox(box,score);
 
    [lhs,rhs]=argn(0)
     
    if rhs<2 then
         error(msprintf(" Not enough input arguments"))
    elseif rhs>6 then
         error(msprintf(" Too many input arguments to the function"))
    elseif lhs<2 then
         error(msprintf(" Not enough output arguments"))
    elseif lhs>3 then
         error(msprintf(" Too many output arguments"))
    end
    [bBoxRows bBoxCols]=size(bBox);
    [scoreRows scoreCols]=size(score);
    if ~bBoxCols==4 then
        error(msprintf("bounding box matrix must be M*4"))
    elseif ~scoreCols==1 then
          error(msprintf(" score matrix must be M*1"))
    elseif ~bBoxRows==scoreRows then
          error(msprintf(" The number of bounding boxes and scores should  be same"))
    elseif ~isreal(bBox)
        error(msprintf(" Wrong input argument,complex matrix is not expected"))
    end
    for i=1:bBoxRows
        if bBox(i,3)<0 | bBox(i,4)<0
            error(msprintf(" The width and height of the bounded box must be positive"))
        end
    end

    ratioType=1;
    overlapThreshold=0.5;
    for i=1:2:rhs-2
       if strcmpi(varargin(i),"RatioType")==0 then
            i=i+1;
            if strcmpi(varargin(i),'union')== 0 then
                ratioType=1;//1-union
            elseif  strcmpi(varargin(i),'min')==0 then
                ratioType=0;//0-min
            else
                error(msprintf(" wrong value for the ratio type,it must be union or min"))
            end
        elseif strcmpi(varargin(i),'OverlapThreshold')==0 then
            i=i+1;
            if 0<=varargin(i) & varargin(i)<=1 then
                overlapThreshold=varargin(i)
            else
                error(msprintf("wrong value for the overlapThreshold,it must be in between 0 and 1"))
            end
        else
            error(msprintf(_(" Wrong value for input argument")));
        end
    end
    for i=1:bBoxRows
        index(i)=i;
    end
    score_temp=score(:,1);
    for i=1:bBoxRows
        for j=i+1:bBoxRows
            if score_temp(i,1)<score_temp(j,1)
                temp=score_temp(i,1);
                score_temp(i,1)=score_temp(j,1);
                score_temp(j,1)=temp;
                temp=index(i);
                index(i)=index(j);
                index(j)=temp;
             end
        end
    end
    bBox_temp=bBox(index,:);
    selection=ones(size(bBox_temp,1),1);
    area = bBox_temp(:,3).*bBox_temp(:,4);
    x1 = bBox_temp(:,1); 
    x2 = bBox_temp(:,1)+bBox_temp(:,3); 
    y1 = bBox_temp(:,2); 
    y2 = bBox_temp(:,2)+bBox_temp(:,4);
    for i = 1:bBoxRows 
        if selection(i)
            for j = (i+1):bBoxRows 
                if selection(j)
                    width = min(x2(i), x2(j)) - max(x1(i), x1(j)); 
                    height = min(y2(i), y2(j)) - max(y1(i), y1(j)); 
                    intersectionArea = width * height; 
                    if ratioType
                        overlapRatio = intersectionArea/(area(i)+area(j)-intersectionArea); 
                    else
                        overlapRatio = intersectionArea/min(area(i), area(j)); 
                    end
                    
                    if overlapRatio > overlapThreshold 
                        selection(j) = 0; 
                    end
                end
            end
        end
    end
    k=1;
    for i=1:bBoxRows
        if selection(i)
            selectionIndex(k)=i;
            indexOriginal(k)=index(i);
            k=k+1;
        end
    end
    //disp(indexOriginal);
    [selectedIndexRows selectedIndexRows]=size(selectionIndex)
    for i=1:selectedIndexRows
        for j=i+1:selectedIndexRows
            if indexOriginal(i)>indexOriginal(j)
                temp=indexOriginal(i);
                indexOriginal(i)=indexOriginal(j)
                indexOriginal(j)=temp;
             end
        end
    end
    //disp("original indexes");
    //disp(indexOriginal);
    selectedBbox=bBox(indexOriginal,:);
    selectedScore=score(indexOriginal,:);
    varargout(1)=indexOriginal;
endfunction
