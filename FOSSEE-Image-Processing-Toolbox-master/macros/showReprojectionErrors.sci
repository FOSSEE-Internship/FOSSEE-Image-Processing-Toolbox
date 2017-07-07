function showReprojectionErrors(cameraParameters)
// It displays reprojection errors using bar graph 
//
// Calling Sequence
//   showReprojectionErrors(cameraParameters)
//
// Parameters
// cameraParameters: cameraParameters strucure
//
// Description
//  It visualizes reprojection errors for a single camera or stereo pair if cameras.
//
// Examples
// showReprojectionErrors(cameraParameters);
    [lhs rhs]=argn(0)
    if rhs<1 then
        error(msprintf("Need atleat one argument"));
    elseif ~isstruct(cameraParameters) then
        error(msprintf("Given input is not a valid cameraParameters structure"));
    end

    ReprojectionErrors=cameraParameters.ReprojectionErrors;
    if ~isempty(ReprojectionErrors) then
        ReprojectedPoints=cameraParameters.ReprojectedPoints;
        ReprojectionErrors=cameraParameters.ReprojectionErrors;
        ImagePoints=ReprojectedPoints-ReprojectionErrors;
        totalErr=0;
        rvRows=size(cameraParameters.RotationVectors,'r');
        wpRows=size(cameraParameters.WorldPoints,'r');
        for i=1:rvRows
            err=0;
            for j=1:wpRows
                err=err+((ReprojectedPoints(j,1,i)-ImagePoints(j,1,i))+(ReprojectedPoints(j,2,i)-ImagePoints(j,2,i))).^2;
            end
            totalErr=totalErr+err;
            errors(i)=sqrt(err/wpRows);
        end
        totalPoints=rvRows*wpRows;
        meanReprojectionError=sqrt(totalErr/totalPoints);
    end
    da=gda();
    da.title.text="Mean Reprojection Errors per Image"
    da.title.foreground = 12;
    da.title.font_size = 5;
    da.x_label.text="Images";
    da.x_label.font_style = 3;
    da.x_label.font_size = 3;
    da.x_label.foreground = 3;
    da.x_location = "bottom";
    da.y_label.text="Mean Reprojection Errors";
    da.y_label.font_style = 3;
    da.y_label.font_size = 3;
    da.y_label.foreground = 3;
    da.y_location = "left";
    a=gca();
    bar(a,errors,0.2);    //0.2- width of each bar
endfunction
