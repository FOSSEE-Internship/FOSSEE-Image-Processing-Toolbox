function [outputImg]=undistort(inputImage,cameraMatrix,distCoeffActual,newCameraMatrix)
	inputList=mattolist(inputImage);
    outputList=opencv_undistort(inputList,cameraMatrix,distCoeffActual,newCameraMatrix);
    for i=1:size(outputList)
        outputImg(:,:,i)=outputList(i)
    end
endfunction
