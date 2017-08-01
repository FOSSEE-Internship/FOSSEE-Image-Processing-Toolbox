// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Gursimar Singh
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function [kalmanFilter,plocation]=kalmanPredict(KalmanFilter)
//This function predicts the state of the object based on the previous states.
//
//Calling Sequence
//[plocation]=kalmanPredict(KalmanFilter);
//
//Parameters
//plocation - It is the predicted state or location of the object.It is a row vector of size same as dimensionality of state in KalmanFilter.
//KalmanFilter:Output of the function configKalmanFilter.It is a structure specifying parameters of the KalmanFilter.For more details see configKalmanFilter.
//
//Description
//The function is commonly used for object tracking in videos.The trajectory of the object to be tracked can be predicted based on the previous states of the object.
//
//Examples
//
//Authors
//Gursimar Singh
//
//See also
//kalmanCorrect
//configKalmanFilter

	
    [lhs rhs]=argn(0);
  
     	if lhs>2
        	error(msprintf(" Too many output arguments\n"));
    	elseif rhs>1
        	error(msprintf(" Too many input arguments\n"));
    	elseif rhs<1
        	error(msprintf("Too few input arguments\n"));
    	end 
   
    kalman_list=kalmanStructToList(KalmanFilter);

    [kalmanl,plocation]=raw_kalmanPredict(kalman_list);
    //plocation=kalman_list;
     kalmanFilter=struct("StateTransitionModel",kalmanl(1),"MeasurementModel",kalmanl(2),"ControlModel",kalmanl(3),"State",kalmanl(4),"StateCovariance",kalmanl(5),"ProcessNoise",kalmanl(6),"MeasurementNoise",kalmanl(7));

endfunction
