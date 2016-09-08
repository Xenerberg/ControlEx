%Abstract class definition for Realizing a VREP robot
%Classname: CRobot
%Utitlity: Encapsulates the generic behavior of a VREP robot
%Author: Hrishik Mishra
classdef (Abstract = true) CRobot
   properties (Constant = true)
       f_lambda = 1e-6; %DLS parameter for kinematic damping during inv(J)
   end
   properties (SetAccess = protected)       
       n_Joints = 0;
       n_Links = 0;
       mat_DH = sym(zeros(0));
       vec_q = sym(zeros(0));%Generalized coordinates (Joint space)
       vec_Links = sym(zeros(0));%Link lengths for DH-parameters (sym)
       vec_base = [0;0;0;1];        
       mat_T_ee = sym(zeros(0));%End effector HTM
       mat_T=sym(zeros(0));%Transformation to (i)th from (0)th joint.
       %T is a 3-D array in which (:,:,i) gives the ith Transformation. i.e
       %from base to i till n_joints
       mat_T_CoM=sym(zeros(0));%Transformation to link CoM from joint frame
       mat_J=sym(zeros(0));%Full Jacobian of the Robot EE.      
       
       vec_link_len = zeros(0);       
       h_func_EEJacob = [];
       mat_curr_J = [];
   end
   properties(SetAccess = public)
       vec_curr_ee_pos = zeros(3,1);
       vec_curr_ee_or = zeros(3,3);
       vec_q_true = zeros(0);  
       vec_ee_V = zeros(3,1);
       vec_ee_Omega = zeros(3,1);
   end
   methods (Static = true)
       function [JStar] = fn_PseudoInverse(J)
            JStar = pinv(J);
       end
       function [JStar] = fn_DLSInverse(J)
            JStar = J'/(J*J' + 1e-6*eye(12,12));
       end     
       
   end
  
end