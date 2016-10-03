classdef CUR5 < CRobot
   methods
       function objCUR5 = CUR5()
           %Initialize the kinematic chain
           objCUR5.n_Joints = 6;
           objCUR5.n_Links = 7;
           %Initialize symbolic links and joint positions
           objCUR5.vec_q = sym('q_%d',[6,1]);
           objCUR5.vec_Links = sym('l_%d',[7,1]);
           %Initialize DH parameters in symbolic form
           objCUR5.mat_DH(1,:) = [objCUR5.vec_Links(2)+objCUR5.vec_Links(1),objCUR5.vec_q(1),0,90];
           objCUR5.mat_DH(2,:) = [0,objCUR5.vec_q(2)+90,objCUR5.vec_Links(3),0];
           objCUR5.mat_DH(3,:) = [0,objCUR5.vec_q(3),objCUR5.vec_Links(4),0];
           objCUR5.mat_DH(4,:) = [objCUR5.vec_Links(5),90+objCUR5.vec_q(4),0,90];
           objCUR5.mat_DH(5,:) = [objCUR5.vec_Links(6),180+objCUR5.vec_q(5),0,90];
           objCUR5.mat_DH(6,:) = [objCUR5.vec_Links(7),objCUR5.vec_q(6),0,0];
           %Initialize HTMs in symbolic form
           objCUR5.mat_T(:,:,1) = DH2HTM(objCUR5.mat_DH(1,:));           
           objCUR5.mat_T(:,:,2) = DH2HTM(objCUR5.mat_DH(2,:));
           objCUR5.mat_T(:,:,3) = DH2HTM(objCUR5.mat_DH(3,:));
           objCUR5.mat_T(:,:,4) = DH2HTM(objCUR5.mat_DH(4,:));
           objCUR5.mat_T(:,:,5) = DH2HTM(objCUR5.mat_DH(5,:));
           objCUR5.mat_T(:,:,6) = DH2HTM(objCUR5.mat_DH(6,:));
           
           objCUR5.mat_T_ee = objCUR5.mat_T(:,:,1)*objCUR5.mat_T(:,:,2)*objCUR5.mat_T(:,:,3)*objCUR5.mat_T(:,:,4)*objCUR5.mat_T(:,:,5)*objCUR5.mat_T(:,:,6);
           ee_position = objCUR5.mat_T_ee*objCUR5.vec_base;
           ee_orientation = objCUR5.mat_T_ee(1:3,1:3);
           objCUR5.mat_J = jacobian([ee_position(1:3);ee_orientation(1:3,1);ee_orientation(1:3,2);ee_orientation(1:3,3)],objCUR5.vec_q);
           %Instantiate the links
           objCUR5.vec_link_len = zeros(7,1);
           objCUR5.vec_link_len(1) = 0.0085;
           objCUR5.vec_link_len(2) = 0.0661;
           objCUR5.vec_link_len(3) = 0.4251;
           objCUR5.vec_link_len(4) = 0.3922;
           objCUR5.vec_link_len(5) = 0.1100;
           objCUR5.vec_link_len(6) = 0.0948;
           objCUR5.vec_link_len(7) = 0.0750;
           %Bad design/Remove it later
           if exist('J','file')               
               matlabFunction(objCUR5.mat_J,'Vars',{[objCUR5.vec_Links(1);objCUR5.vec_Links(2);objCUR5.vec_Links(3);objCUR5.vec_Links(4);objCUR5.vec_Links(5);objCUR5.vec_Links(6);objCUR5.vec_Links(7)],[objCUR5.vec_q(1);objCUR5.vec_q(2);objCUR5.vec_q(3);objCUR5.vec_q(4);objCUR5.vec_q(5);objCUR5.vec_q(6)]},'File','J');
           end
           rehash;
           objCUR5.h_func_EEJacob = @J;
           objCUR5.vec_curr_ee_pos = [-0.00036;-0.1850;0.9865];%All joint angles are zero
           objCUR5.vec_curr_ee_or = [1;0;0;0;0;1;0;-1;0];
           objCUR5.vec_q_true = zeros(6,1);
           
       end
      
   end
       
   
    
    
end

