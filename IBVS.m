clear all;
b_cont = true;

SetPointFeatures = rgb2gray(imread('SetFeatures.jpg'));
corners1 = corner(SetPointFeatures,4);
%Link lengths based on DH-analysis
lenLinks(1) = 0.0085;
lenLinks(2) = 0.0661;
lenLinks(3) = 0.4251;
lenLinks(4) = 0.3922;
lenLinks(5) = 0.1100;
lenLinks(6) = 0.0948;
lenLinks(7) = 0.0750;



base_vector = [0;0;0;1]; %In homogeneous coordinates
%Homogenous Transformation Matrix of J1 w.r.t Base (0)
%DH parameters: [d,theta,r,alpha]
q = sym('q_%d',[6,1]);
lenLinks = sym('l_%d',[7,1]);
DH(1,:) = [lenLinks(2)+lenLinks(1),q(1),0,90];
DH(2,:) = [0,q(2)+90,lenLinks(3),0];
DH(3,:) = [0,q(3),lenLinks(4),0];
DH(4,:) = [lenLinks(5),90+q(4),0,90];
DH(5,:) = [lenLinks(6),180+q(5),0,90];
DH(6,:) = [lenLinks(7),q(6),0,0];

%[a,b,c] = dcm2angle(R','XYZ');%Rotation Matrix
%ee_angles = [a,b,c]*180/pi;

%Full Transformation
T_1_0 = DH2HTM(DH(1,:));
T_2_1 = DH2HTM(DH(2,:));
T_3_2 = DH2HTM(DH(3,:));
T_4_3 = DH2HTM(DH(4,:));
T_5_4 = DH2HTM(DH(5,:));
T_6_5 = DH2HTM(DH(6,:));
T_F = T_1_0*T_2_1*T_3_2*T_4_3*T_5_4*T_6_5;
computeT = matlabFunction(T_F);
%T = computeT(lenLinks(1),lenLinks(2),lenLinks(3),lenLinks(4),lenLinks(5),lenLinks(6),lenLinks(7),q(1),q(2),q(3),q(4),q(5),q(6));
%Transforming base vector
ee = T_F*base_vector;%Computed using DH parameters
ee = ee(1:3);
J_v = jacobian(T_F(1:3,4),q);
J_w = jacobian(T_F(1:3,3),q);
%J = [J_v;J_w];
J = jacobian([T_F(1:3,4);T_F(1:3,1);T_F(1:3,2);T_F(1:3,3)],q);
%Define Matlab function to obtain Jacobian
func_Jacobi = matlabFunction(J);%Only position control for now
clear lenLinks;clear q;
q = zeros(6,1);
% q(1) = 80;
% q(2) = -80;
% q(3) = 80;
% q(4) = 10;
% q(5) = 30;
% q(6) = 55;
lenLinks(1) = 0.0085;
lenLinks(2) = 0.0661;
lenLinks(3) = 0.4251;
lenLinks(4) = 0.3922;
lenLinks(5) = 0.1100;
lenLinks(6) = 0.0948;
lenLinks(7) = 0.0750;
J = func_Jacobi(lenLinks(3),lenLinks(4),lenLinks(5),lenLinks(6),lenLinks(7),q(1),q(2),q(3),q(4),q(5),q(6));


%Default state of end effector
ee_currentPos = [-0.00036;-0.1850;0.9865];%All joint angles are zero
ee_currentOr = [1;0;0;0;0;1;0;-1;0];
%Dampled Least squared inverse of Jacobian (DLS)
lambda = 1e-5;


objCUR5 = CUR5();
%Script to connect with V-REP and run simulation to check on Joint
%velocities and Joint positions.
close all;clc;
disp('Program started');
%vrep = remApi('remoteApi','extApi.h'); %Using header
vrep = remApi('remoteApi');
vrep.simxFinish(-1); %Close connections that exist.
clientID = vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
q_target_pos = zeros(6,1);
if clientID > -1
   disp('Connected to V-REP remote API server');
   %Print to V-Rep that Client is connected
   vrep.simxAddStatusbarMessage(clientID,'Server connected to MATLAB',vrep.simx_opmode_blocking);
   [~,Objects_Scene, ~, ~, ObjectsInScene] = vrep.simxGetObjectGroupData(clientID,vrep.sim_appobj_object_type,0,vrep.simx_opmode_blocking); 
   ObjectsInScene = ObjectsInScene(:,1);
   %Fetch the handle for the vision sensor
   h_VisionSensor = Objects_Scene(46);
   h_joints = zeros(6,1);
   ind_joints = [18:3:30,32];
   h_joints = Objects_Scene(ind_joints);
   h_ee = Objects_Scene(43);
   %Enable the syncrhonous mode
   vrep.simxSynchronous(clientID,true)
   
   %Start simulation 
   vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);
  
   
   pause(1);
   counterStep = 0;
   
   K_p = 5;
   q_prev = zeros(6,1);
   
   
   while(1)
       counterStep = counterStep + 1;%increment counter
       %If the mode is not continuous, preset the target point pose
       h_target = Objects_Scene(45);    
       if b_cont == false
          vrep.simxSetObjectPosition(clientID,h_target,-1,[0.1 ;0.14;0.8],vrep.simx_opmode_blocking);
          vrep.simxSetObjectOrientation(clientID,h_target,-1,[0*pi/180,-90*pi/180,0*pi/180],vrep.simx_opmode_blocking); 
       end
       %Fetch the target pose
       
       [rtrn,ee_targetPos] = vrep.simxGetObjectPosition(clientID,h_target,-1,vrep.simx_opmode_blocking);
       ee_targetPos = ee_targetPos';
       [rtrn,ee_targetAngles] = vrep.simxGetObjectOrientation(clientID,h_target,-1,vrep.simx_opmode_blocking);
       ee_targetOr = reshape(angle2dcm(ee_targetAngles(1),ee_targetAngles(2),ee_targetAngles(3),'XYZ')',9,1);
       %Fetch image and match features       
       
       [rtrn_code,arr_size,mat_image] = vrep.simxGetVisionSensorImage2(clientID,h_VisionSensor,0,vrep.simx_opmode_blocking);
       mat_image = rgb2gray(mat_image);
       corners2 = corner(mat_image,4);
       error = corners1 - corners2;
       
       %Fetch the end-effector pose
       [rtrn,ee_pos] = vrep.simxGetObjectPosition(clientID,h_ee,-1,vrep.simx_opmode_blocking);
       if rtrn == 0
          objCUR5.vec_curr_ee_pos = ee_pos'; 
       end
       [rtrn,ee_anglesVREP] = vrep.simxGetObjectOrientation(clientID,h_ee,-1,vrep.simx_opmode_blocking);
       if rtrn == 0
          objCUR5.vec_curr_ee_or = angle2dcm(ee_anglesVREP(1),ee_anglesVREP(2),ee_anglesVREP(3),'XYZ')';       
       end
       
       %Fetch the joint positions
       for iCount = 1:6
          [rtrn,q(iCount)] = vrep.simxGetJointPosition(clientID,h_joints(iCount),vrep.simx_opmode_blocking); 
          if rtrn == 0
             objCUR5.vec_q_true = q(iCount); 
          end
       end   
       q = q*180/pi;  
       objCUR5.vec_q_true = q;
       dq = (objCUR5.vec_q_true - q_prev)/0.05;%Single difference
       q_prev = objCUR5.vec_q_true;
       
       %Compute Jacobian
       CUR5.mat_curr_J = objCUR5.h_func_EEJacob(objCUR5.vec_link_len,objCUR5.vec_q_true);
       J_star = CRobot.fn_DLSInverse(CUR5.mat_curr_J);
       %Formulate Control law
       ee_currentPos(:,counterStep) = objCUR5.vec_curr_ee_pos;
       R = objCUR5.vec_curr_ee_or;
       ee_currentOr(:,counterStep) = reshape(R,9,1);       
       q_target_v = J_star*([ee_targetPos-ee_currentPos(:,counterStep);ee_targetOr - ee_currentOr(:,counterStep)]);   
       %Derive v/omega
       tem = CUR5.mat_curr_J*dq;
       dR = reshape(tem(4:12),3,3);
       omegaSO = dR*R';
       objCUR5.vec_ee_Omega = [-omegaSO(2,3);omegaSO(1,3);-omegaSO(1,2)];
       omega_ee(:,counterStep) = objCUR5.vec_ee_Omega;
       objCUR5.vec_ee_V = tem(1:3);
       v_ee(:,counterStep) = objCUR5.vec_ee_V;
       %Fetch velocities from VREP for a check
       [rtrn,linVel(counterStep,:),angVel(counterStep,:)] = vrep.simxGetObjectVelocity(clientID, h_ee,vrep.simx_opmode_blocking);
       
       %Set the Joint positions
       u = K_p*q_target_v*pi/180;
       for iCount = 1:length(h_joints)
            %[rtrn] = vrep.simxSetJointTargetVelocity(clientID,h_joints(iCount),q_target_v(iCount),vrep.simx_opmode_blocking);
            [rtrn] = vrep.simxSetJointTargetVelocity(clientID,h_joints(iCount),u(iCount),vrep.simx_opmode_blocking);
       end
       if rtrn ~= 0
            display('Error in setting joint position');
       end
       
       vrep.simxSynchronousTrigger(clientID); 
       %pause(0.1);       
       
       
          
       error_pos(counterStep) = norm(ee_currentPos(:,counterStep) - ee_targetPos);
       error_or(counterStep) = norm(ee_currentOr(:,counterStep)-ee_targetOr);
       if (b_cont == false)
             
           if ((error_pos(counterStep) < 1e-2) && error_or(counterStep) < 1e-2  ) || counterStep >= 100
                break;
           end
       end
   end
   
   figure;
   subplot(2,1,1);
   hold off;
   plot((1:counterStep)',repmat(ee_targetPos(1),counterStep,1),'linestyle','--');hold all;
   plot((1:counterStep)',repmat(ee_targetPos(2),counterStep,1),'linestyle','--');
   plot((1:counterStep)',repmat(ee_targetPos(3),counterStep,1),'linestyle','--');
   grid on;
   plot(ee_currentPos(1,:),'linewidth',2);
   plot(ee_currentPos(2,:),'linewidth',2);
   plot(ee_currentPos(3,:),'linewidth',2);
   legend('x-set','y-set','z-set','x-traj','y-traj','z-traj');
   subplot(2,1,2);
   hold off;
   plot((1:counterStep)',repmat(ee_targetOr(1),counterStep,1),'linestyle','--');hold all;
   plot((1:counterStep)',repmat(ee_targetOr(2),counterStep,1),'linestyle','--');
   plot((1:counterStep)',repmat(ee_targetOr(3),counterStep,1),'linestyle','--');
   plot(ee_currentOr(1,:),'linewidth',2);
   plot(ee_currentOr(2,:),'linewidth',2);
   plot(ee_currentOr(3,:),'linewidth',2);
   legend('z-x-set','z-y-set','z-z-set','z-x-traj','z-y-traj','z-z-traj');
   
   figure;
   subplot(2,1,1);
   plot(error_pos);
   grid on;
   title('error in position');
   subplot(2,1,2);   
   plot(error_or);
   grid on;
   title('error in orientation');
   
   figure;
   subplot(2,1,1);
   plot(v_ee(1,:));hold all;
   plot(v_ee(2,:));
   plot(v_ee(3,:));
   title('velocity (lin.) of end-effector');
   legend('v_x','v_y','v_z');
   subplot(2,1,2);
   plot(omega_ee(1,:));hold all;
   plot(omega_ee(2,:));
   plot(omega_ee(3,:));
   title('velocity (ang.) of end-effector');
   legend('\omega_x','\omega_y','\omega_z');
   
   figure;
   subplot(2,1,1);
   plot(linVel(:,1)');hold all;
   plot(linVel(:,2)');
   plot(linVel(:,3)');
   title('velocity (lin.) of end-effector (VREP)');
   legend('v_x','v_y','v_z');
   subplot(2,1,2);
   plot(angVel(:,1)');hold all;
   plot(angVel(:,2)');
   plot(angVel(:,3)');
   title('velocity (ang.) of end-effector (VREP)');
   legend('\omega_x','\omega_y','\omega_z');
   
   [rtrn,ee_pos] = vrep.simxGetObjectPosition(clientID,h_ee,-1,vrep.simx_opmode_blocking);
   [rtrn,ee_anglesVREP] = vrep.simxGetObjectOrientation(clientID,h_ee,-1,vrep.simx_opmode_blocking);
   ee_anglesVREP = ee_anglesVREP*180/pi;
   if rtrn ~= 0
       display('Error in fetching ee position');
   end
   %Stop simulation
   vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);
   
   %Close connection
   vrep.simxFinish(clientID);
end
vrep.delete();

