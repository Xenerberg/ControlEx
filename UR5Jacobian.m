clear all;
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

ee_targetPos = [0.5;-0.34;0.2];
ee_currentPos = [-0.00036;-0.1850;0.9865];%All joint angles are zero

ee_targetOr =  reshape(angle2dcm(-110*pi/180,20*pi/180,70*pi/180,'XYZ')',9,1);
ee_currentOr = [1;0;0;0;0;1;0;-1;0];
%Dampled Least squared inverse of Jacobian (DLS)
lambda = 1e-5;
J_star = J'*inv(J*J' + lambda*eye(12,12));
q_target_v = J_star*([ee_targetPos-ee_currentPos;ee_targetOr - ee_currentOr]);

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
   q_target_pos = q_target_pos + q_target_v*0.05;
   K_p = 5;
   while(1)
       counterStep = counterStep + 1;%increment counter
       
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
       [rtrn,ee_pos] = vrep.simxGetObjectPosition(clientID,h_ee,-1,vrep.simx_opmode_blocking);
       [rtrn,ee_anglesVREP] = vrep.simxGetObjectOrientation(clientID,h_ee,-1,vrep.simx_opmode_blocking);
       
       for iCount = 1:6
          [rtrn,q(iCount)] = vrep.simxGetJointPosition(clientID,h_joints(iCount),vrep.simx_opmode_blocking); 
       end
       %q = q_target_pos;
       q = q*180/pi;
       %T_F = computeT(lenLinks(1),lenLinks(2),lenLinks(3),lenLinks(4),lenLinks(5),lenLinks(6),lenLinks(7),q(1),q(2),q(3),q(4),q(5),q(6));
       
       J = func_Jacobi(lenLinks(3),lenLinks(4),lenLinks(5),lenLinks(6),lenLinks(7),q(1),q(2),q(3),q(4),q(5),q(6));
       J_star = J'*inv(J*J' + lambda*eye(12,12));
       %ee_pos_TF = T_F(1:3,4)';
       %ee_targetPos = [0.2485;0.4021;0.6405];
       ee_currentPos(:,counterStep) = ee_pos';%All joint angles are zero
       %ee_targetOr = [0.9384;0.3345;0.0868];
       R = angle2dcm(ee_anglesVREP(1),ee_anglesVREP(2),ee_anglesVREP(3),'XYZ')';
       ee_currentOr(:,counterStep) = reshape(R,9,1);
       
       q_target_v = J_star*([ee_targetPos-ee_currentPos(:,counterStep);ee_targetOr - ee_currentOr(:,counterStep)]);       
       error_pos(counterStep) = norm(ee_currentPos(:,counterStep) - ee_targetPos);
       error_or(counterStep) = norm(ee_currentOr(:,counterStep)-ee_targetOr);
       if ((error_pos(counterStep) < 1e-3) && error_or(counterStep) < 1e-2  ) || counterStep >= 100
           break;
       end
   end
   
   figure;
   subplot(2,1,1);
   hold off;
   plot((1:counterStep)',repmat(ee_targetPos(1),counterStep,1),'line','--');hold all;
   plot((1:counterStep)',repmat(ee_targetPos(2),counterStep,1),'line','--');
   plot((1:counterStep)',repmat(ee_targetPos(3),counterStep,1),'line','--');
   grid on;
   plot(ee_currentPos(1,:),'linewidth',2);
   plot(ee_currentPos(2,:),'linewidth',2);
   plot(ee_currentPos(3,:),'linewidth',2);
   legend('x-set','y-set','z-set','x-traj','y-traj','z-traj');
   subplot(2,1,2);
   hold off;
   plot((1:counterStep)',repmat(ee_targetOr(1),counterStep,1),'line','--');hold all;
   plot((1:counterStep)',repmat(ee_targetOr(2),counterStep,1),'line','--');
   plot((1:counterStep)',repmat(ee_targetOr(3),counterStep,1),'line','--');
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

