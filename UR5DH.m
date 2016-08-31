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
q = zeros(6,1);
q(1) = 10;
q(2) = 10;
q(3) = 20;
q(4) = 50;
q(5) = 10;
q(6) = 15;
DH = zeros(6,4);

DH(1,:) = [lenLinks(2)+lenLinks(1),q(1),0,90];
DH(2,:) = [0,q(2)+90,lenLinks(3),0];
DH(3,:) = [0,q(3),lenLinks(4),0];
DH(4,:) = [lenLinks(5),90+q(4),0,90];
DH(5,:) = [lenLinks(6),180+q(5),0,90];
DH(6,:) = [lenLinks(7),q(6),0,0];
R = DH2HTM(DH(1,:))*DH2HTM(DH(2,:))*DH2HTM(DH(3,:))*DH2HTM(DH(4,:))*DH2HTM(DH(5,:))*DH2HTM(DH(6,:));%*DH2HTM(DH(5,:));
R = R(1:3,1:3);
[a,b,c] = dcm2angle(R','XYZ');%Rotation Matrix
ee_angles = [a,b,c]*180/pi;



%Full Transformation
T_1_0 = DH2HTM(DH(1,:));
T_2_1 = DH2HTM(DH(2,:));
T_3_2 = DH2HTM(DH(3,:));
T_4_3 = DH2HTM(DH(4,:));
T_5_4 = DH2HTM(DH(5,:));
T_6_5 = DH2HTM(DH(6,:));
T_F = T_1_0*T_2_1*T_3_2*T_4_3*T_5_4*T_6_5;


%Transforming base vector
ee = T_F*base_vector;%Computed using DH parameters
ee_pos = 0; %Fetched position from model;
%HTM of J2 w.r.t J1

%Script to connect with V-REP and run simulation to check on Joint
%velocities and Joint positions.
close all;clc;
disp('Program started');
%vrep = remApi('remoteApi','extApi.h'); %Using header
vrep = remApi('remoteApi');
vrep.simxFinish(-1); %Close connections that exist.
clientID = vrep.simxStart('127.0.0.1',19997,true,true,5000,5);

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
   %vrep.simxPauseCommunication(clientID,1);
   %Set the Joint positions
   for iCount = 1:length(h_joints)
        [rtrn] = vrep.simxSetJointPosition(clientID,h_joints(iCount),q(iCount)*pi/180,vrep.simx_opmode_blocking);
   end
   %vrep.simxPauseCommunication(clientID,0);
   vrep.simxSynchronousTrigger(clientID); 
   pause(4);
   
   if rtrn ~= 0
       display('Error in setting joint position');
   end
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


fprintf('Computed EE position using DH\n x:%f, y:%f, z:%f\n',ee(1),ee(2),ee(3));
fprintf('Actual EE position from VREP\n x:%f, y:%f, z:%f\n',ee_pos(1),ee_pos(2),ee_pos(3));
fprintf('Computed EE orientation using DH\n x:%f, y:%f, z:%f\n',ee_angles(1),ee_angles(2),ee_angles(3));
fprintf('Actual EE orientation from VREP\n x:%f, y:%f, z:%f\n',ee_anglesVREP(1),ee_anglesVREP(2),ee_anglesVREP(3));
