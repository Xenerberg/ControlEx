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
   
   %Enable the syncrhonous mode
   vrep.simxSynchronous(clientID,true)
   
   %Start simulation 
   vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);
   
   %Compute link lengths
   len_links = zeros(8,1);
   pos = zeros(20,3);
   for iCount = 1:20
      handle = Objects_Scene(iCount+41);
      [res, pos(iCount,:)] = vrep.simxGetObjectPosition(clientID,handle,-1,vrep.simx_opmode_blocking);
   end
   len_links(1) = 0.0935
   len_links(2) = 0.1273;
   len_links(3) = 0.4251;
   len_links(4) = 0.1214;
   len_links(5) = 0.3922;
   len_links(6) = 0.09080;
   len_links(7) = 0.0954;
   len_links(8) = 0.0786;
   
   %Set the LBR_target dummy object to a location
   vrep.simxSetObjectPosition(clientID,Objects_Scene(39),-1,[-0.2;0.2;0.5],vrep.simx_opmode_oneshot);
   pause(0.1);
   %Initialize Joint matrix
   timeToSee = 100;
   JPos = zeros(7,timeToSee);
   %Run simulation as many times
   for iCount = 1:timeToSee      
      %7 joints of the LBR
      for iJoints = 1:7
          [rtn_msg,temp] = vrep.simxGetJointPosition(clientID,Objects_Scene(18 + (iJoints-1)*3),vrep.simx_opmode_oneshot);
          if rtn_msg ~= 0
             disp('Error in invocation to server');             
             temp = NaN;
          end
          JPos(iJoints,iCount) = temp;
          
          
      end
      vrep.simxSynchronousTrigger(clientID);       
      pause(0.1);
   end
   %Stop simulation
       vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);
   
   %Close connection
   vrep.simxFinish(clientID);
end
vrep.delete();


figure;
plot(JPos'*180/pi,'linewidth',3);
xlabel('time');
ylabel('Angle (deg)');
legend('J1','J2','J3','J4','J5','J6','J7');
title('Joint Positions');
