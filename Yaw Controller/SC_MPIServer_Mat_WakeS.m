%% SC_MPIServer matlab file

function SC_MPIServer_Mat(instance) %same name as file

instance=0;
if instance==0
  MPI_sharedfile="../MPI_shared.dat";
else
  mkdir("../MPI_shared_instances");
  MPI_sharedfile=sprintf("../MPI_shared_instances/MPI_shared_%d.dat",instance);
end
MPI_sharedfile = convertStringsToChars(MPI_sharedfile);

%Initialize
disp("Ready to connect");
[nT,nc]=MPIServer_Init(MPI_sharedfile);
disp("Connected");
avrSWAP=zeros(nT,nc, 'single');
disp(['Size avrSWAP: ', num2str(nT), 'x', num2str(nc)]);
i=1;


%% Run and terminate Loop for starting connection

% Initialise filtered generator speed
GenSpeedF = [0, 0, 0];

while true % Farm-level loop
    incomplete=true;
    new=true;
    disp(['Farm-level loop: ', num2str(i)]);
    i=i+1;
    while incomplete % Turbine-level loop, wait for all turbines to complete farm-level timestep. Checking for completion is done internally in dll.
        [iT,avrSWAP,incomplete]=MPIServer_AnyReceive(avrSWAP);  

        if new
            %# < update farm-level control here >
            
            Time = avrSWAP(iT,2);
            if Time >= 50
                Yaw_angle_comm = [0.52, 0, 0];
            else
                Yaw_angle_comm = [0, 0, 0];
            end
%             if Time >= 1
%                 Yaw_angle_comm = [0.52, 0.52, 0];
%             end
            new=false;

        end
        %# < update turbine-level control here >

%         Time = avrSWAP(iT,2);
%         if Time > 80
%            avrSWAP(iT,47) = 50000;
%         else
%            avrSWAP(iT,47) = 0;
%         end

        %set generator torque
        KGen = 79.43986000000; % From ROSCO [Nm/((rad/s)^2)]
        gbRatio=50;
        Omega = avrSWAP(iT,21); % rotor speed (rad/s) (omega*gb = generator speed)

        CornerFreq = 2.5;
        Alpha = exp(-0.05*CornerFreq);
        GenSpeed = Omega*gbRatio;
        GenSpeedF(iT) = ( 1.0 - Alpha )*GenSpeed + Alpha*GenSpeedF(iT);
        avrSWAP(iT,47) = KGen * (GenSpeedF(iT))^2;

        %simple Yaw controller
        Yaw_angle = avrSWAP(iT,37);
        Yaw_angle_com = Yaw_angle_comm(iT);
        Yaw_error = Yaw_angle_com - Yaw_angle;
        Threshold = 0.0035;

        if Yaw_error > Threshold
            YawRate = 0.005235;
        elseif Yaw_error < -Threshold
            YawRate = -0.005235;
        else
            YawRate = 0;
        end
        avrSWAP(iT,48) = YawRate;      

        MPIServer_OneSend(iT,avrSWAP); % Send back message to turbine iT

    end
    if (all(avrSWAP(:,1)<0))
        save('workspace.mat')
        break
    end
end

disp("Stopping server..");
MPIServer_Stop();
disp("Server stopped.");

end


%% Plotting of results
% T1Gout = 'C:\Users\coenj\OneDrive\Documents\Interface_GitHub\Test3turbines\FAST.Farm_N3.T1.out';
% T2Gout = 'C:\Users\coenj\OneDrive\Documents\Interface_GitHub\Test3turbines\FAST.Farm_N3.T2.out';
% T3Gout = 'C:\Users\coenj\OneDrive\Documents\Interface_GitHub\Test3turbines\FAST.Farm_N3.T3.out';
% 
% %options: [outData]=PlotFASToutput(FASTfiles,FASTfilesDesc,ReferenceFile,Channels,ShowLegend,CustomHdr,PlotPSDs,OnePlot)
% 
% %Plot one file
% PlotFASToutput({T1out},{'Turbine 1'},[],{'GenPwr','YawBrTAxp'});
% 
% %Plot two files in one plot
% PlotFASToutput({T1out, T2out},{'Turbine 1', 'Turbine 2'},[],{'GenPwr','YawBrTAxp'});
% %Plot three files in one plot
% PlotFASToutput({T1out, T2out, T3out},{'Turbine 1', 'Turbine 2', 'Turbine 3'},[],{'GenPwr','YawBrTAxp'});
% PlotFASToutput({T1out, T2out, T3out},{'Turbine 1', 'Turbine 2', 'Turbine 3'},[],{'GenPwr','GenTq','BldPitch1'});
% PlotFASToutput({T1Gout, T2Gout, T3Gout},{'Turbine 1', 'Turbine 2', 'Turbine 3'});

%% old code 
% 
% Time = avrSWAP(1,2);
%         if Time > 80
%             avrSWAP(1,47) = 50000;
%             avrSWAP(2,47) = 50000;
%             avrSWAP(3,47) = 50000;
%         end
% 
%         %simple Yaw controller
%         Yaw_angle = avrSWAP(1,37);
%         Yaw_angle_com = 0.52;
%         Yaw_error = Yaw_angle_com - Yaw_angle;
%         Threshold = 0.0035;
%         Time = avrSWAP(1,2);
% 
%         if Time > 300
%             if Yaw_error > Threshold
%                 YawRate = 0.005235;
%             elseif Yaw_error < -Threshold
%                 YawRate = -0.005235;
%             else
%                 YawRate = 0;
%             end
%             avrSWAP(1,48) = YawRate;
%         end
% 
%         Time2 = avrSWAP(2,2);
%         Yaw_angle2 = avrSWAP(2,37);
%         Yaw_error2 = Yaw_angle_com - Yaw_angle2;
% 
%         if Time2 > 600
%             if Yaw_error2 > Threshold
%                 YawRate2 = 0.005235;
%             elseif Yaw_error2 < -Threshold
%                 YawRate2 = -0.005235;
%             else
%                 YawRate2 = 0;
%             end
%             avrSWAP(2,48) = YawRate2;
%         end        