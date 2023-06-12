%% SC_MPIServer matlab file

function SC_MPIServer_Mat(instance) %same name as file


% Create connection
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


%% Start values from script Jean

% Load the yaw setpoint LUT and set-up a simple function
nTurbs = 3;

% Initial control settings
degRad=pi()/180.0;
bladePitchFine = 0.013090000000/degRad; %From ROSCO [deg]
torqueArrayOut     = 0.0 *ones(1,nTurbs); % Not used unless 'torqueSC' set in turbineProperties
yawAngleArrayOut   = 120.*ones(1,nTurbs); % Not used unless 'yawSC' set in turbineProperties
pitchAngleArrayOut = bladePitchFine *ones(1,nTurbs); % Not used unless 'PIDSC' or 'pitchSC' set in turbineProperties

% Reference signal
dt = 0.50;

%%%%%%%%%% added reference signal
load('AGCdata')
initialtime=20000; 
dt = 0.50;
timeSetpointArray = [initialtime+dt:dt:initialtime+1000];
powerSetpointFarmArray=zeros(1,size(timeSetpointArray,2));
for ii=1:(size(timeSetpointArray,2))
if timeSetpointArray(ii)>20300
    powerSetpointFarmArray(ii) = interp1(AGCdata(:,1),AGCdata(:,2),(timeSetpointArray(ii)-initialtime+114),'linear');
end
end
powerSetpointFarmArray=powerSetpointFarmArray.*(nTurbs*0.5e6)+(nTurbs*3.5e6);
%plot(timeSetpointArray,powerSetpointFarmArray./nTurbs);
%%%%%%%%%%% added reference signal

% Setup wind farm power controller
applyFarmPowerControl = true;
trackingErrorInt = 0;
wfcPidGainKP = 0;
wfcPidGainKI = 1 / (2*nTurbs*dt);
powerSetpointVariationPowerControl=zeros(1,nTurbs);
turbIsGreedy = zeros(1,nTurbs);

%DTU 10MW parameters
powerRated = 10e6; %[W]
genEfficiency = 1.0; % Generator efficiency for the NREL 5MW turbine
Rr=89.2;
gbRatio=50;
fluidDensity=1.23;
RpmtoRad=9.5493;

%Pitch control parameters
InitializePitchControl1(1:nTurbs)=true;
InitializePitchControl2(1:nTurbs)=true;
rpmRadSec=2*pi()/60;
PitchMin = bladePitchFine; %[deg]
PitchMax = 90; %[deg]
PitchSwitch = 0.01745329; %[rad]
PitchSwitch = PitchSwitch/degRad; %[deg]
load('controlTables/gainSchedulingPitch_DTU10MW_ROSCO.mat')
load('controlTables/rotorSpeedInterpolant_DTU10MW_ROSCO_constt.mat')

%Torque control [Based on Jonkman 2009 and ROSCO]
VS_RtPwr = powerRated/genEfficiency; %Rated mechanical power in Region 3
KGen = 79.43986000000; % From ROSCO [Nm/((rad/s)^2)]
VS_CtInSp = 200.0/RpmtoRad; % Transitional generator speed (HSS side) bet [rad/s]
%Region 1.1/2 is defined to span the range of generator speed between 200
%rpm and 50% above this value (or 300rpm)
VS_Rgn2Sp = 300.0/RpmtoRad; %Transitional generator speed (HSS side) bet [rad/s]
VS_Rgn3MP=0.01745329; % Minimum pitch angle at which the torque is tracking [rad]
VS_Slope15 = ( KGen*VS_Rgn2Sp*VS_Rgn2Sp )/( VS_Rgn2Sp - VS_CtInSp );
VS_TrGnSp = 405.0/RpmtoRad; %[rad/s]
VS_SlPc = 10.0; % Rated generator slip percentage in Region 2
VS_SySp = (VS_TrGnSp)/( 1.0 + 0.01*VS_SlPc ); %Synchronous speed of region 2 1/2 induction [rad/s]
Region2EndGenTorque=KGen*VS_TrGnSp*VS_TrGnSp;
VS_Slope25 = Region2EndGenTorque/( VS_TrGnSp  - VS_SySp );
Affine_GenTorque= Region2EndGenTorque-VS_Slope25*VS_TrGnSp; %from the VS_Slope25 the affine parameter with the VS_slope25
VS_RtGnSp = (-Affine_GenTorque+sqrt((Affine_GenTorque^2)+4*VS_Slope25*0.95*VS_RtPwr))/(2*VS_Slope25); %Rated generator speed (HSS side) [rad/s]
PC_RefSpd = (1/0.95)*VS_RtGnSp;
RatedGenTorque=VS_RtPwr/PC_RefSpd; %[Nm]
VS_MaxTq = 250000.0; %Maximum generator torque in Region 3 (HSS side) [Nm]
rotorSpeedRated= PC_RefSpd/gbRatio; %[rad/s]
downRegulationMode=2;

% Initialization of filter for the blade pitch measurements
betaf=0.961; %cutfrequency of 0.4rad/s
bladePitchFiltered(1:nTurbs)=bladePitchFine*ones(1,nTurbs);




%% Start control loop
disp(['Entering controller loop...']);

while true % Farm-level loop
    incomplete=true;
    new=true;
    disp(['Farm-level loop: ', num2str(i)]);
    i=i+1;
    while incomplete % Turbine-level loop, wait for all turbines to complete farm-level timestep. Checking for completion is done internally in dll.
        [iT,avrSWAP,incomplete]=MPIServer_AnyReceive(avrSWAP);  

        % Measurements: Receive data from FAST.Farm
        ii = iT;
        currentTime = avrSWAP(iT,2) + 20000; %in seconds
        rotorSpeedArray = avrSWAP(:,21); %in rad/s
        genTorqueArray = avrSWAP(:,23); %in Nm
        bladePitchArray = avrSWAP(:,4) * 180 / pi(); %in deg (original in rad)
        generatorPowerArray = avrSWAP(:,15); %in W 

        if new
            %# < update farm-level control here >
            new=false;
        end
        
        %D_DT_time = 0.2;
        %test = mod(currentTime,D_DT_time);
        %if test == 0

        %# < update turbine-level control here >
	
	        % Update control values for each turbine

            % Update current farm power setpoint and divide between turbines 
            powerSetpointFarmCurrent = interp1(timeSetpointArray,powerSetpointFarmArray,currentTime,'nearest');
            powerSetpointTurbCurrent(ii) = powerSetpointFarmCurrent/nTurbs;
            powerSetpointTurbNC=powerSetpointTurbCurrent;
 
            % Apply wind farm power controller
            if applyFarmPowerControl
                powerSetpointFarmPrevious = interp1(timeSetpointArray,powerSetpointFarmArray,currentTime-dt,'nearest'); 
		        powerErrorPrevious = powerSetpointFarmPrevious - sum(generatorPowerArray);
		        powerSetpointVariationPowerControl=0;
		        if (currentTime > 20100)
                    % Freeze integrator error if all turbines are saturated and setpoint not going down
                    if ~all(turbIsGreedy) || (powerErrorPrevious < 0) % Allow increase if not all turbines greedy, also allow if power setpoint goes down
                        trackingErrorInt = trackingErrorInt + dt * powerErrorPrevious;
                    end
                % Ask non-greedy controllers to compensate for losses elsewhere
                powerSetpointVariationPowerControl = wfcPidGainKP*powerErrorPrevious + wfcPidGainKI * trackingErrorInt;
		        end
                powerSetpointTurbCurrent = powerSetpointTurbCurrent + powerSetpointVariationPowerControl;
            end
            powerSetpointTurbNTC=powerSetpointTurbCurrent;


	            %% LP Filter for the blade pitch measurements
		        bladePitchFiltered(ii)= bladePitchFiltered(ii)*betaf + (1-betaf)*bladePitchArray(ii);

		        rotorSpeedReference(ii) = rotorSpeedInterpolant_DTU10MW(powerSetpointTurbCurrent(ii));    

                if (powerSetpointTurbCurrent(ii)>VS_RtPwr) || (rotorSpeedArray(ii)<rotorSpeedReference(ii) && bladePitchArray(ii)<PitchSwitch)
                    turbIsGreedy(ii) = true;
			        disp(['Turbine is greedy.'])
			        % TORQUE CONTROLLER
                    genSpeed = rotorSpeedArray(ii) * gbRatio; % in rad/s
                    torqueGreedy   = KGen * (genSpeed.^2); % Greedy control signal (Does Kgen is set fot genSpeed or rotSpeed?)
                    %torqueTracking =  VS_RtPwr / (genSpeed*genEfficiency); % Tracking control signal  
                    torqueTracking =  RatedGenTorque; % Constant torque [also need to change the transition
                        if (genSpeed > VS_RtGnSp)
                            disp(['Current torque control mode: Region 3.']);
                            torqueArrayOut(ii) = torqueTracking;
                        elseif genSpeed< (VS_CtInSp)
                            torqueArrayOut(ii)=0;
                            disp(['Current torque control mode: Region 1.']);
                        elseif genSpeed< (VS_Rgn2Sp)   
                            torqueArrayOut(ii)= VS_Slope15*( genSpeed - VS_CtInSp);
                            disp(['Current torque control mode: Region 1.1/2.']);
                        elseif genSpeed< (VS_TrGnSp)
                            disp(['Current torque control mode: Region 2.']);
                            torqueArrayOut(ii) = torqueGreedy; % Perfect tracking whenever possible, otherwise fall back on greedy
                        else
                            disp(['Current torque control mode: Region 2.1/2.']);
                            torqueArrayOut(ii)= Region2EndGenTorque + VS_Slope25*( genSpeed - VS_TrGnSp);
                        end
               
				        % BLADE PITCH CONTROLLER - Gain-scheduling PID CONTROL
                        if rotorSpeedArray(ii)<rotorSpeedRated &&  bladePitchArray(ii)<PitchSwitch
                            pitchAngleArrayOut(ii)=bladePitchFine;
					        InitializePitchControl1(ii)=true;
                        else
					        if InitializePitchControl1(ii)
							        PitchControlKI(ii) = -Ki(bladePitchArray(ii)*degRad)*gbRatio;
							        intSpeedError(ii) = (bladePitchArray(ii)*degRad) / PitchControlKI(ii);
							        speedErrorLast(ii)= 0;
						        InitializePitchControl1(ii)=false;
					        end
				        %Compute the gains
				        PitchControlKP(ii)= -Kp(bladePitchFiltered(ii)*degRad)*gbRatio;
				        PitchControlKI(ii)= -Ki(bladePitchFiltered(ii)*degRad)*gbRatio;
				        PitchControlKD(ii)= -Kd(bladePitchFiltered(ii)*degRad)*gbRatio;
				        %Compute the low speed shaft speed error.
				        speedError(ii)= rotorSpeedArray(ii)- rotorSpeedRated; %[rad/s]
				        %Numerically integrate the speed error over time.
				        intSpeedError(ii)= intSpeedError(ii)+speedError(ii)*dt;
				        %Numerically take the deriviative of speed error w.r.t time.
				        derivSpeedError(ii) = (speedError(ii) - speedErrorLast(ii)) / dt;
				        %Store the old value of speed error.
				        speedErrorLast(ii)=speedError(ii);
				        %Saturate the integrated speed error based on pitch saturation.
				        intSpeedError(ii) = min(max(intSpeedError(ii), PitchMin*degRad/PitchControlKI(ii)), PitchMax*degRad/PitchControlKI(ii));
				        %Compute the pitch components from the proportional, integral, and derivative parts and sum them.
				        pitchP =  PitchControlKP(ii)* speedError(ii); %[rad]
				        pitchI =  PitchControlKI(ii)* intSpeedError(ii); %[rad]
				        pitchD =  PitchControlKD(ii)* derivSpeedError(ii); %[rad]
				        pitchAngleArrayOut(ii)= (pitchP + pitchI + pitchD) / degRad; %[deg]
				        pitchAngleArrayOut(ii)= min(max(pitchAngleArrayOut(ii), PitchMin), PitchMax);
                        end
                        InitializePitchControl2(ii)=true;
                else
                  %POWER TRACKING ALGORITHM (KNU2) 
			        turbIsGreedy(ii) = false;
			        disp(['Turbine is tracking power.'])
			        %PITCH POWER REFERENCE CONTROL
                        rotorSpeedReference(ii) = min(rotorSpeedReference(ii),rotorSpeedRated);
					        if InitializePitchControl2(ii)    
							        PitchControlKI(ii) = -Ki(bladePitchArray(ii)*degRad)*gbRatio;
							        intSpeedError(ii) = (bladePitchArray(ii)*degRad) / PitchControlKI(ii);
							        speedErrorLast(ii)= 0;
						        InitializePitchControl2(ii)=false;
					        end
				        %Compute the gains
				        PitchControlKP(ii)= -Kp(bladePitchFiltered(ii)*degRad)*gbRatio;
				        PitchControlKI(ii)= -Ki(bladePitchFiltered(ii)*degRad)*gbRatio;
				        PitchControlKD(ii)= -Kd(bladePitchFiltered(ii)*degRad)*gbRatio;
				        %Compute the low speed shaft speed error.
				        speedError(ii)= rotorSpeedArray(ii)- rotorSpeedReference(ii); %[rad/s]
				        %Numerically integrate the speed error over time.
				        intSpeedError(ii)= intSpeedError(ii)+speedError(ii)*dt;
				        %Numerically take the deriviative of speed error w.r.t time.
				        derivSpeedError(ii) = (speedError(ii) - speedErrorLast(ii)) / dt;
				        %Store the old value of speed error.
				        speedErrorLast(ii)=speedError(ii);
				        %Saturate the integrated speed error based on pitch saturation.
				        intSpeedError(ii) = min(max(intSpeedError(ii), PitchMin*degRad/PitchControlKI(ii)), PitchMax*degRad/PitchControlKI(ii));
				        %Compute the pitch components from the proportional, integral, and derivative parts and sum them.
				        pitchP =  PitchControlKP(ii)* speedError(ii); %[rad]
				        pitchI =  PitchControlKI(ii)* intSpeedError(ii); %[rad]
				        pitchD =  PitchControlKD(ii)* derivSpeedError(ii); %[rad]
				        pitchAngleArrayOut(ii)= (pitchP + pitchI + pitchD) / degRad; %[deg]
				        pitchAngleArrayOut(ii)= min(max(pitchAngleArrayOut(ii), PitchMin), PitchMax);       
			        
			        % TORQUE CONTROLLER
			        genSpeed = rotorSpeedArray(ii) * gbRatio; % in rad/s
                    torqueTracking =  powerSetpointTurbCurrent(ii) / (genSpeed*genEfficiency); % Tracking control signal  
			        if downRegulationMode==1
                        torqueArrayOut(ii)=torqueTracking;
                    elseif downRegulationMode==2
                        torqueGreedy   = KGen * (genSpeed.^2); % Greedy control signal (Does Kgen is set fot genSpeed or rotSpeed?  
                        if (genSpeed > VS_RtGnSp)
                            disp(['Current torque control mode: Region 3.']);
                            torqueOutConv = RatedGenTorque; % Constant torque [also need to change the transition]
                        elseif genSpeed< (VS_CtInSp)
                            torqueOutConv=0;
                            disp(['Current torque control mode: Region 1.']);
                        elseif genSpeed< (VS_Rgn2Sp)   
                            torqueOutConv= VS_Slope15*( genSpeed - VS_CtInSp);
                            disp(['Current torque control mode: Region 1.1/2.']);
                        elseif genSpeed< (VS_TrGnSp)
                            disp(['Current torque control mode: Region 2.']);
                            torqueOutConv = torqueGreedy; % Perfect tracking whenever possible, otherwise fall back on greedy
                        else
                            disp(['Current torque control mode: Region 2.1/2.']);
                            torqueOutConv= Region2EndGenTorque + VS_Slope25*( genSpeed - VS_TrGnSp);
                        end
                         torqueArrayOut(ii) = min( torqueTracking, torqueOutConv); %see more details in Silva 2022 at TORQUE paper
                        if torqueArrayOut(ii)==torqueTracking
                            disp(['Torque tracking has been applied']);
                        else
                            disp(['Torque greedy has been applied']);
                        end
                    else
                        disp(['The down-regulation mode is not found.']);
                        return
                    end
			        InitializePitchControl1(ii)=true;
		        end
		        
		        %Saturate using the maximum torque limit
                if (min(torqueArrayOut(ii),VS_MaxTq)==VS_MaxTq)
                    torqueArrayOut(ii)=VS_MaxTq;
                    disp(['Generator torque reaches maximum!']);
                end 
		        
		        % RATE LIMITERS
                applyRateLimiterTorque = true;
                if applyRateLimiterTorque
                    torqueRateLimit = 15.0e3 * dt;
                    deltaTorque = torqueArrayOut(ii) - genTorqueArray(ii);
                    deltaTorque = max(min(deltaTorque,torqueRateLimit),-torqueRateLimit);
                    torqueArrayOut(ii) = genTorqueArray(ii) + deltaTorque;
                end
                %%{
                applyRateLimiterPitch = true;
                if applyRateLimiterPitch
                    pitchRateLimit = 2.5 * dt;
                    deltaPitch = pitchAngleArrayOut(ii) - bladePitchArray(ii);
                    deltaPitch = max(min(deltaPitch,pitchRateLimit),-pitchRateLimit);
                    pitchAngleArrayOut(ii) = bladePitchArray(ii) + deltaPitch;
                end		

            avrSWAP(ii,47) = torqueArrayOut(ii); %Generator torque in Nm
            avrSWAP(ii,45) = pitchAngleArrayOut(ii) / 180 * pi(); %Blade pitch angle in rad (original in deg)
        


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
% T1out = 'C:\Users\vmart\Documents\Bestanden_Coen\Bundle_UPDATE\Test3turbines_DTUWEC\FAST.Farm_N3.T1.out';
% T2out = 'C:\Users\vmart\Documents\Bestanden_Coen\Bundle_UPDATE\Test3turbines_DTUWEC\FAST.Farm_N3.T2.out';
% T3out = 'C:\Users\vmart\Documents\Bestanden_Coen\Bundle_UPDATE\Test3turbines_DTUWEC\FAST.Farm_N3.T3.out';
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
% PlotFASToutput({T1out, T2out, T3out},{'Turbine 1', 'Turbine 2', 'Turbine 3'});
