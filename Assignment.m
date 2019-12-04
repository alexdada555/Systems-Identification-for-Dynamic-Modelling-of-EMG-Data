clc;    % Clear Command Window

% The objective of this investigations is to generate from EMG and Force
% data, a model which accurately maps the input the resultant output force.
% This model will then be used to derive to input to a feedback controller
% Which will produced the desired force response
% 
% In the course of this investigation, 3 different methods for relating
% The input EMG signals to the output forces will be utilised and their
% performances compared
% These are Fuzzy Neural Inference, Deep Neural Network, Systems Idenfication
% They will be compared again the provided model which will act at the 
% experimental Control.

%% Import The EMG and Force Data Into The Workspace

load lab2;   

%% Plot the Raw EMG and Force Data

figure(1);
subplot(2,1,1);
plot(emg(:,1),emg(:,2));
xlabel('Time (s)');
ylabel('EMG (V)');
title('Raw EMG');
subplot(2,1,2);
plot(force(:,1),force(:,2),'r');
xlabel('Time (s)');
ylabel('Force (N)');
title('Force, in Newtons');

% This shows the raw emg data and the output force.

%% EMG Signal Processing: Design For A Low Pass Biquad Filter

Fs = 2000;           % Sampling Frequency
Fpass = 2;           % Passband Frequency
Fstop = 10;          % Stopband Frequency
Apass = 1;           % Passband Ripple (dB)
Astop = 80;          % Stopband Attenuation (dB)
match = 'stopband';  % Band to match exactly

% Construct an FDESIGN object and call its BUTTER method.
h  = fdesign.lowpass(Fpass, Fstop, Apass, Astop, Fs);
Hd = design(h, 'butter', 'MatchExactly', match);
 
rEMG = abs(emg(:,2));      % EMG signal rectification
filtEMG = filter(Hd,rEMG); % low pass filter
maxEMG = max(filtEMG);     % max EMG value
normEMG = filtEMG./maxEMG; % signal normalisation

figure(2);
t = emg(:,1); % time index
subplot(2,1,1);
plot(t,normEMG);
xlabel('Time (s)');
ylabel('EMG (no units)');
title('Rectified, Filtered and Normalised EMG');
subplot(2,1,2);
plot(force(:,1),force(:,2),'r');
xlabel('Time (s)');
ylabel('Force (N)');
title('Force, in Newtons');

% Before the EMG Data can be used it most first be digitally processed
% This is done remove the unwanted elements of the signal and to make
% it easier to map to an output function.
% The raw emg data if first rectified as only the positive elements are
% useful, This rectified data is then passed through a series of Low-pass
% Filters in Biquad configuration. The signal is then normalised to fit
% in range 0 to 1.

%% Using ANFIS to Create a Model
size = length(normEMG);

trainingSize = round(size*2/3);
checkingSize = round(size*1/3);
trainingEMG = normEMG(1:trainingSize);
checkingEMG = normEMG(trainingSize:size);
trainingForce = force(1:trainingSize,2);
checkingForce = force(trainingSize:size,2);
trainingData = [trainingEMG trainingForce];
checkingData = [checkingEMG checkingForce];

opt = genfisOptions('GridPartition');
opt.NumMembershipFunctions = 10;

InitialFis = genfis(trainingEMG,trainingForce,opt);
InitialFis.MFType = 'gaussMF';
InitialFis.AndMethod = 'prod';
anfisopt = anfisOptions('InitialFis', InitialFis);
anfisopt.EpochNumber = 20;
anfisopt.ValidationData = checkingData;
[Emg2Force,trainErr,~,chk_fis,checkErr] = anfis(trainingData,anfisopt);

% An Adaptive Fuzzy Network based Inference System or ANFIS is a
% hybrid architecture that combined the interpolating abilities of 
% an Artificial Neural Network to optimise the TSK-based fuzzy rules
% of a fuzy system through the use of input and output data.
% 
% in order to utilise ANFIS input and ouput data most be provided
% in this case the input data is the normalised EMG, and the output
% is the force generated.
% 
% This data is  combined and seprated into 2 sections, training and 
% checking data, in the ration of 2:1 with 2/3s of the data used to 
% train the model and 1/3 used to validate it.

%% Using a Deep Neural Network to Create a Model
NNEmg = normEMG;
NNForce = force(:,2);
[NNY,Xf,Af] = myNeuralNetworkFunction(NNEmg);

%% Run Simulink Model

% Define simulation variables and parameters
simin = emg; % simulink input matrix (two columns)
dt = 0.0005; % sample time
[numM,denM] = tfdata(M,'v'); % EMG-force control model 
[numP,denP] = tfdata(plant,'v'); % Robot finger plant

sim('Assignment2'); % Run the simulink model

% Control Outputs
figure(3);
plot(force(:,1),force(:,2),'b'); 
hold on;
plot(DesiredForce.Time,DesiredForce.Data,'r');
xlabel('Time (s)');
ylabel('Force (N)');
legend('Human Force','Desired Prosthetic Force');
title('Control Model Finger Force');

% Fuzzy Output
figure(4);
plot(force(:,1),force(:,2),'b'); 
hold on;
plot(DesiredForce1.Time,DesiredForce1.Data,'r');
xlabel('Time (s)');
ylabel('Force (N)');
legend('Human Force','Desired Prosthetic Force');
title('Neural Fuzzy Finger Force');

% NN Output
figure(5);
plot(force(:,1),force(:,2),'b'); 
hold on;
plot(DesiredForce2.Time,DesiredForce2.Data,'r');
xlabel('Time (s)');
ylabel('Force (N)');
legend('Human Force','Desired Prosthetic Force');
title('Neural Network Finger Force');

%% Model Performance Metric Evaluations

ControlError = force(1:length(DesiredForce.Data),2) - DesiredForce.Data;
ANFISError = force(1:length(DesiredForce1.Data),2) - DesiredForce1.Data;
NNError = force(1:length(DesiredForce2.Data),2) - DesiredForce2.Data;

MeanAbsErrorControl = mae(ControlError);
MeanAbsErrorANFIS = mae(ANFISError);
MeanAbsErrorNN = mae(NNError);

RootMeanSqErrorControl = sqrt(mean((ControlError).^2));
RootMeanSqErrorANFIS = sqrt(mean((ANFISError).^2));
RootMeanSqErrorNN = sqrt(mean((NNError).^2));

PercentImprovementANFIS = 100*((RootMeanSqErrorControl-RootMeanSqErrorANFIS)/RootMeanSqErrorControl);
PercentImprovementNN = 100*((RootMeanSqErrorControl-RootMeanSqErrorNN)/RootMeanSqErrorControl);
