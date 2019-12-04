clc;

opt = genfisOptions('GridPartition');
opt.NumMembershipFunctions = 5;
InitialFis = genfis(trainingEMG,trainingForce,opt);

InitialFis.MFType = 'gaussMF';
InitialFis.AndMethod = 'prod';

anfisopt = anfisOptions('InitialFis', InitialFis);
anfisopt.EpochNumber = 15;
anfisopt.ValidationData = checkingData;

[Emg2Force,trainErr,~,chk_fis,checkErr] = anfis(trainingData,anfisopt);