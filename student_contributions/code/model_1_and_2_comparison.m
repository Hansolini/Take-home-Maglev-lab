% Load parameters
parameters_maggy_V2;

% Nikos data
Z = [0.07325, 0.07425, 0.07525, 0.07625, 0.07725, 0.07825, 0.07925, 0.08025, 0.08125, 0.08225, 0.08325, 0.08425, 0.08525, 0.08625, 0.08725, 0.08825, 0.08925, 0.09025, 0.09125, 0.09225, 0.09325, 0.09425, 0.09525, 0.09625, 0.09725, 0.09825, 0.09925, 0.10025, 0.10125, 0.10225, 0.10325, 0.10425, 0.10525, 0.10625, 0.10725, 0.10825, 0.10925, 0.11025, 0.11125, 0.11225, 0.11325, 0.11425, 0.11525, 0.11625, 0.11725, 0.11825, 0.11925, 0.12025, 0.12125, 0.12225, 0.12325, 0.12425, 0.12525, 0.12625, 0.12725, 0.12825, 0.12925, 0.13025, 0.13125, 0.13225, 0.13325, 0.13425, 0.13525, 0.13625, 0.13725, 0.13825, 0.13925, 0.14025, 0.14125, 0.14225, 0.14325, 0.14425, 0.14525, 0.14625, 0.14725, 0.14825, 0.14925, 0.15025, 0.15125, 0.15225, 0.15325, 0.15425, 0.15525, 0.15625, 0.15725, 0.15825, 0.15925, 0.16025, 0.16125, 0.16225, 0.16325, 0.16425, 0.16525, 0.16625, 0.16725, 0.16825, 0.16925, 0.17025, 0.17125, 0.17225, 0.17325, 0.17425, 0.17525, 0.17625, 0.17725, 0.17825, 0.17925, 0.18025, 0.18125, 0.18225, 0.18325, 0.18425, 0.18525, 0.18625, 0.18725, 0.18825, 0.18925, 0.19025, 0.19125, 0.19225, 0.19325, 0.19425, 0.19525, 0.19625, 0.19725, 0.19825, 0.19925, 0.20025, 0.20125, 0.20225, 0.20325, 0.20425, 0.20525, 0.20625, 0.20725, 0.20825, 0.20925, 0.21025, 0.21125, 0.21225, 0.21325, 0.21425, 0.21525, 0.21625, 0.21725, 0.21825, 0.21925, 0.22025, 0.22125, 0.22225];
Fz_measured = [0.03179421, 0.0316863, 0.03224547, 0.03333438, 0.03606156, 0.0390438, 0.04313457, 0.04696047, 0.05013891, 0.05364108, 0.05713344, 0.06037074, 0.06310773, 0.06546213, 0.06773805, 0.06985701, 0.0718092, 0.07345728, 0.07476201, 0.0760275, 0.07718508, 0.07815627, 0.07894107, 0.07950024, 0.08003979, 0.08049105, 0.08076573, 0.08094231, 0.08099136, 0.08095212, 0.08089326, 0.08072649, 0.08049105, 0.08013789, 0.07983378, 0.0793629, 0.07881354, 0.0783819, 0.07785216, 0.07724394, 0.0766161, 0.0759294, 0.07522308, 0.07459524, 0.07382025, 0.07302564, 0.07225065, 0.07145604, 0.07069086, 0.06988644, 0.06903297, 0.06815988, 0.06735546, 0.06659028, 0.06571719, 0.06481467, 0.06392196, 0.0630783, 0.0622935, 0.06146946, 0.0605277, 0.05970366, 0.05885019, 0.05806539, 0.05728059, 0.05639769, 0.05551479, 0.05475942, 0.05402367, 0.05323887, 0.05241483, 0.05157117, 0.05082561, 0.0501291, 0.04935411, 0.04857912, 0.04782375, 0.04709781, 0.04642092, 0.04572441, 0.04501809, 0.04427253, 0.0436545, 0.04299723, 0.04233015, 0.04163364, 0.04093713, 0.04033872, 0.03975012, 0.0391419, 0.03852387, 0.0378666, 0.03731724, 0.03673845, 0.03616947, 0.03555144, 0.03497265, 0.03435462, 0.03389355, 0.03334419, 0.03272616, 0.03221604, 0.0316863, 0.03123504, 0.0308034, 0.03029328, 0.0298224, 0.02935152, 0.02895912, 0.02852748, 0.02803698, 0.02757591, 0.0271737, 0.02676168, 0.02633985, 0.02592783, 0.02552562, 0.0251136, 0.02477025, 0.02441709, 0.02400507, 0.02362248, 0.0232497, 0.02293578, 0.02258262, 0.02222946, 0.02186649, 0.02154276, 0.02126808, 0.02094435, 0.02063043, 0.0203067, 0.02000259, 0.01976715, 0.01948266, 0.01914912, 0.01886463, 0.01859976, 0.01836432, 0.01813869, 0.017658, 0.01758933, 0.0173637, 0.01711845, 0.01686339, 0.01662795, 0.01641213, 0.0161865, 0.01601973, 0.01584315];

% Adjusting parameters (to speed up computation)
paramsOuter = params;
paramsOuter.solenoids.r = []; paramsOuter.solenoids.x = []; paramsOuter.solenoids.y = []; paramsOuter.solenoids.z = [];

paramsOuter.magnet.r = 0.015/2;
paramsOuter.magnet.l = 0.005;
paramsOuter.magnet.J = 6261*paramsOuter.physical.mu0/paramsOuter.magnet.l; % 5106 A

% Outer magnet
paramsOuter.permanent.x = 0; paramsOuter.permanent.y = 0; paramsOuter.permanent.z = 0;%0.005/2;

paramsOuter.permanent.l = 0.0225;
paramsOuter.permanent.r = 0.165/2;
paramsOuter.permanent.J = 5106*paramsOuter.physical.mu0/paramsOuter.permanent.l; % 6261 A (from Nikos)

% Inner magnet
paramsInner = paramsOuter;

paramsInner.permanent.r = 0.080/2;
paramsInner.permanent.J = -5106*paramsInner.physical.mu0/paramsInner.permanent.l; % 6261 A (from Nikos)

%% Setup system equations
f_fast = @(z) computeForce(z,paramsOuter,'fast') + computeForce(z,paramsInner,'fast');
f_accurate = @(z) computeForce(z,paramsOuter,'accurate') + computeForce(z,paramsInner,'accurate');

% Sample forces
Fz_fast = zeros(size(Z));
Fz_accurate = zeros(size(Z));
for i = 1:length(Z)
    disp(i)
    Fz_fast(i) = f_fast(Z(i));
    Fz_accurate(i) = f_accurate(Z(i));
end

% Figure
figure(1);
clf; grid on; hold on; box on;
plot(Z,Fz_measured,'b','linewidth',2,'DisplayName','Measured')
plot(Z,Fz_fast,'r','linewidth',2,'DisplayName','Single wire model')
plot(Z,Fz_accurate,'g--','linewidth',2,'DisplayName','Current sheet model')
xlim([0.073,0.22])

xlabel('$z$','FontSize',24);
ylabel('$F_z$','FontSize',24);
legend('location','best','FontSize',16)

%% Functions
function fz = computeForce(z,params,modelName)
    u = zeros(size(params.solenoids.l));
    x = [0;0;z;zeros(9,1)];
    [~,~,fz] = computeForceAndTorque(x,u,params,modelName);
end