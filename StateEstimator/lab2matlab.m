%the length of the box unit is cm
t=1;
L=50;
W=50;
widthdistance=.5;

%initial state x=[x,y,theta,xdot,ydot,thetadot]
x0=input('initial x  ');
y0=input('initial y  ');
angle=input('initial angle  ');
xd=input('initial x dot  ');
yd=input('initial y dot  ');
angled=input('initial angle dot  ');

xk_minus1=[x0;y0;angle;xd;yd;angled]; %x_k-1

%the input from laser sensors and IMU9250 sensor
dFront=input('measurement from front laser  ');
dRight=input('measurement from right laser  ');
angleIMU=input('measurement from IMU  ');

sensor=[dFront;dRight;angleIMU];

%wheel pwm input
pwmR = input('pwm for right wheel  ');
pwmL = input('pwm for left wheel  ');

inputUk=[pwmL;pwmR]; %U_k

%the matrix that use in prediction function
A=[1 0 0 t 0 0;0 1 0 0 t 0; 0 0 1 0 0 t; 0 0 0 0 0 0; 0 0 0 0 0 0;0 0 0 0 0 0];
B=[0 0;0 0;0 0;0.5*cos(angled) 0.5*cos(angled);0.5*sin(angled) 0.5*sin(angled);-0.5*widthdistance -0.5*widthdistance];
P_kminus1=[0 0 0 0 0 0;0 0 0 0 0 0;0 0 0 0 0 0;0 0 0 0 0 0;0 0 0 0 0 0;0 0 0 0 0 0];
Q=[2 0 0 0 0 0; 0 2 0 0 0 0;0 0 2 0 0 0;0 0 0 2 0 0; 0 0 0 0 2 0;0 0 0 0 0 2];

%predict function
X_k_minus=A*xk_minus1 +B*inputUk;
P_k_minus=A*P_kminus1+Q;

%condition for C
x=X_k_minus(1);
y=X_k_minus(2);
theta=X_k_minus(3);

%below calculate the C 
%evaluate H1,
d1=(L-x)/cos(theta);
d2=(W-y)/sin(theta);
d3=x/cos(theta+pi);
d4=y/sin(theta+pi);
distanceValue=[d1 d2 d3 d4];
sortVector=sort(distanceValue);
secondMax=sortVector(2);

if secondMax == d1
   H1 =  [(L/x-1)/cos(theta) 0 0 0 0 0;0 0 0 0 0 0;0 0 0 0 0 0];
elseif secondMax == d2
   H1 =  [0 (W/y-1)/sin(theta) 0 0 0 0;0 0 0 0 0 0;0 0 0 0 0 0];
elseif secondMax ==d3
   H1 =  [1/cos(theta+pi) 0 0 0 0 0;0 0 0 0 0 0;0 0 0 0 0 0];
else    
   H1 =  [0 1/sin(theta+pi) 0 0 0 0;0 0 0 0 0 0;0 0 0 0 0 0];
end

%calculate H2
L1=(L-x)/sin(theta);
L2=(W-y)/cos(theta+pi);
L3=x/sin(theta+pi);
L4=y/cos(theta);
distanceValue2=[L1 L2 L3 L4];
sortVector2=sort(distanceValue2);
secondMax2=sortVector2(2);
if secondMax2 == L1
    H2 = [0 0 0 0 0 0;(L/x-1)/sin(theta) 0 0 0 0 0;0 0 0 0 0 0];
elseif secondMax2 == L2
    H2 = [0 0 0 0 0 0;0 (W/y-1)/cos(theta+pi) 0 0 0 0;0 0 0 0 0 0];
elseif secondMax2 == L3
    H2 = [0 0 0 0 0 0;1/sin(theta+pi) 0 0 0 0 0;0 0 0 0 0 0];
else
    H2 = [0 0 0 0 0 0;0 1/cos(theta) 0 0 0 0;0 0 0 0 0 0];
end

%add H1 and H2 to C
C=H1+H2 +[0 0 0 0 0 0;0 0 0 0 0 0;0 0 1 0 0 0];

%update session
R=[1 0 0;0 1 0;0 0 1];
K_k = P_k_minus*transpose(C)*pinv(C*P_k_minus*transpose(C)+R);
I = [1 0 0 0 0 0;0 1 0 0 0 0;0 0 1 0 0 0;0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1];

%update function
X_k = X_k_minus+K_k*(sensor-C*X_k_minus);
P_k = (I - K_k*C)*P_k_minus;

fprintf('prediction X_k_minus:\n');
disp(X_k_minus);
fprintf('prediction covariance:\n');
disp(P_k_minus);
fprintf('correct update X_k:\n');
disp(X_k);
fprintf('copprect update covariance:\n');
disp(P_k);







