% bicycle_whipple.m
% KJA, 20 Aug 07
%
% This file contains the parameters that are used for the Whipple
% bicycle model, introduced in Section 3.2 of AM08.  The model is
% based on the linearized 4th order model and analysis of eigenvalues
% from IEEE CSM (25:4) August 2005 pp 26-47

%   Basic data is given by 26 parameters
g = 9.81;			% Acceleration of gravity [m/s^2]
b = 1.00;			% Wheel base [m]
c = 0.08;			% Trail [m]
Rrw = 0.35; Rfw = 0.35;		% Wheel radii
lambda = pi*70/180;		% Head angle [radians]

% Rear frame mass [kg], center of mass [m], and inertia tensor [kgm^2]
mrf=12;xrf=0.439;zrf=0.579;
Jxxrf=0.475656;Jxzrf=0.273996;Jyyrf=1.033092;Jzzrf=0.527436;
mrf=87;xrf=0.491586;zrf=1.028138;
Jxxrf=3.283666;Jxzrf=0.602765;Jyyrf=3.8795952;Jzzrf=0.565929;

% Front frame mass [kg], center of mass [m], and inertia tensor [kgm^2]
mff=2;xff=0.866;zff=0.676;
Jxxff=0.08;Jxzff=-0.02;Jyyff=0.07;Jzzff=0.02;

% Rear wheel mass [kg], center of mass [m], and inertia tensor [kgm^2]
mrw=1.5;Jxxrw=0.07;Jyyrw=0.14;

% Front wheel mass [kg], center of mass [m], and inertia tensor [kgm^2]
mfw=1.5;Jxxfw=0.07;Jyyfw=0.14;

% Auxiliary variables
xrw=0;zrw=Rrw;xfw=b;zfw=Rfw;
Jzzrw=Jxxrw;Jzzfw=Jxxfw;
mt=mrf+mrw+mff+mfw;
xt=(mrf*xrf+mrw*xrw+mff*xff+mfw*xfw)/mt;
zt=(mrf*zrf+mrw*zrw+mff*zff+mfw*zfw)/mt;
Jxxt=Jxxrf+mrf*zrf^2+Jxxrw+mrw*zrw^2+Jxxff+mff*zff^2+Jxxfw+mfw*zfw^2;
Jxzt=Jxzrf+mrf*xrf*zrf+mrw*xrw*zrw+Jxzff+mff*xff*zff+mfw*xfw*zfw;
Jzzt=Jzzrf+mrf*xrf^2+Jzzrw+mrw*xrw^2+Jzzff+mff*xff^2+Jzzfw+mfw*xfw^2;
mf=mff+mfw;
xf=(mff*xff+mfw*xfw)/mf;zf=(mff*zff+mfw*zfw)/mf;
Jxxf=Jxxff+mff*(zff-zf)^2+Jxxfw+mfw*(zfw-zf)^2;
Jxzf=Jxzff+mff*(xff-xf)*(zff-zf)+mfw*(xfw-xf)*(zfw-zf);
Jzzf=Jzzff+mff*(xff-xf)^2+Jzzfw+mfw*(xfw-xf)^2;
d=(xf-b-c)*sin(lambda)+zf*cos(lambda);
Fll=mf*d^2+Jxxf*cos(lambda)^2+2*Jxzf*sin(lambda)*cos(lambda)+Jzzf*sin(lambda)^2;
Flx=mf*d*zf+Jxxf*cos(lambda)+Jxzf*sin(lambda);
Flz=mf*d*xf+Jxzf*cos(lambda)+Jzzf*sin(lambda);
gamma=c*sin(lambda)/b;
Sr=Jyyrw/Rrw;Sf=Jyyfw/Rfw;St=Sr+Sf;Su=mf*d+gamma*mt*xt;

% Matrices for linearized fourth order model
M=[Jxxt -Flx-gamma*Jxzt;-Flx-gamma*Jxzt Fll+2*gamma*Flz+gamma^2*Jzzt];
K0=[-mt*g*zt g*Su;g*Su  -g*Su*cos(lambda)];
K2=[0 -(St+mt*zt)*sin(lambda)/b;0 (Su+Sf*cos(lambda))*sin(lambda)/b];
c12=gamma*St+Sf*sin(lambda)+Jxzt*sin(lambda)/b+gamma*mt*zt;
c22=Flz*sin(lambda)/b+gamma*(Su+Jzzt*sin(lambda)/b);
C0=[0 -c12;(gamma*St+Sf*sin(lambda)) c22]; 
one=diag([1 1]);null=zeros(2,2);

% Nominal velocity 
v0=5;

% Matrices of state model
A=[null one;-M\(K0+K2*v0^2) -M\(C0*v0)];
bm=M\[0;1];
B=[0;0;bm];    
eig(A)'

% Use the place function here to set the eigenvalues of A-B * K 
p1 = [-2, -10, -1+1i, -1-1i]; 
p2 = [-2, -10, -2+2*1i, -2-2*1i]; 
p3 = [-2, -10, -5+5*1i, -5-5*1i]; 

K1 = place(A, B, p1)
K2 = place(A, B, p2)
K3 = place(A, B, p3) 

A1_tilde = A - B * K1
A2_tilde = A - B * K2 
A3_tilde = A - B * K3 

% Define the output variables
D = 0; 
C = [1 0 0 0]; 

% Calculate kr1
Kr1 = 1/(0 - (C - D * K1)*inv(A-B*K1)*B)
Kr2 = 1/(0 - (C - D * K2)*inv(A-B*K2)*B)
Kr3 = 1/(0 - (C - D * K3)*inv(A-B*K3)*B)

% Compute the systems
r_i = 0; 
r_f = 0.002; 
sys1 = ss(A1_tilde,B*Kr1 * r_f,C,D);
sys2 = ss(A2_tilde,B*Kr2 * r_f,C,D);
sys3 = ss(A3_tilde,B*Kr3 * r_f,C,D);

% Compute the step response
clf
hold on
step(sys1)
step(sys2)
step(sys3)
legend('eig set i.', 'eig set ii.', 'eig set iii.')

% % Eigenvalue check
% K=K0+K2*v0^2;
% AE=[one null;null M];
% E=[null one;-K -C0*v0];
% D=eig(E,AE);
