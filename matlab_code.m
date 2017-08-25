% The first assignment of RMCP course
% Group members : Palash Halder , Farid Khosravi , Mehdi Mahmoodpour

%Computing the transformation matrix by symbolic Math toolbox
clear all
clc
%
syms alpha theta d a a2 
syms d2 d3 d4 theta1 theta2 theta3 theta4 theta5 theta6
alpha1 = -pi/2;
alpha3 = pi/2;
alpha4 = -pi/2;
alpha5 = pi/2;
T_t = trotx(alpha) * transl([a,0,0]) * transl([0,0,d]) * trotz(theta);

pretty(T_t);

T01 = subs(T_t,[theta  d  a alpha],[theta1 0 0 0])
T12 = subs(T_t,[theta  d  a alpha],[theta2 d2 0 pi/2])
T23 = subs(T_t,[theta  d  a alpha],[theta3 -d3 a2 0])
T34 = subs(T_t,[theta  d  a alpha],[theta4 d4 0 pi/2])
T45 = subs(T_t,[theta  d  a alpha],[theta5 0 0 -pi/2])
T56 = subs(T_t,[theta  d  a alpha],[theta6 0 0 pi/2])

T06 = T01*T12*T23*T34*T45*T56;

simplify (T06)

X=T06(11); % to check the element number 11 of T06 as a sample  

simplify(X)
%%
clear all
clc
%parameters defination

%            theta    d        a    alpha

L(1) = Link([  0      0        0        0      0], 'modified');
L(2) = Link([  0      0.44     0      -pi/2    0], 'modified');
L(3) = Link([  0     -0.12    1.05      0      0], 'modified');
L(4) = Link([  0      1.23     0       pi/2    0], 'modified');
L(5) = Link([  0      0        0      -pi/2    0], 'modified');
L(6) = Link([  0      0        0       pi/2    0], 'modified');

six_link = SerialLink(L,'name','six link');

six_link.fkine([0 0 0 0 0 0])

six_link.tool=transl([0.03,0,0.135])

six_link.plot([0 0 0 0 0 0])

six_link.teach([0 0 0 0 0 0])

%% Plot the robot in different configurations

%Plot the robot in straight horizontal configuration
six_link.plot([0 0 pi/2 0 0 0])
six_link.fkine([0 0 pi/2 0 0 0])
%Plot the robot in straight vertical configuration
six_link.plot([0 -pi/2 pi/2 0 0 0])
six_link.fkine([0 -pi/2 pi/2 0 0 0])
%Plot the robot in working position configuration
six_link.plot([-93 3.6 108 7.2 20 0])
six_link.fkine([-93 3.6 108 7.2 20 0])
%%
%directing the manipulator to the first point[-1,0.3,1.6]

p1=transl([1,-0.3,1.6])

Qp1=six_link.ikunc(p1,[0.01,0.01,0.01,0.01,0.01,0.01])

six_link.fkine(Qp1)

six_link.teach(Qp1)

hold on

trplot(p1,'frame','P','color','b')




%% directing the manipulator to the 2nd point[1.6,-0.7,0.8]

p2=transl([1.6,-0.7,0.8])*troty(pi/2)

Qp2=six_link.ikunc(p2,[0.1,0.1,0.1,0.1,0.1,0.1])

six_link.fkine(Qp2)

six_link.teach(Qp2)

hold on

trplot(p2,'frame','P','color','r')

%%
%homogenous transformation matrix for all joint angles to be set zero :

six_link.fkine([0 0 0 0 0 0])

