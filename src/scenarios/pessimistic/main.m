clc
clear
close all
addpath("vincoli\");
% Inizializzazione parametri
nTask = 8;
nNodes = nTask + 1;
nv = 7;
M = 1000;
c = [0 32 17 9 92 52 88 53 28;32 0 49 27 55 88 73 95 98;17 49 0 19 95 21 42 88 46;9 27 19 0 75 46 97 52 93;92 55 95 75 0 52 61 75 29;52 88 21 46 52 0 1 56 85;88 73 42 97 61 1 0 74 60;53 95 88 52 75 56 74 0 87;28 98 46 93 29 85 60 87 0];
cost_drone = 1000;
weight = [0.37 0.54 0.5 0.22 0.95 0.31 0.46 0.75];
weight_two = weight;
weight_three = weight;
weight_two([1 4 7]) = weight_two([1 4 7]) + 0.2;
weight_three([1 4 7]) = weight_two([1 4 7]) + 0.2;

% Inizializzazione delle variabili simboliche
x_matrix = sym('x', [nNodes, nNodes]);
u = sym('u', [nTask, 1]);
v = sym('v', [nTask, 1]); 
R = sym('R', [nTask, 1]);
d = sym('d');
y = sym('y');
x = [x_matrix(:); u; v; R;d;y];
numel_x = numel(x);
index_r = numel_x - numel(R) + (1:numel(R))-2-numel(d);
index_d = numel_x - 1;

% Inizializzazione della matrice A
Aeq_sym = sym([]);
beq = [];

degreeConstraintOut;
degreeConstraintIn;

A_sym = sym([]);
b =[];

numberOfDronesUpperBound;
subTourUConstraints;
subTourVConstraints;
UAVOutConstraints;
UAVOInConstraints;
lowerBoundUConstraints;
lowerBoundVConstraints;
upperBoundUConstraints;
uppurBoundVConstraints;
revisitPeriodConstraints;
diagonalConstrains;
uncertainConstraints;

A = double(A_sym);
Aeq = double(Aeq_sym);
%%
cost = zeros(size(x));
cost(index_d) = 0.05*cost_drone;
cost(index_r) = 0.95*weight_three;

lb = zeros(size(x));
lb(end-1) = 1;
ub = inf.*ones(size(x));
ub(1:nNodes^2) = 1;
[x_opt,fval] = intlinprog(cost,[1:nNodes^2 index_d],A,b,Aeq,beq,lb,ub);

x_matrix_opt = reshape(x_opt(1:(nNodes * nNodes)), nNodes, nNodes);

u_opt = x_opt(nNodes * nNodes + 1 : nNodes * nNodes + nTask);
v_opt = x_opt(nNodes * nNodes + nTask + 1 : nNodes * nNodes + 2 * nTask);
R_opt = x_opt(nNodes * nNodes + 2 * nTask + 1 : nNodes * nNodes + 3 * nTask);
d_opt = x_opt(index_d);
y_opt = x_opt(end);

disp('Matrice ottimale x_matrix:');
disp(x_matrix_opt);
disp('Vettore ottimale u:');
disp(u_opt);
disp('Vettore ottimale v:');
disp(v_opt);
disp('Vettore ottimale R:');
disp(R_opt);
disp('Valore ottimale d:')
disp(d_opt);
disp('Valore ottimale y:');
disp(y_opt);