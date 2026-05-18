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
weight = [0.0207894723856046 0.113390028294325 0.161277715136209 0.191459528425378 0.0820039317878117 0.0387486683408848 0.228102688107747 0.16422796752204];

% Inizializzazione delle variabili simboliche
x_matrix = sym('x', [nNodes, nNodes]);
u = sym('u', [nTask, 1]);
v = sym('v', [nTask, 1]);
R = sym('R', [nTask, 1]);
d = sym('d');
x = [x_matrix(:); u; v; R;d];
numel_x = numel(x);
index_r = numel_x - numel(R) + (1:numel(R))-1;
index_d = numel(x);

% Inizializzazione della matrice A
Aeq_sym = sym([]);
beq = [];

degreeConstraintOut;
degreeConstraintIn;

A_sym = sym([]);
b =[];

subTourUConstraints;
subTourVConstraints;
UAVOutConstraints;
UAVOInConstraints;
lowerBoundUConstraints;
lowerBoundVConstraints;
upperBoundUConstraints;
uppurBoundVConstraints;
numberOfDronesUpperBound;
revisitPeriodConstraints;
diagonalConstrains;

A = double(A_sym);
Aeq = double(Aeq_sym);
lb = zeros(size(x));
lb(end) = 1;
ub = inf.*ones(size(x));
ub(1:nNodes^2) = 1;

funzione_obiettivo = @(var)[weight*(var(index_r))',cost_drone*var(end)];

opts = optimoptions("gamultiobj","FunctionTolerance",1e-12,"PopulationSize",100,"PlotFcn","gaplotpareto");
[x_opt, fval, exitflag, output, population, scores] = gamultiobj(funzione_obiettivo, numel(x),A,b,Aeq,beq,lb,ub,[],[1:nNodes^2 numel(x)],opts);
