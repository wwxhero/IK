%% 2-D Path Tracing With Inverse Kinematics
%% Introduction
% This example shows how to calculate inverse kinematics for a simple 2D
% manipulator using the <docid:robotics_ref.bvdhj7x-1 InverseKinematics> class.
% The manipulator robot is a simple 2-degree-of-freedom planar
% manipulator with revolute joints which is created by assembling rigid bodies into
% a <docid:robotics_ref.bvan8uq-1 rigidBodyTree> object. A circular trajectory is
% created in a 2-D plane and given as points to the inverse kinematics solver. The solver
% calculates the required joint positions to achieve this trajectory.
% Finally, the robot is animated to show the robot configurations that
% achieve the circular trajectory.
%% Construct The Robot
% Create a |rigidBodyTree| object and rigid bodies with their
% associated joints. Specify the geometric properties of each rigid body
% and add it to the robot.

%%
% Start with a blank rigid body tree model.
robot = rigidBodyTree('DataFormat','column','MaxNumBodies',3);
%%
% Specify arm lengths for the robot arm.
L1 = 0.3;
L2 = 0.3;

%%
% Add |'entity_x'| body with |'joint_x'| joint.
body = rigidBody('entity_x');
joint = rigidBodyJoint('joint_x', 'prismatic');
setFixedTransform(joint,trvec2tform([0 0 0]));
joint.JointAxis = [1 0 0];
body.Joint = joint;
addBody(robot, body, 'base');

% Add |'entity_y'| body with |'joint_y'| joint.
body = rigidBody('entity_y');
joint = rigidBodyJoint('joint_y', 'prismatic');
setFixedTransform(joint,trvec2tform([0 0 0]));
joint.JointAxis = [0 1 0];
body.Joint = joint;
addBody(robot, body, 'entity_x');

%%
% Add |'link1'| body with |'joint1'| joint.
body = rigidBody('link1');
joint = rigidBodyJoint('joint1', 'revolute');
setFixedTransform(joint,trvec2tform([0 0 0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'entity_y');
%%
% Add |'link2'| body with |'joint2'| joint.
body = rigidBody('link2');
joint = rigidBodyJoint('joint2','revolute');
setFixedTransform(joint, trvec2tform([L1,0,0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'link1');
%%
% Add |'tool'| end effector with |'fix1'| fixed joint.
body = rigidBody('tool');
joint = rigidBodyJoint('fix1','fixed');
setFixedTransform(joint, trvec2tform([L2, 0, 0]));
body.Joint = joint;
addBody(robot, body, 'link2');

%%
% Show details of the robot to validate the input properties. The robot
% should have two non-fixed joints for the rigid bodies and a fixed body
% for the end-effector.
showdetails(robot)

%% Define The Trajectory
% Define a circle to be traced over the course of 10 seconds. This circle
% is in the _xy_ plane with a radius of 0.15.
t = (0:0.2:10)'; % Time
n_targets = length(t);
center = [0.3 0.1 0];
radius = 0.15;
theta = t*(2*pi/t(end));
targets = center + radius*[cos(theta) sin(theta) zeros(size(theta))];

%% Inverse Kinematics Solution
% Use an |inverseKinematics| object to find a solution of robotic
% configurations that achieve the given end-effector positions along the
% trajectory.

%%
% Pre-allocate configuration solutions as a matrix |qs|.
q0 = homeConfiguration(robot);
ndof = length(q0);
qs = zeros(n_targets, ndof);
%%
% Create the inverse kinematics solver. Because the _xy_ Cartesian points are the
% only important factors of the end-effector pose for this workflow,
% specify a non-zero weight for the fourth and fifth elements of the
% |weight| vector. All other elements are set to zero.
start_0 = tic;

params.MaxIterations = 1500;
params.MaxTime = 10;
params.GradientTolerance = 5.0000e-09;
params.SolutionTolerance = 1.0000e-06;
params.EnforceJointLimits = 1;
params.AllowRandomRestart = 1;
params.StepTolerance = 1.0000e-12;
params.ErrorChangeTolerance = 1.0000e-12;
params.DampingBias = 0.0025;
params.UseErrorDamping = 1;
ik = inverseKinematics('RigidBodyTree', robot ...
					, 'SolverAlgorithm', 'LevenbergMarquardt' ...
					, 'SolverParameters', params...
					, 'UseTimer', false );


weights = [1, 1, 1, 1, 1, 0];
endEffector = 'tool';
endEffector_confs = zeros(4, 4, n_targets);
%%
% Loop through the trajectory of points to trace the circle. Call the |ik|
% object for each point to generate the joint configuration that achieves
% the end-effector position. Store the configurations to use later.

qInitial = q0; % Use home configuration as the initial guess
elapsedTime_0 = toc(start_0);
start_1 = tic;
for i = 1:n_targets
	% Solve for the configuration satisfying the desired end effector
	% position
	point = targets(i,:);
	z_prime = [0 0 1];
	x_prime = point - center;
	x_prime = x_prime./norm(x_prime);
	y_prime = cross(z_prime, x_prime);
	ori_t = affine3d([x_prime(1), x_prime(2), x_prime(3), 0;...
					, y_prime(1), y_prime(2), y_prime(3), 0;...
					, z_prime(1), z_prime(2), z_prime(3), 0;...
					, point(1), point(2), point(3), 1]);

	endEffector_confs(:, :, i) = ori_t.T';
	[qSol, solInfo(i)] = ik(endEffector, endEffector_confs(:, :, i), weights, qInitial);
	% Store the configuration
	qs(i,:) = qSol;
	% Start from prior solution
	qInitial = qSol;
end
elapsedTime_1 = toc(start_1);

%animateFIK(robot, qs, targets, solInfo);
writematrix(qs, 'ik_solutions_rootmov.csv');
writetable(struct2table(solInfo), 'ik_solutions_info_rootmov.txt');

%%
ik_j = jacobianIK(robot);
qs_j = zeros(n_targets, ndof);
qInitial = q0;
epsilon_d = 0.01; %1 centimeter
epsilon_rd = 3.4437; % in degree
epsilon_r = (pi/180) * epsilon_rd;
for i_target = 1 : n_targets
	point = targets(i_target, :);
	z_prime = [0 0 1];
	x_prime = point - center;
	x_prime = x_prime./norm(x_prime);
	y_prime = cross(z_prime, x_prime);
	ori_t = affine3d([x_prime(1), x_prime(2), x_prime(3), 0;...
					, y_prime(1), y_prime(2), y_prime(3), 0;...
					, z_prime(1), z_prime(2), z_prime(3), 0;...
					, point(1), point(2), point(3), 1]);

	endEffector_confs(:, :, i) = ori_t.T';
	[qSol, solInfo_prime(i_target), ~, ~] = ik_j(endEffector, endEffector_confs(:, :, i), qInitial, epsilon_d, epsilon_r, 500, 1, weights);
	qs_j(i_target, :) = qSol;
	qInitial = qSol;
end

e_sol = qs - qs_j;
%assert(max(max(e_sol, [], 1)) < epsilon);
fprintf('error:%f', max(max(e_sol, [], 1)));

animateFIK(robot, qs_j, targets, solInfo_prime);
epsilon_r_d = 1;
epsilon_r_n = round(epsilon_r_d * epsilon_rd);
epsilon_d_d = 100;
epsilon_d_n = round(epsilon_d_d * epsilon_d);
sol_file_name = sprintf('ik_solutions_j_epsilon=%d_%dx%d_%d', epsilon_r_n,  epsilon_r_d, epsilon_d_n, epsilon_d_d);
solInfo_file_name = sprintf('ik_solutions_info_j_epsilon=%d_%dx%d_%d', epsilon_r_n,  epsilon_r_d, epsilon_d_n, epsilon_d_d);

writematrix(qs_j, sol_file_name);
writetable(struct2table(solInfo_prime), solInfo_file_name);
[~, n_theta] = size(qs_j);
case_inspect.theta_0 = [0	0	0	0; ...
						-0.148092149	0.08440374	0.05415156	0.028506282; ...
						];
case_inspect.targets = [0.45	0.1	0; ...
						0.445287474	0.137303483	0; ...
						];
[n_case, ~] = size(case_inspect.theta_0);
for i_case = 1 : n_case
	point = case_inspect.targets(i_case, :);
	z_prime = [0 0 1];
	x_prime = point - center;
	x_prime = x_prime./norm(x_prime);
	y_prime = cross(z_prime, x_prime);
	ori_t = affine3d([x_prime(1), x_prime(2), x_prime(3), 0;...
					, y_prime(1), y_prime(2), y_prime(3), 0;...
					, z_prime(1), z_prime(2), z_prime(3), 0;...
					, point(1), point(2), point(3), 1]);
	conf = ori_t.T';
	theta_0 = case_inspect.theta_0(i_case, :)';
	lambda_sqr = [1, 0.1];
	[~, sInfo, theta, err_k_r, err_k_p] = ik_j(endEffector, conf, theta_0, epsilon_d, epsilon_r, 1000, lambda_sqr(1), weights);
	[~, sInfo2, theta2, err_k2_r, err_k2_p] = ik_j(endEffector, conf, theta_0, epsilon_d, epsilon_r, 1000, lambda_sqr(2), weights);
	n_theta = sInfo.Iterations+1; % theta(0) = theta_0
	n_theta_prime = sInfo2.Iterations+1; %theta_prime(0) = theta_0
    Theta = theta(1:n_theta, :);
    Theta_prime = theta2(1:n_theta_prime, :);
	Err_p = err_k_p(1:n_theta);
	Err_p_prime = err_k2_p(1:n_theta_prime);
	Err_r = err_k_r(1:n_theta);
	Err_r_prime = err_k2_r(1:n_theta_prime);
	plotIKOriAlgorPerf(Theta, Err_r, Err_p, lambda_sqr(1) ...
			     , Theta_prime, Err_r_prime, Err_p_prime, lambda_sqr(2));
end
