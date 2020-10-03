robot = rigidBodyTree('DataFormat','column','MaxNumBodies',3);

L1 = 0.3;
L2 = 0.3;

body_1 = rigidBody('link1');
joint_1 = rigidBodyJoint('joint1', 'revolute');
m0_1 = trvec2tform([0 0 0]);
setFixedTransform(joint_1, m0_1);
joint_1.JointAxis = [0 0 1];
body_1.Joint = joint_1;
addBody(robot, body_1, 'base');

body_2 = rigidBody('link2');
joint_2 = rigidBodyJoint('joint2','revolute');
m0_2 = trvec2tform([L1, 0, 0]);
setFixedTransform(joint_2, m0_2);
joint_2.JointAxis = [0 0 1];
body_2.Joint = joint_2;
addBody(robot, body_2, 'link1');

%body_2_2 = rigidBody('link2_2');
%joint_2_2 = rigidBodyJoint('joint2_2','revolute');
%setFixedTransform(joint_2_2, trvec2tform([0, 0, 0]));
%joint_2_2.JointAxis = [0 1 0];
%body_2_2.Joint = joint_2_2;
%addBody(robot, body_2_2, 'link2');

body_3 = rigidBody('tool');
joint_3 = rigidBodyJoint('fix1','fixed');
m0_3 = trvec2tform([0, -L2, 0]);
setFixedTransform(joint_3, m0_3);
body_3.Joint = joint_3;
addBody(robot, body_3, 'link2');

showdetails(robot)

t = (0:0.02:10)';                    %Time
n_targets = length(t);
center = [0.3 0.1 0];
radius = 0.15;
theta = t*(2*pi/t(end));
targets = center + radius*[cos(theta) sin(theta) zeros(size(theta))];



q0 = homeConfiguration(robot);      %?
ndof = length(q0);
qs = zeros(n_targets, ndof);


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

weights = [0, 0, 0, 1, 1, 0];       %?
endEffector = 'tool';
endEffector_confs = zeros(4, 4, n_targets);
qInitial = q0; % Use home configuration as the initial guess
elapsedTime_0 = toc(start_0);

start_1 = tic;
for i = 1:n_targets
	% Solve for the configuration satisfying the desired end effector
	% position
	point = targets(i,:);
	endEffector_confs(:, :, i) = trvec2tform(point);
	%assert(isequal(endEffector_confs(1:3, 1:3, i), eye(3)));

	[qSol, solInfo(i)] = ik(endEffector, endEffector_confs(:, :, i), weights, qInitial);
	% Store the configuration
	qs(i,:) = qSol;
	% Start from prior solution
	qInitial = qSol;
end
elapsedTime_1 = toc(start_1);
figure

show(robot,qs(1,:)');				%?
%show(robot,q0);               %?
view(2)								%?
ax = gca;							%?
ax.Projection = 'orthographic';		%?
hold on
plot(targets(:,1),targets(:,2),'k')
axis([-0.1 0.7 -0.3 0.5])

%framesPerSecond = 15;
%r = rateControl(framesPerSecond);
%for i = 1:n_targets
%    show(robot,qs(i,:)','PreservePlot',false);
%    drawnow
%    waitfor(r);
%end

writematrix(qs, 'ik_solutions.csv');
writetable(struct2table(solInfo), 'ik_solutions_info.txt');

jac = zeros(6, ndof, n_targets);
v_r = [0 0 1];
local2parent0(:,:, 1) = m0_1;
local2parent0(:,:, 2) = m0_2;
local2parent0(:,:, 3) = m0_3;
epsilon = 0.0001
for i_target = 1 : n_targets
	theta = qs(i_target, :);
	jac(:, :, i_target) = geometricJacobian(robot, theta', endEffector);
	p = zeros(3, ndof + 1);
	local2world = eye(4,4);
	for i_joint = 1 : ndof
		axangle = [v_r(1), v_r(2), v_r(3), theta(i_joint)];
		delta_m = rotm2tform(axang2rotm(axangle));
		local2world = local2world * local2parent0(:, : , i_joint) * delta_m;
		p(:, i_joint) = local2world(1:3, 4);
	end
	local2world = local2world * local2parent0(:, :, ndof + 1);
	targets_prime = local2world(1:3, 4);
	e_target = norm(targets(i_target, :)' - targets_prime);
	assert(e_target < epsilon);

	p(:, ndof + 1) = local2world(1:3, 4);
	j = jac(:, :, i_target);
	j_prime = zeros(6, ndof);
	for i_joint = 1 : ndof
		j_prime(1:3, i_joint) = v_r;
		j2e = p(:, ndof + 1) - p(:, i_joint);
		j_prime(4:6, i_joint) = cross(v_r, j2e);
	end
	e_jac = j-j_prime;
	max_e_jac = max(max(e_jac, [], 1));
	assert(max_e_jac < epsilon);
end

ik_j = jacobianIK(robot);
qs_j = zeros(n_targets, ndof);
qInitial = q0;

for i_target = 1 : n_targets
	t = targets(i_target, :);
	%[qSol, solInfo(i_target)] = ik_j(endEffector, t, qInitial);
    qSol = qs(i_target, :);
	qs_j(i_target, :) = qSol;
	qInitial = qSol;
end

e_sol = qs - qs_j;
assert(max(max(e_sol, [], 1)) < epsilon);