robot = rigidBodyTree('DataFormat','column','MaxNumBodies',3);

L1 = 0.3;
L2 = 0.3;

body_1 = rigidBody('link1');
joint_1 = rigidBodyJoint('joint1', 'revolute');
setFixedTransform(joint_1,trvec2tform([0 0 0]));
joint_1.JointAxis = [0 0 1];
body_1.Joint = joint_1;
addBody(robot, body_1, 'base');

body_2 = rigidBody('link2');
joint_2 = rigidBodyJoint('joint2','revolute');
setFixedTransform(joint_2, trvec2tform([L1, 0, 0]));
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
setFixedTransform(joint_3, trvec2tform([0, -L2, 0]));
body_3.Joint = joint_3;
addBody(robot, body_3, 'link2');

showdetails(robot)

t = (0:0.02:10)';                    %Time
count = length(t);
center = [0.3 0.1 0];
radius = 0.15;
theta = t*(2*pi/t(end));
points = center + radius*[cos(theta) sin(theta) zeros(size(theta))];



q0 = homeConfiguration(robot);      %?
ndof = length(q0);
qs = zeros(count, ndof);


start_0 = tic;
ik = inverseKinematics('RigidBodyTree', robot, 'SolverAlgorithm', 'LevenbergMarquardt');
weights = [0, 0, 0, 1, 1, 0];       %?
endEffector = 'tool';
endEffector_confs = zeros(4, 4, count);
qInitial = q0; % Use home configuration as the initial guess
elapsedTime_0 = toc(start_0);

start_1 = tic;
for i = 1:count
    % Solve for the configuration satisfying the desired end effector
    % position
    point = points(i,:);
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
plot(points(:,1),points(:,2),'k')
axis([-0.1 0.7 -0.3 0.5])

framesPerSecond = 15;
r = rateControl(framesPerSecond);
for i = 1:count
    show(robot,qs(i,:)','PreservePlot',false);
    drawnow
    waitfor(r);
end

writematrix(qs, 'ik_solutions.csv');
writetable(struct2table(solInfo), 'ik_solutions_info.txt');