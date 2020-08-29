%% Solve Inverse Kinematics for a Four-Bar Linkage
% This example shows how to solve inverse kinematics for a four-bar linkage, 
% a simple planar closed-chain linkage. Robotics System Toolbox™ doesn't directly 
% support closed-loop mechanisms. However, the loop-closing joints can be approximated 
% using kinematic constraints. This example shows how to setup a rigid body tree 
% for a four-bar linkage, specify the kinematic constraints, and solve for a desired 
% end-effector position.
% 
% Initialize the four-bar linkage rigid body tree model. 

robot = rigidBodyTree('Dataformat','column','MaxNumBodies',7);
%% 
% Define body names, parent names, joint names, joint types, and fixed transforms 
% in cell arrays. The fixed transforms define the geometry of the four-bar linkage. 
% The linkage rotates in the _xz_-plane. An offset of |-0.1| is used in the _y_-axis 
% on the  |'b4'| body to isolate the motion of the overlapping joints for |'b3'| 
% and |'b4'|.

bodyNames = {'b1','b2','b3','b4','b5','b6'};
parentNames = {'base','b1','b2','base','b4','b5'};
jointNames = {'j1','j2','j3','j4','j5','j6'};
jointTypes = {'revolute','revolute','fixed','revolute','revolute','fixed'};
fixedTforms = {eye(4), ...
                trvec2tform([0 0 0.5]), ...
                trvec2tform([0.8 0 0]), ...
                trvec2tform([0.0 -0.1 0]), ...
                trvec2tform([0.8 0 0]), ...
                trvec2tform([0 0 0.5])};
%% 
% Use a |for| loop to assemble the four-bar linkage:
%% 
% * Create a rigid body and specify the joint type.
% * Specify the |JointAxis| property for any non-fixed joints.
% * Specify the fixed transformation.
% * Add the body to the rigid body tree.

for k = 1:6

    b = rigidBody(bodyNames{k});
    b.Joint = rigidBodyJoint(jointNames{k},jointTypes{k});
    
    if ~strcmp(jointTypes{k},'fixed')
        b.Joint.JointAxis = [0 1 0];
    end
    
    b.Joint.setFixedTransform(fixedTforms{k});
    
    addBody(robot,b,parentNames{k});
end
%% 
% Add a final body to function as the end-effector (handle) for the four-bar 
% linkage.

bn = 'handle';
b = rigidBody(bn);
setFixedTransform(b.Joint,trvec2tform([0 -0.15 0]));
addBody(robot,b,'b6');
%% 
% Specify kinematic constraints for the |GeneralizedInverseKinematics| object:
%% 
% * *Position constraint 1* : The origins of |'b3'| body frame and |'b6'| body 
% frame should always overlap. This keeps the handle in line with the approximated 
% closed-loop mechanism. Use the |-0.1| offset for the _y_-coordinate.
% * *Position constraint 2* : End-effector should target the desired position.
% * *Joint limit bounds* : Satisfy the joint limits in the rigid body tree model.

gik = generalizedInverseKinematics('RigidBodyTree',robot);
gik.ConstraintInputs = {'position',...  % Position constraint for closed-loop mechanism
                        'position',...  % Position constraint for end-effector 
                        'joint'};       % Joint limits
gik.SolverParameters.AllowRandomRestart = false;

% Position constraint 1
positionTarget1 = constraintPositionTarget('b6','ReferenceBody','b3');
positionTarget1.TargetPosition = [0 -0.1 0];
positionTarget1.Weights = 50;
positionTarget1.PositionTolerance = 1e-6;

% Joint limit bounds
jointLimBounds = constraintJointBounds(gik.RigidBodyTree);
jointLimBounds.Weights = ones(1,size(gik.RigidBodyTree.homeConfiguration,1))*10;

% Position constraint 2
desiredEEPosition = [0.9 -0.1 0.9]'; % Position is relative to base.
positionTarget2 = constraintPositionTarget('handle');
positionTarget2.TargetPosition = desiredEEPosition; 
positionTarget2.PositionTolerance = 1e-6;
positionTarget2.Weights = 1;
%% 
% Compute the kinematic solution using the |gik| object. Specify the initial 
% guess and the different kinematic constraints in the proper order.

iniGuess = homeConfiguration(robot);
[q, solutionInfo] = gik(iniGuess,positionTarget1,positionTarget2,jointLimBounds);
%% 
% Examine the results in |solutionInfo|. Show the kinematic solution compared 
% to the home configuration. Plots are shown in the _xz_-plane.

loopClosingViolation = solutionInfo.ConstraintViolations(1).Violation;
jointBndViolation = solutionInfo.ConstraintViolations(2).Violation;
eePositionViolation = solutionInfo.ConstraintViolations(3).Violation;

subplot(1,2,1)
show(robot,homeConfiguration(robot));
title('Home Configuration')
view([0 -1 0]);
subplot(1,2,2)
show(robot,q);
title('GIK Solution')
view([0 -1 0]);
%% 
% _Copyright 2019 The MathWorks, Inc._