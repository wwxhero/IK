classdef(StrictDefaults) jacobianIK < matlab.System & ...
							robotics.manip.internal.InternalAccess
	properties
		bodyTree
	end
	methods
        function obj = jacobianIK(varargin)
            %inverseKinematics Inverse kinematics Constructor
            %   The first six possible inputs consist of the three
            %   Name-Value pairs specified in the documentation and help
            %   text above (RigidBodyTree, SolverAlgorithm, and
            %   SolverParameters, and their associated values).
            %
            %   The following input is for internal use only, and may not
            %   be supported in a future release:
            %      The constructor can also accept 8 inputs. These include
            %      the six inputs above, as well as an additional
            %      Name-Value pair:
            %         UseTimer -   Indicates whether to use the
            %                      SystemTimeProvider or a mock version
            %                      that does not call platform-specific
            %                      time functions.
            %                      Values: true (default) | false
            assert(isa (varargin{1}, 'rigidBodyTree'));
            obj.bodyTree = copy(varargin{1});
        end
    end
    methods (Access = protected)
        function [QSol, solutionInfo] = stepImpl(obj, endEffectorName, tform, initialGuess)
            %stepImpl Solve IK
            fprintf('stepImpl\n');
            %body = getBody(obj, endEffectorName);
            QSol= zeros(1, obj.bodyTree.NumBodies-1);
            solutionInfo.Iterations = 10;
            solutionInfo.NumRandomRestarts = 0;
            solutionInfo.PoseErrorNorm = 0.00001;
            solutionInfo.ExitFlag = 1;
            solutionInfo.Status = 'success';
        end
    end
end