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
        function [theta, solutionInfo] = stepImpl(obj, name_eef, tform_t, theta_0)
            %stepImpl Solve IK
            d2r = pi/180;
            max_delta_norm_theta = 20 * d2r;
            epsilon = 0.01;
            theta_k = theta_0;

            tform_eef = getTransform(obj.bodyTree, theta_k, name_eef);
            p_t(1:3) = tform_t(1:3, 4);
            p_eef(1:3) = tform_eef(1:3, 4);
            e(1:3, 1) = 0;
            e(4:6, 1) = p_t - p_eef;
            %fprintf('stepImpl: [%f %f %f]\n', e(4, 1), e(5, 1), e(6, 1));

            lambd = 1;
            n_it = 0;
            while (norm(e) > epsilon)
                jak = geometricJacobian(obj.bodyTree, theta_k, name_eef);
                %jak_inv = jak'*inv(jak*jak' + lambd * lambd * eye(6,6));
                jak_inv = jak'/(jak*jak' + lambd * lambd * eye(6,6));
                abs_e = abs(e);
                beta_e = max_delta_norm_theta/(max(max_delta_norm_theta, max(abs_e)));
                delta_e = beta_e * e;
                delta_theta = jak_inv * delta_e;
                theta_k = theta_k + delta_theta;
                tform_eef = getTransform(obj.bodyTree, theta_k, name_eef);
                p_t(1:3) = tform_t(1:3, 4);
                p_eef(1:3) = tform_eef(1:3, 4);
                e(1:3, 1) = 0;
                e(4:6, 1) = p_t - p_eef;
                n_it = n_it + 1;
            end
            theta = theta_k;
            solutionInfo.Iterations = n_it;
            solutionInfo.NumRandomRestarts = 0;
            solutionInfo.PoseErrorNorm = norm(e);
            solutionInfo.ExitFlag = 1;
            solutionInfo.Status = 'success';
        end
    end
end