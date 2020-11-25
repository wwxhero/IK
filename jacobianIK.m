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
        function [theta, solutionInfo, Theta, Err] = stepImpl(obj, name_eef, tform_t, theta_0, epsilon_p, epsilon_r, N, lambd_sqr, weights)
            %stepImpl Solve IK
            d2r = pi/180;
            max_delta_norm_theta = 20 * d2r;
            theta_k = theta_0;
            %weights = [0 0 0 1 1 1];
            w = diag(weights);


            [n_theta, ~] = size(theta_0);
            Theta = zeros(N, n_theta);
            Err = zeros(N, n_theta);

            tform_eef = getTransform(obj.bodyTree, theta_k, name_eef);
            r_t = tform_t(1:3, 1:3);
            p_t(1:3) = tform_t(1:3, 4);
            r_eef = tform_eef(1:3, 1:3);
            p_eef(1:3) = tform_eef(1:3, 4);
            e_r_ua = rotm2axang(r_t * r_eef');
            e(1:3, 1) = e_r_ua(4) * e_r_ua(1:3);
            e(4:6, 1) = p_t - p_eef;

            norm_e_r = norm(e(1:3));
            norm_e_p = norm(e(4:6));
            %fprintf('stepImpl: [%f %f %f]\n', e(4, 1), e(5, 1), e(6, 1));

            %lambd_sqr ^ 6 == 0.0001 (epsilon for determinant)
            n_it = 0;
            sigmove = true;
            while ((norm_e_p > epsilon_p || norm_e_r > epsilon_r) ...
                    & sigmove ...
                    & n_it < N)
                jak = geometricJacobian(obj.bodyTree, theta_k, name_eef);
                %jak_inv = jak'*inv(jak*jak' + lambd * lambd * eye(6,6));
                jak_inv = jak'/(jak*jak' + lambd_sqr * eye(6,6));
                delta_theta = jak_inv * w * e;
                beta_theta = max_delta_norm_theta/(max(max_delta_norm_theta, max(abs(delta_theta))));
                delta_theta = delta_theta * beta_theta;
                theta_k = theta_k + delta_theta;
                sigmove = (beta_theta > 0.1);
                tform_eef = getTransform(obj.bodyTree, theta_k, name_eef);
                p_t(1:3) = tform_t(1:3, 4);
	            r_eef = tform_eef(1:3, 1:3);
                p_eef(1:3) = tform_eef(1:3, 4);
	            e_r_ua = rotm2axang(r_t * r_eef');
                e(1:3, 1) = e_r_ua(4) * e_r_ua(1:3);
                e(4:6, 1) = p_t - p_eef;
                norm_e_r = norm(e(1:3));
                norm_e_p = norm(e(4:6));
                n_it = n_it + 1;
                Theta(n_it, :) = theta_k;
                Err(n_it, :) = norm_e_p;
            end
            theta = theta_k;
            solutionInfo.Iterations = n_it;
            solutionInfo.NumRandomRestarts = 0;
            solutionInfo.PoseErrorNorm = norm_e_p;
            solutionInfo.ExitFlag = 1;
            solutionInfo.Status = 'success';
            solutionInfo.theta = theta_0;
            solutionInfo.target = p_t;
            solutionInfo.theta_prime = theta;
        end
    end
end