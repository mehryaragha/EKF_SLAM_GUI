% Written by: Mehryar Emambakhsh
% Email: mehryar_emam@yahoo.com
% Date: 23 March 2019
% GUI Implementation of the EKF SLAM explain in the Probabilistic Robotics
% book by Sebastian Thrun, Wolfram Burgard and Dieter Fox.

function [mu_t, sigma_t] = EKF_SLAM_Known_Correspondences(mu_t_minus_1, sigma_t_minus_1, u_t, z_t, c_t)
global Num_of_landmarks;
global delta_t
F_x = [eye(3), zeros(3, 3* Num_of_landmarks)];

mu_t_bar = mu_t_minus_1 +...
    F_x'* [cos(mu_t_minus_1(3) + delta_t* u_t(2)) * u_t(1) * delta_t;...
    sin(mu_t_minus_1(3) + delta_t* u_t(2)) * u_t(1) * delta_t;...
    u_t(2)* delta_t];

% Make sure mu_t_theta is always wrapped btw 0 and 2*pi
mu_t_bar(3)
mu_t_bar(3) = wrapTo2Pi(mu_t_bar(3));

G_t = eye(3 + 3* Num_of_landmarks) + F_x'* [0 0 -sin(mu_t_minus_1(3) + delta_t* u_t(2)) * u_t(1) * delta_t;...
    0 0 cos(mu_t_minus_1(3) + delta_t* u_t(2)) * u_t(1) * delta_t;...
    0 0 0]* F_x;

% R_t = rand(3); R_t = R_t'* R_t;
R_t = diag(rand(1, 3)/40);
sigma_t_bar = G_t* sigma_t_minus_1* G_t' + F_x'* R_t* F_x;

sig_r = 0.01;
sig_phi = 0.01* pi/180;
sigma_s = 0.25;
Q_t = [sig_r^2 0 0; 0 sig_phi^2 0; 0 0 sigma_s^2];

for observed_landmark_cnt = 1: size(z_t, 1)
    curr_landmark_ind = z_t(observed_landmark_cnt, 3);
    if isempty(c_t) || sum(curr_landmark_ind == c_t) == 0
        % The landmark has never seen before
        r_i = z_t(observed_landmark_cnt, 1);
        phi_i = z_t(observed_landmark_cnt, 2);
        s_i = z_t(observed_landmark_cnt, 3);
        
%         mu_j_x_bar = mu_t_bar(1) + r_i* cos(mu_t_bar(3) + phi_i);
%         mu_j_y_bar = mu_t_bar(2) + r_i* sin(mu_t_bar(3) + phi_i);
        mu_j_x_bar = mu_t_bar(1) + r_i* cos(phi_i);
        mu_j_y_bar = mu_t_bar(2) + r_i* sin(phi_i);
        mu_j_s_bar = s_i;
        
        
        mu_t_bar(1 + 3*curr_landmark_ind) = mu_j_x_bar;
        mu_t_bar(1 + 3*curr_landmark_ind + 1) = mu_j_y_bar;
        mu_t_bar(1 + 3*curr_landmark_ind + 2) = mu_j_s_bar;
    else
        % if the landmark was seen before, use the state vector as its
        % location
        mu_j_x_bar = mu_t_bar(1 + 3*curr_landmark_ind);
        mu_j_y_bar = mu_t_bar(1 + 3*curr_landmark_ind + 1);
        mu_j_s_bar = curr_landmark_ind;
    end
    
    mydelta_x = mu_j_x_bar - mu_t_bar(1);
    mydelta_y = mu_j_y_bar - mu_t_bar(2);
    mydelta = [mydelta_x; mydelta_y];
    
    q = mydelta' * mydelta;
    
    
    if atan2(mydelta_y, mydelta_x) < 0
        z_t_i_hat = [sqrt(q); (2*pi + atan2(mydelta_y, mydelta_x)); mu_j_s_bar];
    else
        z_t_i_hat = [sqrt(q); atan2(mydelta_y, mydelta_x); mu_j_s_bar];
    end
%     if atan2(mydelta_y, mydelta_x) < 0
%         z_t_i_hat = [sqrt(q); (2*pi + atan2(mydelta_y, mydelta_x)) - mu_t_bar(3); mu_j_s_bar];
%     else
%         z_t_i_hat = [sqrt(q); atan2(mydelta_y, mydelta_x) - mu_t_bar(3); mu_j_s_bar];
%     end
    
    a = [eye(3); zeros(3)];
    b = zeros(6, 3* curr_landmark_ind - 3);
    c = [zeros(3); eye(3)];
    d = zeros(6, 3* Num_of_landmarks - 3* curr_landmark_ind);
    F_x_j = [a, b, c, d];
    
    % Remember here my definition of z_t_i_hat is (-1) times the definition
    % in the book. therefore, the second row of my H_t_i would be different
    % from the book (would be -1 times the book's).
%     H_t_i = (1/q) * [-sqrt(q)* mydelta_x, -sqrt(q)* mydelta_y, 0, sqrt(q)* mydelta_x, sqrt(q)* mydelta_y, 0;...
%         mydelta_y, -mydelta_x, -q, -mydelta_y, mydelta_x, 0;...
%         0, 0, 0, 0, 0, q]...
%         * F_x_j;
    
    H_t_i = (1/q) * [-sqrt(q)* mydelta_x, -sqrt(q)* mydelta_y, 0, sqrt(q)* mydelta_x, sqrt(q)* mydelta_y, 0;...
        mydelta_y, -mydelta_x, 0, -mydelta_y, mydelta_x, 0;...
        0, 0, 0, 0, 0, q]...
        * F_x_j;
    
    
    K_t_i = sigma_t_bar* H_t_i'* inv(H_t_i* sigma_t_bar* H_t_i' + Q_t);
    
    (z_t(observed_landmark_cnt, :)' - z_t_i_hat)
    mu_t_bar = mu_t_bar + K_t_i* (z_t(observed_landmark_cnt, :)' - z_t_i_hat);
    
    sigma_t_bar = sigma_t_bar - K_t_i* H_t_i* sigma_t_bar;
end

mu_t = mu_t_bar;
sigma_t = sigma_t_bar;

end