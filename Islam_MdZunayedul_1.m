clear all; close all; clc;
warning('off', 'vision:calibrate:boardShouldBeAsymmetric');

addpath(genpath('./lib'));
res_dir = 'results/';
if ~exist(res_dir, 'dir')
    mkdir(res_dir);
end

% =========================================================================
% SECTION 1.1: DIGITAL SCREEN CALIBRATION (MATRIX A)
% =========================================================================
fprintf('======================================================\n');
fprintf('STARTING PART 1: DIGITAL SCREEN CALIBRATION\n');
fprintf('======================================================\n');

img_dir_screen = 'images/calibration_screen/';
img_W = 1008;
img_H = 1792;
diag_half = sqrt(img_W^2 + img_H^2) / 2;

board_width_mm = 182.6;
squares_per_side = 8;
square_size_mm = board_width_mm / squares_per_side;

real_pts_screen = generateCheckerboardPoints([squares_per_side squares_per_side], square_size_mm)';

files_screen = [dir(fullfile(img_dir_screen, '*.jpg')); dir(fullfile(img_dir_screen, '*.JPG'))];
H_screen = cell(1, length(files_screen));
valid_idx = 1;

for i = 1:length(files_screen)
    img_path = fullfile(img_dir_screen, files_screen(i).name);
    I = imread(img_path);
    if size(I,3) == 3, I_gray = rgb2gray(I); else, I_gray = I; end

    [img_pts, ~] = detectCheckerboardPoints(I_gray);
    if isempty(img_pts), continue; end

    H_init = homography_solve_vmmc(real_pts_screen, img_pts');
    [H_ref, ~] = homography_refine_vmmc(real_pts_screen, img_pts', H_init);
    H_screen{valid_idx} = H_ref;
    valid_idx = valid_idx + 1;
end
H_screen(valid_idx:end) = [];

A = internal_parameters_solve_vmmc(H_screen);
save(fullfile(res_dir, 'A_screen.mat'), 'A');

alpha_A = A(1,1); beta_A = A(2,2); gamma_A = A(1,2);
ar_A = alpha_A / beta_A;
ar_error_A = 100 * abs(ar_A - 1);
pp_dist_A = sqrt((A(1,3) - img_W/2)^2 + (A(2,3) - img_H/2)^2);
pp_error_A = 100 * (pp_dist_A / diag_half);
theta_A = atand(-gamma_A / alpha_A);

fprintf('\n--- Matrix A (Screen) ---\n'); disp(A);
fprintf('Aspect Ratio Error: %.2f%%\n', ar_error_A);
fprintf('Principal Point Offset: %.2fpx (%.2f%%)\n', pp_dist_A, pp_error_A);
fprintf('Orthogonality Deviation: %.4f degrees\n\n', theta_A);

% =========================================================================
% SECTION 1.2: CUSTOM PATTERN CALIBRATION (MATRIX A')
% =========================================================================
fprintf('======================================================\n');
fprintf('STARTING PART 2: CUSTOM FLOOR TILE CALIBRATION\n');
fprintf('======================================================\n');

img_dir_custom = 'images/calibration_custom/';
tile_size_mm = 330; 

% Generate 3x3 grid world coordinates (9 points)
real_pts_custom = zeros(2, 9);
idx = 1;
for y = 0 : tile_size_mm : tile_size_mm*2
    for x = 0 : tile_size_mm : tile_size_mm*2
        real_pts_custom(:, idx) = [x; y];
        idx = idx + 1;
    end
end

files_custom = [dir(fullfile(img_dir_custom, '*.jpg')); dir(fullfile(img_dir_custom, '*.JPG'))];
H_custom = cell(1, length(files_custom));
montage_custom_files = cell(1, length(files_custom));

for i = 1:length(files_custom)
    img_path = fullfile(img_dir_custom, files_custom(i).name);
    montage_custom_files{i} = img_path;
    I = imread(img_path);
    
    fprintf('Image %d/%d: Click 8 points with LEFT click, and the 9th with RIGHT click.\n', i, length(files_custom));
    
    % Manual Extraction Tool
    img_pts_custom = get_user_points_vmmc(I);
    
    if size(img_pts_custom, 2) ~= 9
        error('You clicked %d points. You MUST click exactly 9 points. Restart the script.', size(img_pts_custom, 2));
    end
    
    H_init = homography_solve_vmmc(real_pts_custom, img_pts_custom);
    [H_ref, ~] = homography_refine_vmmc(real_pts_custom, img_pts_custom, H_init);
    H_custom{i} = H_ref;
end

% Save custom montage
montage(montage_custom_files);
saveas(gcf, fullfile(res_dir, 'montage_custom.png'));
close all;

% Compute A'
A_prime = internal_parameters_solve_vmmc(H_custom);
save(fullfile(res_dir, 'A_custom.mat'), 'A_prime');

alpha_P = A_prime(1,1); beta_P = A_prime(2,2); gamma_P = A_prime(1,2);
ar_P = alpha_P / beta_P;
ar_error_P = 100 * abs(ar_P - 1);
pp_dist_P = sqrt((A_prime(1,3) - img_W/2)^2 + (A_prime(2,3) - img_H/2)^2);
pp_error_P = 100 * (pp_dist_P / diag_half);
theta_P = atand(-gamma_P / alpha_P);

fprintf('\n--- Matrix A'' (Custom Pattern) ---\n'); disp(A_prime);
fprintf('Aspect Ratio Error: %.2f%%\n', ar_error_P);
fprintf('Principal Point Offset: %.2fpx (%.2f%%)\n', pp_dist_P, pp_error_P);
fprintf('Orthogonality Deviation: %.4f degrees\n\n', theta_P);

% =========================================================================
% SECTION 1.2: COMPARISON (A vs A')
% =========================================================================
fprintf('======================================================\n');
fprintf('CALIBRATION COMPARISON\n');
fprintf('======================================================\n');

diff_matrix = A - A_prime;
frob_norm = norm(diff_matrix, 'fro');

fprintf('Frobenius Norm of Difference (||A - A''||): %.2f\n', frob_norm);
fprintf('Percentage Difference in Focal Length (alpha): %.2f%%\n', 100 * abs(alpha_A - alpha_P) / alpha_A);
fprintf('Percentage Difference in Focal Length (beta): %.2f%%\n', 100 * abs(beta_A - beta_P) / beta_A);
fprintf('\n>>> Calibration Complete. Results saved to /results/ <<<\n');