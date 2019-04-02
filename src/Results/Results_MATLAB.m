filename = '../../tmp.csv';

results = readtable(filename, 'ReadRowNames', false, 'HeaderLines', 0);

figure;
a = scatter3(results.x_pos, results.y_pos, results.z_pos, 10, results.MotorCurrent, 'filled');