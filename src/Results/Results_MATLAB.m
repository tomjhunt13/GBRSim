filename = '../../tmp.csv';

results = readtable(filename, 'ReadRowNames', true, 'HeaderLines', 0);

figure;scatter3(results.x_pos, results.y_pos, results.z_pos, 10, results.MotorCurrent, 'filled');