function plot_map(map)

[rows,cols]=size(map);
[X, Y] = meshgrid(1:cols, 1:rows);
all_coords = [X(:), Y(:)];
is_obstacle = map(:) == 1;




scatter(all_coords(is_obstacle,1), all_coords(is_obstacle,2), 10, 'k', 'filled');
hold on;
xlabel('X');
ylabel('Y');

axis equal;
xlim([1 cols]);
ylim([1 rows]);
grid on;


end