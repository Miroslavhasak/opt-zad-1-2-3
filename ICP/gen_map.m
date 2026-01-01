function [map] = gen_map()


rows = 100;
cols = 150;

map = zeros(rows, cols);

map(1, :) = 1;          
map(end, :) = 1;        
map(:, 1) = 1;          
map(:, end) = 1;      

map(20, 50:90) = 1;
map(80, 60:120) = 1; 
map(10:60, 125) = 1;

map(50, 20:80) = 1;
map(20:80, 30) = 1;

map(10:15, 85:90) = 1;  
map(11:14, 86:89) = 0; 

map(70:80, 10:20) = 1;     
map(71:79, 11:19) = 0;  
map(45:55, 55:65) = 1; 
map(46:54, 56:64) = 0; 
map(50:60,105:110) = 1;
map(51:59,106:109) = 0;   

map(25:75, 145:150) = 1; 
map(26:74, 146:149) = 0; 
map(1:5, 1:30) = 1;
map(2:4, 2:29) = 0;

margin=5;
map=[zeros(margin,cols+2*margin);
     zeros(rows,margin),map,zeros(rows,margin);
     zeros(margin,cols+2*margin)];


end