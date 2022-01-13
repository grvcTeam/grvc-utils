clear; close all; clc
format long g

% Estimate pylons' height
pylons_height = 15;

pylons_position_cartesian = [80.7665 1.30242 1.32937;
-92.9081 -67.2953 -5.42197;
-198.16 -109.34 -6.00928;
-357.534 -172.716 -4.37349;
65.7817 2.49236 0.944771;
31.9492 76.0934 -0.21331;
0.347093 145.776 -1.23331;
-27.4924 206.404 -0.277641;
-65.9503 289.206 2.59584;
141.512 3.34838 2.76901;
221.271 6.44622 5.39417;
299.497 9.54586 7.12665;
378.85 9.2912 8.26408;];

pylons_connections_indexes = [ { [2 5 10] }
{ [1 3] }
{ [2 4] }
{ [3] }
{ [1 6] }
{ [5 7] }
{ [6 8] }
{ [7 9] }
{ [8] }
{ [1 11] }
{ [10 12] }
{ [11 13] }
{ [12] }
];

# Wires: calculate and print the position, rotation and length of the wires to copy it into the world file:
# Wires' Z pose is calculated by the mean of the two pylons' top that connects and we also substract one extra meter for visualization purpose
wires=[0 0 0 0 0 0 0];
wires_count = 1;

[N_pylons,~] = size(pylons_position_cartesian);
for it_1=1:N_pylons
  [~,N_connections_current_pylon] = size(pylons_connections_indexes{it_1});
  for it_2=1:N_connections_current_pylon
    if it_1 > pylons_connections_indexes{it_1}(1,it_2)  # Don't repeat twice the connection
      wires(wires_count,:) = [(pylons_position_cartesian(it_1,1)+pylons_position_cartesian(pylons_connections_indexes{it_1}(1,it_2),1))/2 (pylons_position_cartesian(it_1,2)+pylons_position_cartesian(pylons_connections_indexes{it_1}(1,it_2),2))/2 ((pylons_position_cartesian(it_1,3)+pylons_height-1)+(pylons_position_cartesian(pylons_connections_indexes{it_1}(1,it_2),3)+pylons_height-1))/2 pi/2-atan2((pylons_position_cartesian(it_1,3)-pylons_position_cartesian(pylons_connections_indexes{it_1}(1,it_2),3)),(sqrt((pylons_position_cartesian(it_1,1)-pylons_position_cartesian(pylons_connections_indexes{it_1}(1,it_2),1))^2+(pylons_position_cartesian(it_1,2)-pylons_position_cartesian(pylons_connections_indexes{it_1}(1,it_2),2))^2))) 0 pi/2+atan2((pylons_position_cartesian(it_1,2)-pylons_position_cartesian(pylons_connections_indexes{it_1}(1,it_2),2)),(pylons_position_cartesian(it_1,1)-pylons_position_cartesian(pylons_connections_indexes{it_1}(1,it_2),1))) sqrt((pylons_position_cartesian(it_1,1)-pylons_position_cartesian(pylons_connections_indexes{it_1}(1,it_2),1))^2+(pylons_position_cartesian(it_1,2)-pylons_position_cartesian(pylons_connections_indexes{it_1}(1,it_2),2))^2+(pylons_position_cartesian(it_1,3)-pylons_position_cartesian(pylons_connections_indexes{it_1}(1,it_2),3))^2)];
      wires_count = wires_count+1;
    end
  end
end

wires


# Pylons: calculate and print the pylons position, rotation and length to copy it into the world file:
% Pylons' frame is on its center of mass, that's why Z pose depends on pylons_heigth/2
pylons=[0 0 0 0 0 0 0];
pylons_count = 1;

[N_pylons,~] = size(pylons_position_cartesian);
for it_1=1:N_pylons
  pylons(pylons_count,:)=[pylons_position_cartesian(it_1,1) pylons_position_cartesian(it_1,2) (pylons_position_cartesian(it_1,3)+pylons_height)/2 0 0 0 pylons_position_cartesian(it_1,3)+pylons_height];
  pylons_count = pylons_count+1;
end

pylons
