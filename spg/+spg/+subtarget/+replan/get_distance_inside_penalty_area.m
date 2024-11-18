function distance = get_distance_inside_penalty_area(d, pos)

x = -abs(pos(1))+d.par.field_penalty_area(1).*.5;
y = abs(pos(2))-(d.par.field_size(2)*.5-d.par.field_penalty_area(2));
distance = max(0, min([x y]));
