function target = adjust_to_field(d, target)


target.p(1:2) = max(target.p(1:2),-d.par.field_size*.5-d.par.field_border_margin);
target.p(1:2) = min(target.p(1:2),d.par.field_size*.5+d.par.field_border_margin);

end
