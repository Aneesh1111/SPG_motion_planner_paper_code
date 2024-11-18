function R = rot(phi)

cosphi = cos(phi);
sinphi = sin(phi);
R = [cosphi -sinphi;sinphi cosphi];