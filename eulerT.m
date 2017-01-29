function R = eulerT(psi,theta,phi)


Rx = [1,    0,      0;
      0,cosd(psi),-sind(psi);
      0,sind(psi),cosd(psi)];
  
Ry = [cosd(theta) 0	sind(theta)
            0	 1	0
	-sind(theta)	 0	cosd(theta);];

Rz = [cosd(phi)      -sind(phi)   0;
        sind(phi)	cosd(phi)	0;
        0               0       1;];


R = Rz*Ry*Rx;
end