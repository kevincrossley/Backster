function R = transformation( psi, theta, phi, psi0,theta0,phi0 )

    R1 = [cosd(phi)*cosd(theta), cosd(phi)*sind(theta)*sind(psi) - sind(phi)*cosd(psi), sind(phi)*sind(psi) + cosd(phi)*sind(theta)*cosd(psi);
         sind(phi)*cosd(theta), sind(phi)*sind(theta)*sind(psi) + cosd(phi)*cosd(psi), sind(phi)*sind(theta)*cosd(psi) - cosd(phi)*sind(psi);
         -sind(theta),          cosd(theta)*sind(psi),                                 cosd(theta)*cos(psi)                                  ];

    
    R2 = [cosd(phi0)*cosd(theta0), cosd(phi0)*sind(theta0)*sind(psi0) - sind(phi0)*cosd(psi0), sind(phi0)*sind(psi0) + cosd(phi0)*sind(theta0)*cosd(psi0);
         sind(phi0)*cosd(theta0), sind(phi0)*sind(theta0)*sind(psi0) + cosd(phi0)*cosd(psi0), sind(phi0)*sind(theta0)*cosd(psi0) - cosd(phi0)*sind(psi0);
         -sind(theta0),          cosd(theta0)*sind(psi0),                                 cosd(theta0)*cos(psi0)                                  ];

    R = R1*R2;
end

