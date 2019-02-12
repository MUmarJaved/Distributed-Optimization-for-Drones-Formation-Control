function inertia_terms = InertiaTerms(aircraft_parameters)

Jx = aircraft_parameters.Ix;
Jy = aircraft_parameters.Iy;
Jz = aircraft_parameters.Iz;
Jxz = aircraft_parameters.Ixz;

Gamma = Jx*Jz - Jxz^2;

Gam1 = Jxz*(Jx-Jy+Jz)/Gamma;

Gam2 = (Jz*(Jz-Jy)+Jxz^2)/Gamma;

Gam3 = Jz/Gamma;

Gam4 = Jxz/Gamma;

Gam5 = (Jz-Jx)/Jy;

Gam6 = Jxz/Jy;

Gam7 = ((Jx-Jy)*Jx+Jxz^2)/Gamma;

Gam8 = Jx/Gamma;

inertia_terms = [Gam1; Gam2; Gam3; Gam4; Gam5; Gam6; Gam7; Gam8];
