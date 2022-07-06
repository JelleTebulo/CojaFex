PROC RotZYX(num psi, num th, num phi)
    R{1,1} := Cos(psi)*Cos(th);
    R{2,1} := Cos(psi)*Sin(th)*Sin(phi)-Sin(psi)*Cos(phi);
    R{3,1} := Cos(psi)*Sin(th)*Cos(phi)+Sin(psi)*Sin(phi);
    R{1,2} := Sin(psi)*Cos(th);
    R{2,2} := Sin(psi)*Sin(th)*Sin(phi)+Cos(psi)*Cos(phi);
    R{3,2} := Sin(psi)*Sin(th)*Cos(phi)-Cos(psi)*Sin(phi);
    R{1,3} := -Sin(th);
    R{2,3} := Cos(th)*Sin(phi);
    R{3,3} := Cos(th)*Cos(phi);
ENDPROC
