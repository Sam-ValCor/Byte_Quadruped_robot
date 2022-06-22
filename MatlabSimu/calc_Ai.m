function M_Ai=calc_Ai(ai,alphai,di,thetai)
M_Ai=rota_z(thetai)*tras_z(di)*tras_x(ai)*rota_x(alphai);
    