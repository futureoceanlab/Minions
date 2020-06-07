function computeESD(fl, N, WD, ax_mj, pxPitch, trace3D, esd)
    dist = norm(trace3D(1, :));
    COC = fl * fl / (N * (WD - fl))* (abs(dist - WD) / dist)/pxPitch/2;
    esd = esd/0.02;
    esd_new = ax_mj - 2*ceil(2*COC);

end