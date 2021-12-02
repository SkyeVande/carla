def update_rinex_nav( filename_in , use_current_sim_time, current_sim_time,pd):
    filename_out = "../../Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/satellite_ephem.csv"

    f = open(filename_in, 'r')

    # init lists
    PRN = []
    Y = []
    M = []
    D = []
    H = []
    min = []
    sec = []
    af0 = []
    af1 = []
    af2 = []
    IODE = []
    Crs = []
    deltaN = []
    M0 = []
    Cuc = []
    Eccentricity = []
    Cus = []
    sqrtA = []
    Toe = []
    Cic = []
    Omega0 = []
    Cis = []
    Io = []
    Crc = []
    omega = []
    OmegaDot = []
    IDOT = []
    L2_codes = []
    GPSWeek = []
    L2_dataflag = []
    SV_acc = []
    SV_health = []
    TGD = []
    IODC = []
    msg_trans_t = []
    fit_int = []
    
    toe = current_sim_time

    # skip through header
    end_of_header = False
    h = 0
    for current_line in f:
        if h < 60:
            print(current_line)
        h = h + 1

        if end_of_header:
            if i == 0:
                PRN.append(float(current_line[0:2]))
                Y.append(float(current_line[2:5]))
                M.append(float(current_line[5:8]))
                D.append(float(current_line[8:11]))
                H.append(float(current_line[11:14]))
                min.append(float(current_line[14:17]))
                sec.append(float(current_line[17:22]))
                af0.append(float(current_line[22:37])*(10**float(current_line[38:41])))
                af1.append(float(current_line[41:56])*(10**float(current_line[57:60])))
                af2.append(float(current_line[60:75])*(10**float(current_line[76:79])))
            elif i == 1:
                IODE.append(float(current_line[3:18])*(10**float(current_line[19:22])))
                Crs.append(float(current_line[22:37])*(10**float(current_line[38:41])))
                deltaN.append(float(current_line[41:56])*(10**float(current_line[57:60])))
                M0.append(float(current_line[60:75])*(10**float(current_line[76:79])))
            elif i == 2:
                Cuc.append(float(current_line[3:18])*(10**float(current_line[19:22])))
                Eccentricity.append(float(current_line[22:37])*(10**float(current_line[38:41])))
                Cus.append(float(current_line[41:56])*(10**float(current_line[57:60])))
                sqrtA.append(float(current_line[60:75])*(10**float(current_line[76:79])))
            elif i == 3:
                if use_current_sim_time:
                    Toe.append(toe)
                else:
                    Toe.append(float(current_line[3:18])*(10**float(current_line[19:22])))
                Cic.append(float(current_line[22:37])*(10**float(current_line[38:41])))
                Omega0.append(float(current_line[41:56])*(10**float(current_line[57:60])))
                Cis.append(float(current_line[60:75])*(10**float(current_line[76:79])))
            elif i == 4:
                Io.append(float(current_line[3:18])*(10**float(current_line[19:22])))
                Crc.append(float(current_line[22:37])*(10**float(current_line[38:41])))
                omega.append(float(current_line[41:56])*(10**float(current_line[57:60])))
                OmegaDot.append(float(current_line[60:75])*(10**float(current_line[76:79])))
            elif i == 5:
                IDOT.append(float(current_line[3:18])*(10**float(current_line[19:22])))
                L2_codes.append(float(current_line[22:37])*(10**float(current_line[38:41])))
                GPSWeek.append(float(current_line[41:56])*(10**float(current_line[57:60])))
                L2_dataflag.append(float(current_line[60:75])*(10**float(current_line[76:79])))
            elif i == 6:
                SV_acc.append(float(current_line[3:18])*(10**float(current_line[19:22])))
                SV_health.append(float(current_line[22:37])*(10**float(current_line[38:41])))
                TGD.append(float(current_line[41:56])*(10**float(current_line[57:60])))
                IODC.append(float(current_line[60:75])*(10**float(current_line[76:79])))
            elif i == 7:
                msg_trans_t.append(float(current_line[3:18])*(10**float(current_line[19:22])))
                if (len(current_line) >= 41):
                    fit_int.append(float(current_line[22:37])*(10**float(current_line[38:41])))
                else:
                    fit_int.append(0)
                i = -1
                j = j+1
            i = i + 1

        if "END OF HEADER" in current_line:
            end_of_header = True
            i = 0
            j = 0

    print(j)

    d = {'PRN': PRN, 'Y': Y, 'M': M, 'D': D, 'H': H, 'min': min, 'sec': sec, 'af0': af0, 'af1': af1, 'af2': af2, 'IODE': IODE, 'Crs': Crs, 'deltaN': deltaN, 'M0': M0, 'Cuc': Cuc, 'Eccentricity': Eccentricity, 'Cus': Cus, 'sqrtA': sqrtA, 'Toe': Toe, 'Cic': Cic, 'Omega0': Omega0, 'Cis': Cis, 'Io': Io, 'Crc': Crc, 'omega': omega, 'OmegaDot': OmegaDot, 'IDOT': IDOT, 'L2_codes': L2_codes, 'GPSWeek': GPSWeek, 'L2_dataflag': L2_dataflag, 'SV_acc': SV_acc, 'SV_health': SV_health, 'TGD': TGD, 'IODC': IODC, 'msg_trans_t': msg_trans_t, 'fit_int': fit_int}
    df = pd.DataFrame(data=d, dtype=float)

    df.to_csv(filename_out)
