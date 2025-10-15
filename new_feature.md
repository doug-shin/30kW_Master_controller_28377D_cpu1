scada ui, 마스터에서 모드 구분
    1. battery charge/discharge mode
    2. battery mode

    1. battery charge/discharge mode
        V_max_cmd, V_min_cmd (0~1000V, V_max_cmd > V_min_cmd)
        I_cmd (-480A~480A)
    
    2. battery mode
        I_max_cmd (0~480A)
        I_min_cmd (-480A~0)
        V_cmd (0~1000V)