def gen_mls(iniM, iniN):
    n = len(iniN)
    M = iniM + [0] * (2**n - 1 - len(iniM))
    for i in range(n, len(M)):
        M[i] = cal_M(i, n-1, M, iniN, M[i-n]*iniN[-n])
    return M
def cal_M(i, n, M, h, tmp):
    if n == 1:
        return (tmp + M[i-n]*h[-n])%2
    else:
        return cal_M(i, n-1, M, h, (tmp + M[i-n]*h[-n])%2)

print(gen_mls([0,0,0,1,0,0,1,0,1,1,0], [1,0,0,1,0,0,1,1,0,0,1]))