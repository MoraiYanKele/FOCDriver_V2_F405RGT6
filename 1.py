from scipy import signal

fs = 1000      # 采样率
fc = 10        # 截止频率
order = 2      # 二阶

# 归一化截止频率
Wn = fc / (fs / 2)

# 设计二阶Butterworth低通
b, a = signal.butter(order, Wn, btype='low', analog=False)

print('b:', b)
print('a:', a)