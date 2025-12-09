import requests
import numpy as np
import matplotlib.pyplot as plt

# BASE = 'http://192.168.43.1:8082/download?path=' # 'http://www.example.com/'
# PATH = '' # TODO: Set PATH

# resp = requests.get(BASE+PATH, timeout=3)
# assert resp.status_code == 200

DUMMY = """0,0
0.1,500
0.2,1000
0.3,1400"""

datapoints = []
for line in DUMMY.splitlines(): # resp.text.splitlines()
    datapoints.append([float(i) for i in line.split(',')])

datapoints = np.array(datapoints)
x = datapoints[:, 0]
y = datapoints[:, 1]

# --- Regressions ---
coef_lin = np.polyfit(x, y, 1)
coef_quad = np.polyfit(x, y, 2)
coef_cub = np.polyfit(x, y, 3)

print(coef_lin)
print(coef_quad)
print(coef_cub)

p_lin = np.poly1d(coef_lin)
p_quad = np.poly1d(coef_quad)
p_cub = np.poly1d(coef_cub)

x_fit = np.linspace(min(x), max(x), 400)

# --- Plot ---
plt.figure()
plt.scatter(x, y, label="Data")
plt.plot(x_fit, p_lin(x_fit), label="Linear")
plt.plot(x_fit, p_quad(x_fit), label="Quadratic")
plt.plot(x_fit, p_cub(x_fit), label="Cubic")

plt.xlabel("Power")
plt.ylabel("RPM")
plt.title("Power vs. RPM Curve Approximatino")
plt.legend()
plt.grid(True)
plt.show()