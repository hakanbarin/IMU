import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ——— Verileri Yükle ———
data1 = np.loadtxt('out.txt', delimiter=',')      # shape (N,3)
data2 = np.loadtxt('mag_out.txt', delimiter=',')

# ——— Ham Haliyle Overlay ———
bias1 = data1.mean(axis=0)
bias2 = data2.mean(axis=0)

# ——— Basit Hard-Iron Kalibrasyonu: ortalamayı sıfıra çevir ———
data1_cal = data1 - bias1
data2_cal = data2 - bias2

# ——— Çizim ———
fig = plt.figure(figsize=(12, 6))

# Ham veriler üst üste
ax1 = fig.add_subplot(1, 2, 1, projection='3d')
ax1.scatter(data1[:,0], data1[:,1], data1[:,2],
            c='C0', marker='o', s=30, alpha=0.8, label='Manyo 1')
ax1.scatter(data2[:,0], data2[:,1], data2[:,2],
            c='C1', marker='^', s=30, alpha=0.8, label='Manyo 2')
ax1.set_title('Overlay: Ham Veriler')
ax1.set_xlabel('X'); ax1.set_ylabel('Y'); ax1.set_zlabel('Z')
ax1.legend()

# Kalibre edilmiş hâliyle üst üste
ax2 = fig.add_subplot(1, 2, 2, projection='3d')
ax2.scatter(data1_cal[:,0], data1_cal[:,1], data1_cal[:,2],
            c='C0', marker='o', s=30, alpha=0.8, label='1 (cal)')
ax2.scatter(data2_cal[:,0], data2_cal[:,1], data2_cal[:,2],
            c='C1', marker='^', s=30, alpha=0.8, label='2 (cal)')
ax2.set_title('Overlay: Basit Kalibrasyon (bias çıkarma)')
ax2.set_xlabel('X'); ax2.set_ylabel('Y'); ax2.set_zlabel('Z')
ax2.legend()

plt.tight_layout()
plt.show()
