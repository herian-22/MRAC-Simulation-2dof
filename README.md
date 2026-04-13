# MRAC Simulation for 2-DOF Parabolic Antenna

Simulasi ini memodelkan antena parabola 2-DOF (azimuth–elevation/slope) sebagai **robot manipulator serial** dan mengendalikan geraknya menggunakan kombinasi **Computed Torque + MRAC (MIT Rule)**.

## Preview

![Preview Simulasi MRAC 2-DOF](assets/preview.gif)

## Hasil Simulasi

![Hasil Tracking MRAC 2-DOF](assets/hasil_simulasi.png)

Ringkasan metrik (konfigurasi default):

- Steady-state error joint 1: **0.000002 rad**
- Steady-state error joint 2: **0.000004 rad**
- Overshoot joint 1: **14.30%**
- Overshoot joint 2: **17.08%**
- Settling time joint 1: **3.78 s**
- Settling time joint 2: **3.99 s**
- Max torque joint 1: **176.84 Nm**
- Max torque joint 2: **80.45 Nm**

---

## 1) Analogi Sistem: Antena sebagai Robot Manipulator Serial

### a. Kinematika (Denavit–Hartenberg / DH Notation)
Antena 2-DOF diperlakukan seperti lengan robot serial 2 sendi:
- **Joint 1**: Azimuth (rotasi horizontal)
- **Joint 2**: Slope/Elevation (rotasi vertikal)

Relasi posisi antar-link dalam ruang 3D dibangun dengan transformasi DH, sehingga posisi end-effector (arah pointing dish) dapat dihitung konsisten terhadap perubahan sudut kedua joint.

### b. Dinamika (Euler–Lagrange)
Gerak sistem tidak hanya ditentukan posisi, tapi juga:
- massa dan inersia link,
- kopling non-linear antar-joint,
- efek Coriolis/sentrifugal,
- gravitasi,
- serta friksi viskos.

Model dinamik dibentuk dari energi kinetik dan potensial melalui formulasi Lagrangian, sehingga persamaan gerak mengikuti bentuk umum:

\[
M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) + F_v\dot{q} = \tau
\]

---

## 2) Strategi Kontrol: Linearitas & Adaptasi

### a. Computed Torque Technique
Karena dinamika bersifat non-linear dan saling terkopel, computed torque dipakai untuk mengompensasi bagian non-linear agar dinamika error mendekati sistem linear yang lebih mudah didesain.

### b. Model Reference Adaptive Control (MRAC)
Target kontrol bukan sekadar mencapai setpoint, tetapi membuat plant mengikuti **model referensi ideal orde-2**.

### c. Kompensasi Ketidakpastian
Kontrol adaptif dibutuhkan untuk kondisi parameter tidak pasti/berubah (mis. friksi viskos dan variasi massa), sehingga performa tracking tetap robust terhadap mismatch model.

---

## 3) Mekanisme “Otak” Kontroler (MIT Rule)

### a. Online Estimation
Parameter adaptif \(\alpha\) (di implementasi sebagai parameter \(\theta\)) diperbarui real-time sesuai error tracking.

### b. Update Rule
MIT Rule meminimalkan fungsi biaya:

\[
J(\alpha) = \frac{1}{2}e_o^2
\]

Dengan pembaruan gradien (arah menurunkan error) secara kontinu selama simulasi berjalan.

### c. Adaptive Gain \(\gamma\)
Nilai \(\gamma\) mengatur kecepatan belajar:
- terlalu besar \(\rightarrow\) berisiko osilasi/instabil,
- terlalu kecil \(\rightarrow\) adaptasi lambat.

---

## 4) Evaluasi Performa Transien

Model referensi orde-2 ditentukan dari:
- **\(\zeta\)**: rasio redaman,
- **\(\omega_n\)**: frekuensi alami.

Target desain:
- **Overshoot = 15%**
- **Peak time = 1.8 s**

Jika respons simulasi mendekati target ini, maka model matematis, desain kontrol, dan implementasi numerik dianggap sinkron dengan teori.

---

## 5) Alur Implementasi (Toolchain)

1. **Modeling (CATIA V5)**  
   Ekstraksi parameter fisik \((m, I, a, d)\) dari geometri mekanik.

2. **Mathematics (Maple 13)**  
   Penurunan simbolik persamaan dinamik yang panjang/kompleks untuk meminimalkan kesalahan manual.

3. **Simulation (MATLAB/Simulink)**  
   Verifikasi hukum kontrol sebelum implementasi hardware.

> Repository ini menyediakan implementasi simulasi numerik ekuivalen berbasis Python (NumPy/SciPy) untuk analisis dan eksperimen MRAC 2-DOF.

---

## Menjalankan Proyek

```bash
pip install -r requirements.txt
python test_all.py
python run.py
```

- `test_all.py` menjalankan pengujian model, kontroler, simulasi, dan import GUI.
- `run.py` menjalankan GUI simulasi.
