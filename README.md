# MRAC Simulation for 2-DOF Parabolic Antenna

Simulasi ini memodelkan antena parabola 2-DOF (azimuth–elevation) sebagai sistem robotik 2-joint dan mengendalikan geraknya dengan kombinasi **Computed Torque + MRAC (MIT Rule)**.

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

## 1) Sistem yang Dimodelkan

### antena sebagai manipulator serial 2-DOF
- **Joint 1 (q1)**: azimuth (rotasi horizontal)
- **Joint 2 (q2)**: elevation/slope (rotasi vertikal)

### Kinematika (DH)
Transformasi Denavit–Hartenberg digunakan untuk menghitung pose antar-link hingga titik ujung (arah pointing antena) dalam ruang 3D.

### Dinamika (Euler–Lagrange)
Persamaan gerak plant mempertimbangkan:
- matriks inersia \(M(q)\),
- efek Coriolis/sentrifugal \(C(q,\dot{q})\dot{q}\),
- gravitasi \(G(q)\),
- torsi disipatif/friksi viskos.

Bentuk umum model:
\[
M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) + F_v\dot{q} = \tau
\]

---

## 2) Fitur Utama Implementasi

### engine simulasi
- Integrasi ODE penuh dengan `scipy.integrate.solve_ivp` (default `RK45`).
- Simulasi tunggal dan batch (`run_simulation`, `run_batch`).
- Pembatasan torsi aktuator (torque clipping) untuk menjaga realistis numerik.

### Strategi kontrol
- **Computed Torque** untuk kompensasi nonlinier plant.
- **MRAC berbasis MIT Rule** untuk update parameter adaptif online.
- Tracking terhadap **model referensi orde-2** (bukan hanya setpoint statis).

### Jenis lintasan input
- `step`
- `sinusoidal`
- `multipoint` (interpolasi spline)

### Analitik dan metrik
- SSE, peak error, overshoot, settling time 2%, RMS, ISE, max torque.
- Logging sinyal penting: \(q\), \(\dot q\), \(q_m\), error, \(\tau\), parameter adaptif, end-effector, torsi friksi.

### GUI interaktif (PySide6)
- Blok diagram ala Simulink.
- Panel parameter untuk plant, referensi, kontroler, dan simulasi.
- 6 tab output:
  1. Basic System Output
  2. MRAC Performance Analysis
  3. Adaptive Parameters
  4. Optimization: Gain Variation
  5. Metrics
  6. 3D Dynamics
- Visualisasi 3D gerak antena.
- Export hasil ke PNG, CSV, dan TXT metrik.

---

## 3) Kelebihan Sistem

- **Model fisik lengkap**: kinematika + dinamika nonlinier + friksi.
- **Tracking berbasis model referensi**: performa transien lebih terarah (sesuai spesifikasi desain).
- **Tahan mismatch model**: adaptasi online membantu saat parameter plant tidak ideal.
- **Eksperimen cepat**: parameter bisa diubah langsung dari GUI.
- **Analisis komprehensif**: kurva, metrik numerik, sweep gain, dan visualisasi 3D tersedia dalam satu alur.

---

## 4) Kenapa Sistem Ini Adaptif?

Sistem ini adaptif karena parameter kontrol **tidak tetap**, tetapi diperbarui selama simulasi berdasarkan error tracking:

- Error utama: \(e = q - q_m\)
- Parameter adaptif \(\alpha\) diperbarui kontinu via MIT Rule.
- Laju adaptasi diatur gain \(\gamma\):
  - \(\gamma\) terlalu besar → respons cepat tapi berpotensi osilatif.
  - \(\gamma\) terlalu kecil → stabil tapi adaptasi lambat.

Dengan mekanisme ini, kontroler bisa menyesuaikan aksi kontrol saat terjadi ketidakpastian (misalnya perubahan friksi/parameter efektif plant), sehingga tracking ke model referensi tetap terjaga.

---

## 5) Bagaimana Prosesnya (Alur Kerja End-to-End)

1. **Tentukan konfigurasi simulasi**  
   Input trajectory, parameter fisik plant, gain PD, gain adaptif \(\gamma\), dan parameter model referensi (\(\zeta, \omega_n\)).

2. **Bangun sinyal referensi**  
   Generator trajectory membuat \(q_d, \dot q_d, \ddot q_d\) sesuai mode input (step/sinusoidal/multipoint).

3. **Jalankan model referensi**  
   Dinamika referensi orde-2 menghasilkan lintasan target internal \(q_m, \dot q_m\).

4. **Hitung error tracking**  
   Error antara plant dan model referensi digunakan sebagai sinyal adaptasi dan koreksi kontrol.

5. **Update parameter adaptif (MIT Rule)**  
   \(\dot{\alpha}\) dihitung online dari error dan sinyal sensitivitas/filter.

6. **Hitung torsi kontrol total**  
   Torsi = computed torque (kompensasi nonlinier + PD tracking) + komponen adaptif.

7. **Integrasi dinamika plant**  
   ODE solver memperbarui state \(q, \dot q\) sepanjang horizon waktu.

8. **Post-processing hasil**  
   Sistem menghitung metrik performa, menampilkan plot/tab analisis, animasi 3D, dan opsional export data.

---

## 6) Konteks Toolchain Pengembangan

Alur akademik/referensi metode:
1. **Modeling (CATIA V5)**: ekstraksi parameter fisik \((m, I, a, d)\).
2. **Mathematics (Maple 13)**: penurunan simbolik model dinamik.
3. **Simulation (MATLAB/Simulink)**: verifikasi hukum kontrol.

Repository ini menyediakan implementasi numerik ekuivalen berbasis **Python (NumPy/SciPy + GUI PySide6)** untuk eksperimen MRAC 2-DOF.

---

## Menjalankan Proyek

```bash
pip install -r requirements.txt
python test_all.py
python run.py
```

- `test_all.py` menguji modul model, kontroler, simulasi, dan import GUI.
- `run.py` menjalankan GUI simulasi utama.
