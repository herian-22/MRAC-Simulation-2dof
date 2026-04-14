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

### Antena sebagai manipulator serial 2-DOF
- **Joint 1 (q1)**: azimuth (rotasi horizontal)
- **Joint 2 (q2)**: elevation/slope (rotasi vertikal)

### Kinematika (DH)
Transformasi Denavit–Hartenberg digunakan untuk menghitung pose antar-link hingga titik ujung (arah pointing antena) dalam ruang 3D.

### Dinamika (Euler–Lagrange)
Persamaan gerak plant mempertimbangkan:
- matriks inersia <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20M%28q%29" alt="M(q)"/>,
- efek Coriolis/sentrifugal <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20C%28q%2C%5Cdot%7B%7Bq%7D%7D%29%5Cdot%7B%7Bq%7D%7D" alt="C(q,\dot{{q}})\dot{{q}}"/>,
- gravitasi <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20G%28q%29" alt="G(q)"/>,
- torsi disipatif/friksi viskos.

Bentuk umum model:

<p align="center"><img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20M%28q%29%5Cddot%7B%7Bq%7D%7D%20%2B%20C%28q%2C%5Cdot%7B%7Bq%7D%7D%29%5Cdot%7B%7Bq%7D%7D%20%2B%20G%28q%29%20%2B%20F_v%5Cdot%7B%7Bq%7D%7D%20%3D%20%5Ctau" alt="formula"/></p>


---

## 2) Fitur Utama Implementasi

### Engine Simulasi
- Integrasi ODE penuh dengan `scipy.integrate.solve_ivp` (default `RK45`).
- Simulasi tunggal dan batch (`run_simulation`, `run_batch`).
- Pembatasan torsi aktuator (torque clipping) untuk menjaga realistis numerik.

### Strategi Kontrol
- **Computed Torque** untuk kompensasi nonlinier plant.
- **MRAC berbasis MIT Rule** untuk update parameter adaptif online.
- Tracking terhadap **model referensi orde-2** (bukan hanya setpoint statis).

### Jenis Lintasan Input
- `step`
- `sinusoidal`
- `multipoint` (interpolasi spline)

### Analitik dan Metrik
- SSE, peak error, overshoot, settling time 2%, RMS, ISE, max torque.
- Logging sinyal penting: <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20q" alt="q"/>, <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20%5Cdot%7B%7Bq%7D%7D" alt="\dot{{q}}"/>, <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20q_m" alt="q_m"/>, error, <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20%5Ctau" alt="\tau"/>, parameter adaptif, end-effector, torsi friksi.

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

- Error utama: <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20e%20%3D%20q%20-%20q_m" alt="e = q - q_m"/>
- Parameter adaptif <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20%5Calpha" alt="\alpha"/> diperbarui kontinu via MIT Rule.
- Laju adaptasi diatur gain <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20%5Cgamma" alt="\gamma"/>:
  - <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20%5Cgamma" alt="\gamma"/> terlalu besar → respons cepat tapi berpotensi osilatif.
  - <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20%5Cgamma" alt="\gamma"/> terlalu kecil → stabil tapi adaptasi lambat.

Dengan mekanisme ini, kontroler bisa menyesuaikan aksi kontrol saat terjadi ketidakpastian (misalnya perubahan friksi/parameter efektif plant), sehingga tracking ke model referensi tetap terjaga.

---

## 5) Bagaimana Prosesnya (Alur Kerja End-to-End)

1. **Tentukan konfigurasi simulasi**  
   Input trajectory, parameter fisik plant, gain PD, gain adaptif <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20%5Cgamma" alt="\gamma"/>, dan parameter model referensi (<img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20%5Czeta%2C%5Comega_n" alt="\zeta,\omega_n"/>).

2. **Bangun sinyal referensi**  
   Generator trajectory membuat <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20q_d%2C%20%5Cdot%7B%7Bq%7D%7D_d%2C%20%5Cddot%7B%7Bq%7D%7D_d" alt="q_d, \dot{{q}}_d, \ddot{{q}}_d"/> sesuai mode input (step/sinusoidal/multipoint).

3. **Jalankan model referensi**  
   Dinamika referensi orde-2 menghasilkan lintasan target internal <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20q_m%2C%20%5Cdot%7B%7Bq%7D%7D_m" alt="q_m, \dot{{q}}_m"/>.

4. **Hitung error tracking**  
   Error antara plant dan model referensi digunakan sebagai sinyal adaptasi dan koreksi kontrol.

5. **Update parameter adaptif (MIT Rule)**  
   <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20%5Cdot%7B%7B%5Calpha%7D%7D" alt="\dot{{\alpha}}"/> dihitung online dari error dan sinyal sensitivitas/filter.

6. **Hitung torsi kontrol total**  
   Torsi = computed torque (kompensasi nonlinier + PD tracking) + komponen adaptif.

7. **Integrasi dinamika plant**  
   ODE solver memperbarui state <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20q%2C%20%5Cdot%7B%7Bq%7D%7D" alt="q, \dot{{q}}"/> sepanjang horizon waktu.

8. **Post-processing hasil**  
   Sistem menghitung metrik performa, menampilkan plot/tab analisis, animasi 3D, dan opsional export data.

---

## 6) Konteks Toolchain Pengembangan

Alur akademik/referensi metode:
1. **Modeling (CATIA V5)**: ekstraksi parameter fisik <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20%28m%2C%20I%2C%20a%2C%20d%29" alt="(m, I, a, d)"/>.
2. **Mathematics (Maple 13)**: penurunan simbolik model dinamik.
3. **Simulation (MATLAB/Simulink)**: verifikasi hukum kontrol.

Repository ini menyediakan implementasi numerik ekuivalen berbasis **Python (NumPy/SciPy + GUI PySide6)** untuk eksperimen MRAC 2-DOF.

---

## 7) Flowchart Proses Simulasi (Sesuai Kode)

```mermaid
flowchart TD
    A["Mulai: run_simulation(config)"] --> B["Inisialisasi dynamics plant/controller, reference model, trajectory generator"]
    B --> C["Definisikan state gabungan x = q,dq,xm1,xm2,phi1,phi2,alpha"]
    C --> D["solve_ivp memanggil system_ode(t,x)"]
    D --> E["Hitung q_d dari trajectory_generator"]
    E --> F["Update model referensi: dxm1, dxm2"]
    F --> G["Update filter sensitivitas: dphi1, dphi2"]
    G --> H["Hitung error: e = q - qm"]
    H --> I["Adaptasi MIT Rule: dalpha"]
    I --> J["Hitung torque kontrol: tau = tau_m + tau_a"]
    J --> K["Clip torque -500 sampai 500"]
    K --> L["Plant forward dynamics: ddq"]
    L --> M["Kembalikan dx ke solver"]
    M --> N{"Waktu selesai?"}
    N -- belum --> D
    N -- ya --> O["Post-process: q_ref, error, tau, end-effector, friction, metrics"]
    O --> P["SimResult"]
```

---

## 8) Logika Sistem per Tahap

1. **Trajectory layer**  
   `trajectory_generator()` menghasilkan <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20q_d%2C%5Cdot%7B%7Bq%7D%7D_d%2C%5Cddot%7B%7Bq%7D%7D_d" alt="q_d,\dot{{q}}_d,\ddot{{q}}_d"/> berdasarkan mode `step/sinusoidal/multipoint`.

2. **Reference model layer**  
   Untuk tiap joint, `ReferenceModel.state_derivative()` memperbarui state <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20x_m%3D%5Bq_m%2C%5Cdot%7B%7Bq%7D%7D_m%5D" alt="x_m=[q_m,\dot{{q}}_m]"/> dari input <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20r%3Dq_d" alt="r=q_d"/>.

3. **Adaptive sensitivity layer**  
   State filter <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20%5Cphi" alt="\phi"/> diperbarui dari input <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20q_i%2F%5Comega_n%5E2" alt="q_i/\omega_n^2"/>, lalu sensitivitas yang dipakai adaptasi adalah <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20%5Cdot%7B%7B%5Cphi%7D%7D_i" alt="\dot{{\phi}}_i"/> (`phi_i[1]`).

4. **Error and adaptation layer**  
   Error utama: <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20e%3Dq-q_m" alt="e=q-q_m"/>.  
   Parameter adaptif <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20%5Calpha" alt="\alpha"/> diperbarui online dengan MIT Rule.

5. **Control synthesis layer**  
   Kontrol total = computed torque nominal (<img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20%5Ctau_m" alt="\tau_m"/>) + kompensasi adaptif friksi (<img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20%5Ctau_a" alt="\tau_a"/>).

6. **Plant propagation layer**  
   Plant dihitung dengan `forward_dynamics()` memakai model plant (termasuk friksi), menghasilkan <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20%5Cddot%7B%7Bq%7D%7D" alt="\ddot{{q}}"/>, lalu solver mengintegrasikan state.

7. **Monitoring layer**  
   Setelah integrasi selesai, sistem menghitung metrik performa (SSE, overshoot, settling time, RMS, ISE, max torque, peak error).

---

## 9) Algoritma Inti (Pseudo-algoritmik dari `run_simulation`)

1. Inisialisasi objek `Dynamics2DoF`, `ReferenceModel`, `MRACController`, trajectory function, dan FK.  
2. Bentuk state awal nol: <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20x_0%3D%5Bq%2Cdq%2Cxm1%2Cxm2%2C%5Cphi1%2C%5Cphi2%2C%5Calpha%5D" alt="x_0=[q,dq,xm1,xm2,\phi1,\phi2,"/>.  
3. Jalankan `solve_ivp(system_ode, ...)`.  
4. Pada setiap evaluasi `system_ode(t,x)`:
   - unpack <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20x" alt="x"/>,
   - set `controller.alpha = alpha`,
   - hitung <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20q_d" alt="q_d"/>, set <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20r%3Dq_d" alt="r=q_d"/>,
   - hitung <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20dxm_1%2C%20dxm_2" alt="dxm_1, dxm_2"/>,
   - hitung <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20d%5Cphi_1%2C%20d%5Cphi_2" alt="d\phi_1, d\phi_2"/>,
   - hitung <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20q_m%2C%5Cdot%7B%7Bq%7D%7D_m%2Ce" alt="q_m,\dot{{q}}_m,e"/>,
   - hitung <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20d%5Calpha" alt="d\alpha"/> via `adaptation_law(e, dphi_val)`,
   - hitung <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20%5Ctau" alt="\tau"/> via `compute_full_torque(...)`,
   - clip <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20%5Ctau" alt="\tau"/> ke <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20%5B-500%2C500%5D" alt="[-500,500]"/>,
   - hitung <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20%5Cddot%7B%7Bq%7D%7D" alt="\ddot{{q}}"/> via `plant_dynamics.forward_dynamics(...)`,
   - pack turunan state dan kembalikan ke solver.
5. Setelah solver selesai, bentuk semua sinyal output (<img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20q" alt="q"/>, <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20dq" alt="dq"/>, <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20q_%7B%7Bref%7D%7D" alt="q_{{ref}}"/>, <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20error" alt="error"/>, <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20%5Ctau" alt="\tau"/>, <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20%5Calpha_%7B%7Badapt%7D%7D" alt="\alpha_{{adapt}}"/>, <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20end%5C_effector" alt="end\_effector"/>, <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20friction%5C_torque" alt="friction\_torque"/>).
6. Hitung metrik dengan `compute_metrics(...)`.
7. Kembalikan `SimResult`.

---

## 10) Rumus Utama (Dipakai di Implementasi Kode)

### a) Model referensi orde-2 (`models/reference_model.py`)

<p align="center"><img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20%5Cdot%7B%7Bx%7D%7D_m%20%3D%20A_m%20x_m%20%2B%20B_m%20r%2C%20%5Cquad%20A_m%3D%5Cbegin%7Bbmatrix%7D0%261%5C%5C-%5Comega_n%5E2%26-2%5Czeta%5Comega_n%5Cend%7Bbmatrix%7D%2C%5C%3B%20B_m%3D%5Cbegin%7Bbmatrix%7D0%5C%5C%5Comega_n%5E2%5Cend%7Bbmatrix%7D" alt="formula"/></p>


Komponen percepatan referensi yang dipakai kontrol (`compute_full_torque`):

<p align="center"><img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20u_%7B%7Bref%7D%7D%20%5Cequiv%20r" alt="formula"/></p>


<p align="center"><img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20%5Cddot%7B%7Bq%7D%7D_%7B%7Bm%2Ci%7D%7D%3D%5Comega_n%5E2%5C%2Cu_%7B%7Bref%2Ci%7D%7D-2%5Czeta%5Comega_n%5C%2C%5Cdot%7B%7Bq%7D%7D_%7B%7Bm%2Ci%7D%7D-%5Comega_n%5E2%5C%2Cq_%7B%7Bm%2Ci%7D%7D" alt="formula"/></p>


### b) Hukum adaptasi MIT (`control/controller.py`)
Dengan <img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20e%3Dq-q_m" alt="e=q-q_m"/>:

<p align="center"><img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20%5Cdot%7B%7B%5Calpha%7D%7D_i%3D-%5Cgamma_i%5C%2Ce_i%5C%2C%5Cdot%7B%7B%5Cphi%7D%7D_i" alt="formula"/></p>


### c) Hukum kontrol total (`control/controller.py`)
Error kontrol terhadap model referensi:

<p align="center"><img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20e_q%3Dq_m-q%2C%5Cqquad%20e_%7B%7B%5Cdot%7B%7Bq%7D%7D%7D%7D%3D%5Cdot%7B%7Bq%7D%7D_m-%5Cdot%7B%7Bq%7D%7D" alt="formula"/></p>


Sinyal bantu:

<p align="center"><img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20v%3D%5Cddot%7B%7Bq%7D%7D_m%2BK_v%20e_%7B%7B%5Cdot%7B%7Bq%7D%7D%7D%7D%2BK_p%20e_q" alt="formula"/></p>


Komponen torsi:

<p align="center"><img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20%5Ctau_m%3DM%28q%29%5C%2Cv%2BC%28q%2C%5Cdot%7B%7Bq%7D%7D%29%5Cdot%7B%7Bq%7D%7D%2BG%28q%29%2C%5Cqquad%20%5Ctau_a%3D%5Cbegin%7Bbmatrix%7D%5Calpha_1%5Cdot%7B%7Bq%7D%7D_1%5C%5C%5Calpha_2%5Cdot%7B%7Bq%7D%7D_2%5Cend%7Bbmatrix%7D" alt="formula"/></p>


Torsi total:

<p align="center"><img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20%5Ctau%3D%5Ctau_m%2B%5Ctau_a%2C%5Cquad%20%5Ctau%5Cleftarrow%5Cmathrm%7B%7Bclip%7D%7D%28%5Ctau%2C-500%2C500%29" alt="formula"/></p>


### d) Dinamika plant (`models/dynamics.py`)

<p align="center"><img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20%5Cddot%7B%7Bq%7D%7D%20%3D%20M_%7B%7Btrue%7D%7D%5E%7B%7B-1%7D%7D%5Cleft%28%5Ctau%20-%20C_%7B%7Btrue%7D%7D%5Cdot%7B%7Bq%7D%7D%20-%20G_%7B%7Btrue%7D%7D%20-%20%5Ctau_f%5Cright%29%2C%5Cqquad%20%5Ctau_f%20%3D%20f_v%20%5Codot%20%5Cdot%7B%7Bq%7D%7D" alt="formula"/></p>


### e) Rumus metrik (`simulation/simulator.py`)

<p align="center"><img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20%5Ctext%7B%7BSSE%7D%7D_i%3D%5Ctext%7B%7Bmean%7D%7D%5Cleft%28%7Ce_i%7C%5Cright%29_%7B%7B10%5C%25%5Ctext%7B%7B%20waktu%20terakhir%7D%7D%7D%7D%2C%5Cquad%20%5Ctext%7B%7BRMS%7D%7D_i%3D%5Csqrt%7B%7B%5Ctext%7B%7Bmean%7D%7D%28e_i%5E2%29%7D%7D%2C%5Cquad%20%5Ctext%7B%7BISE%7D%7D_i%3D%5Csum%20e_i%5E2%5C%2C%5CDelta%20t" alt="formula"/></p>


<p align="center"><img src="http://latex.codecogs.com/gif.latex?%5Cdpi%7B110%7D%5Cbg_white%20%5Ctext%7B%7BOvershoot%7D%7D_i%28%5C%25%29%3D%5Cmax%5Cleft%280%2C%5Cfrac%7B%7B%5Cmax%28q_i%29-q_%7B%7Bd%2Ci%7D%7D%5E%7B%7Bfinal%7D%7D%7D%7D%7B%7Bq_%7B%7Bd%2Ci%7D%7D%5E%7B%7Bfinal%7D%7D%7D%7D%5Cright%29%5Ctimes%20100" alt="formula"/></p>


---

## 11) Input Sinyal: Step vs. Sinusoidal & Multi-Point

### Apa yang Ada di Jurnal Referensi (Soares et al., 2021)?

Jurnal referensi **Soares et al. (2021)** secara eksplisit menyebutkan:

> *"In the simulations step functions were used as inputs of the system. Each entry represents the desired angular displacement..."*

Artinya, jurnal tersebut **hanya menggunakan Step Input** — yaitu perintah posisi mendadak ke 60° dan 30° — untuk memvalidasi bahwa kontroler MRAC berhasil.

### Mengapa Sinusoidal & Multi-Point Ditambahkan ke Proyek Ini?

Kedua jenis sinyal berikut **tidak ada di jurnal referensi**, melainkan ditambahkan sebagai **novelty/pengembangan** proyek ini:

| Jenis Input | Ada di Jurnal? | Tujuan |
|---|---|---|
| `step` | ✅ Ya | Validasi / verifikasi terhadap jurnal |
| `sinusoidal` | ❌ Tidak | Uji robustness pada pelacakan lintasan dinamis berkelanjutan |
| `multipoint` | ❌ Tidak | Uji robustness pada perubahan waypoint (mendekati kondisi dunia nyata) |

### Alasan Penambahan:

1. **Pembuktian Lebih Keras (Robustness Test)**  
   Membuktikan kemampuan MRAC hanya dengan Step Input dianggap standar/biasa untuk skripsi/tesis teknik kendali. Dengan Sinusoidal dan Multi-Point, MRAC diuji terhadap referensi yang selalu berubah — jauh lebih menantang.

2. **Kondisi Dunia Nyata**  
   Pesawat, satelit, atau antena bergerak yang bergeser tidak pernah bergerak secara "Step". Mereka bergerak secara kontinu (sinusoidal) atau mengikuti rute titik-demi-titik (waypoints/multipoint).

3. **Meningkatkan Bobot Riset**  
   Dengan adanya dua sinyal ini, laporan dapat berargumen:  
   > *"Riset ini tidak sekadar meniru jurnal acuan (Soares et al.) yang hanya mengandalkan Step Input, melainkan mengembangkannya dengan menguji ketangguhan MRAC pada pelacakan lintasan dinamis berkelanjutan (Sinusoidal & Waypoints)."*

### Kesimpulan:
- **Step** → fitur **wajib** untuk validasi/verifikasi hasil dengan jurnal referensi.
- **Sinusoidal & Multi-Point** → fitur **pengembangan** untuk eksplorasi keunggulan MRAC yang dibangun.

---

## Menjalankan Proyek

```bash
pip install -r requirements.txt
python test_all.py
python run.py
```

- `test_all.py` menguji modul model, kontroler, simulasi, dan import GUI.
- `run.py` menjalankan GUI simulasi utama.
