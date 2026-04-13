import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

Z = 2.0
W = 4.0
m = 66.0
b = 4.5  # Realistic friction
u = 60.0

def run_sim(gamma):
    def ode(t, x):
        q, dq, qr, dqr, phi, dphi, alpha = x
        e = q - qr
        
        # Reference model
        ddqr = -Z * dqr - W * qr + W * u
        
        # Sensitivity
        ddphi = -Z * dphi - W * phi + q
        
        # Adaptation
        dalpha = -gamma * e * dphi
        
        # Plant
        # tau_m = m * (W*u - Z*dqr - W*qr) = m * ddqr
        # m ddq + b dq = m ddqr + alpha dq
        ddq = ddqr + (alpha - b)/m * dq
        
        return [dq, ddq, dqr, ddqr, dphi, ddphi, dalpha]
    
    sol = solve_ivp(ode, [0, 10], [0, 0, 0, 0, 0, 0, 0], max_step=0.01)
    return sol.t, sol.y[0]

for g in [200, 500, 750, 1000, 1500]:
    t, q = run_sim(g)
    plt.plot(t, q, label=f"g={g}")

# Target reference model
t_ref, qr = run_sim(0) # without adaptation, qr is independent
plt.plot(t_ref, qr, 'k--', label="Ref")

plt.axhline(60, ls=":", color="k")
plt.legend()
plt.savefig("test_gamma.png")
