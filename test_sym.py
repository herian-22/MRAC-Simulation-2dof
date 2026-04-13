import sympy as sp

th1, th2 = sp.symbols('th1 th2')
dq1, dq2 = sp.symbols('dq1 dq2')

# DH params
a1, d1, a2, d2 = 0.19, 1.60, 0.97, 0

def dh(a, alpha, d, theta):
    ca, sa = sp.cos(alpha), sp.sin(alpha)
    ct, st = sp.cos(theta), sp.sin(theta)
    return sp.Matrix([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,   sa,     ca,    d],
        [0,   0,      0,     1]
    ])

T1 = dh(a1, -sp.pi/2, d1, th1)
T2 = T1 * dh(a2, 0, d2, th2)

# centers of mass in local frames
xc1, yc1, zc1 = -0.14, -1.26, 0.07
p_c1 = T1 * sp.Matrix([xc1, yc1, zc1, 1])
p_c1 = p_c1[:3, 0]

xc2, yc2, zc2 = -0.76, 0, 0
p_c2 = T2 * sp.Matrix([xc2, yc2, zc2, 1])
p_c2 = p_c2[:3, 0]

v_c1 = p_c1.jacobian([th1, th2]) * sp.Matrix([dq1, dq2])
v_c2 = p_c2.jacobian([th1, th2]) * sp.Matrix([dq1, dq2])

R1 = T1[:3, :3]
R2 = T2[:3, :3]

omega_1 = sp.Matrix([0, 0, dq1])
omega_2 = omega_1 + R1 * sp.Matrix([0, 0, dq2])

I1_local = sp.diag(58.38, 2.09, 58.17)
I2_local = sp.diag(7.89, 62.40, 62.33)

I1_world = R1 * I1_local * R1.T
I2_world = R2 * I2_local * R2.T

m1 = 29.16
m2 = 97.39

K = 0.5 * m1 * v_c1.dot(v_c1) + 0.5 * m2 * v_c2.dot(v_c2) + 0.5 * omega_1.dot(I1_world * omega_1) + 0.5 * omega_2.dot(I2_world * omega_2)

M = sp.Matrix(2, 2, lambda i, j: sp.diff(sp.diff(K, [dq1, dq2][i]), [dq1, dq2][j]))

print("M11:", sp.simplify(M[0,0]).evalf(5))
print("M12:", sp.simplify(M[0,1]).evalf(5))
print("M22:", sp.simplify(M[1,1]).evalf(5))

g = 9.81
P = m1 * g * p_c1[2] + m2 * g * p_c2[2]

G = sp.Matrix([sp.diff(P, th1), sp.diff(P, th2)])
print("G1:", sp.simplify(G[0]).evalf(5))
print("G2:", sp.simplify(G[1]).evalf(5))
