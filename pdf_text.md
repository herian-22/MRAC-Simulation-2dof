Revista Ibero- Americana de Humanidades, Ciências e Educação- REASE
doi.org/10.29327/217514.7.1-1
ADAPTIVE CONTROLLER FOR AUTOMATIC MANEUVER OF A
SATELLITE DISH RECEIVER
Controlador adaptativo para manobra automática de um receptor de pratos de satélite
Alvaro Manoel de Souza Soares1
João Bosco Gonçalves2
Paulo Henrique Crippa3
ABSTRACT: The objective is to develop a control system capable of performing the automatic
maneuver of a satellite dish and more accurately with less time maneuvering when compared to
manual maneuver. The dish consists of a study on metal parabola 1.60 m in diameter, two sets
of gears and two electric motors to perform the movements. The physical parameters of the
mechanical system could be easily obtained from a three-dimensional modeling in a CAD
software platform. For modeling the system dynamics we used the similarity of the physical
system under study with an serial manipulator of two degrees of freedom that allowed it to apply
concepts related to kinematics and modeling of robotic manipulators. Through the Denavit-
Hartenberg notation of the direct kinematics of the antenna with two degrees of freedom was
successfully obtained. The dynamic equations describing the motion of the system were raised
through an automatic model implemented in symbolic manipulation software. To that end, an
algorithm that describes the steps necessary to obtain the equations of motion of a robotic
manipulator in open chain, from the Lagrangian method, was developed. A model reference
1
adaptive control system was designed and implemented considering the uncertainties of the
model arising from imperfections within the three-dimensional modeling. The results obtained
by simulation of the system of closed loop control were satisfactory as well as the high rates of
the perfect maneuver have been achieved.
Keywords: Serial manipulators. Dynamic equations of motion. Model reference adaptive
control.
RESUMO: O objetivo deste trabalho é o de demonstrar o projeto de um sistema de controle
adaptativo capaz de realizar o apontamento automático de uma antena parabólica de forma mais
precisa e com menor tempo de apontamento quando comparado ao apontamento manual. A
antena parabólica em estudo consta de uma parábola metálica de 1,60 m de diâmetro, dois
conjuntos de engrenagens e dois motores elétricos para realização dos movimentos. Os
parâmetros físicos do sistema mecânico foram facilmente obtidos a partir de uma modelagem
tridimensional em um ambiente CAD. Para a modelagem dinâmica do sistema utilizou-se a
similaridade do sistema físico em estudo com um manipulador de cadeia aberta de dois graus de
liberdade, permitindo aplicar conceitos referentes à cinemática e modelagem de manipuladores
1 Doutorado em Engenharia Aeronáutica e Mecânica pelo Instituto Tecnológico de Aeronáutica (1997).
Professor assistente da Universidade Estadual Paulista Júlio de Mesquita Filho e Professor assistente
doutor da Universidade de Taubaté. E-mail: alvaro@unitau.br.
2 Doutorado em Engenharia Mecânica, Unicamp, 2004.Professor Adjunto da Universidade Federal do
Espírito Santo. E-mail: joao.b.goncalves@ufes.br.
3 Mestre em Engenharia Mecânica, área de concentração: Automação pela Universidade de Taubaté,
2011.Professor titular da Faculdade Canção Nova. E-mail: eng.paulo@cancaonova.com.
Revista Ibero-Americana de Humanidades, Ciências e Educação. São Paulo, v.7.n.1, Jan. 2021.
ISSN - 2675 – 3375
Revista Ibero- Americana de Humanidades, Ciências e Educação- REASE
robóticos. Através da notação de Denavit-Hartenberg a cinemática direta da antena com dois graus
de liberdade foi obtida com sucesso. As equações dinâmicas que descrevem o movimento do
sistema foram levantadas através da modelagem automática desenvolvida utilizando-se um
software de manipulação simbólica. Para tanto foi desenvolvido um algoritmo que descreve os
passos necessários para obtenção das equações de movimento de um manipulador robótico em
cadeia aberta, a partir da formulação Lagrangeana. Um sistema de controle adaptativo por modelo
de referência foi projetado e analisado, admitindo-se incertezas do modelo oriundas de
imperfeições contidas na modelagem tridimensional. Os resultados obtidos por simulação do
sistema de controle adaptativo se mostraram satisfatórios e os índices de desempenho esperados
para um perfeito apontamento foram alcançados.
Palavras-chave: Manipuladores robóticos em cadeia aberta. Equações dinâmicas de movimento.
Controle adaptativo por modelo de referência.
INTRODUCTION
The media play a key role in the development of modern civilization. Many segments
of society use this feature to a rapid and efficient dissemination of information. The satellite has
been used as an effective solution for communication between distant points with each other,
since the existing geostationary satellite coverage reaches kilometer (Ha, T.T., 1986.)
One of the disadvantages of satellite communications is the manual maneuvering of the
satellite dishes of receipt. Besides being a process of high precision this maneuver demand a long
time, especially when referring to the mobile receiving antennas need to adapt to geographic 2
coordinates and climatic conditions and relief that will be exhibited in each use (Marins,
C.N.M., 2004)
In this sense many efforts have been applied by telecommunications companies to
improve the maneuver of antennas. The solutions range from systems based on microcontrollers
(Cúnico, M., 2006) until the controllers proportional integrative (PI) and proportional
integrative and derivative (PID) (Queiroz, K.I.P.M., 2006) (Souto, M.C.B., 2009). Classic
modern control systems are presented as the solution to perform the pointing implements
(Armellini, F., 2006) ( Armellini, F., 2010) (Malaquias, I.M., 2009.)
This paper presents a control system able to perform the automatic appointment a
satellite dish reception in order to minimize the disadvantages found in the manual
maneuvering. Thus, it is accepted for study a parabola antenna made of metal with 1.6 m in
diameter, an iron-based support for attachment to flat surface, and two sets of gears arranged in
a way that enables the movement of the antenna in both directions of displacement: slope and
azimuth. Figure 1 show the schematic of the physical system under study.
Revista Ibero-Americana de Humanidades, Ciências e Educação. São Paulo, v.7.n.1, Jan. 2021.
ISSN - 2675 – 3375
Revista Ibero- Americana de Humanidades, Ciências e Educação- REASE
Fig. 1. Scheme of Antenna Receiver.
Source: Author.
The physical system has three rigid links, with a fixed link (link in the base), and two rotary
joints with viscous friction coefficient. Comparing a serial manipulator of the two degrees of
freedom (Schilling, R.J., 1990) (Fu, K., Gonzalez, R.C., Lee C.S.G., 1987) with the physical 3
system under consideration, it is recognized that such a similarity allowed the assignment of the
concepts of kinematics and dynamic modeling of robotic manipulators.
To develop the control system was used a control action consists of three terms associated
with adaptive control action, a reference model (MRAC) and Computed Torque technique. A
controller MRAC main characteristic is to impose on the system performance indicators in the
reference model. In this work we chose to use an adaptive control action whose unknown
parameters of the plant could be estimated online.
The analysis of the dynamic behavior of the model obtained, and the control system, took
from its implementation in MATLAB/SIMULINK®.
PHYSICAL SYSTEM SATELLITE RECEIVER
The prototype of the receiving antenna was designed using the software CATIA® V5
R19 in order to conduct a three-dimensional modeling of the physical system, demonstrating all
the rigid links and rotary joints of the system. The physical parameters of the prototype were
calculated automatically by the software, from the characteristics of materials used, geometric
shapes, dimensions and coordinate systems defined for each hard link.
Revista Ibero-Americana de Humanidades, Ciências e Educação. São Paulo, v.7.n.1, Jan. 2021.
ISSN - 2675 – 3375
Revista Ibero- Americana de Humanidades, Ciências e Educação- REASE
The satellite dish receiver consists of rigid links represented by: one support base, one
support for the parables, the parable’s metal, LNB (Low Noise Buffer), and rotary joints along
the rented gear sets.
The physical system of the receiving antenna was modeled as an articulated system of
rigid links together through the three-dimensional space, which corroborates with the definition
of a robotic serial manipulator [11]. Figure 2 show the result of three-dimensional modeling of
the physical system of the receiving antenna.
Fig. 2. Three-dimensional model of the Antenna Receiver.
4
Source: Author.
Direct Kinematics
The software CATIA® V5 R19 allows the inclusion of systems of coordinates in three-
dimensional model [12]. Were assigned the coordinate systems for each link of the physical
model of the receiving antenna following the methodology of Denavit-Hartenberg (DH)
(Adade Filho, A., 1999) (Lopes, A.M., 2002.) Figure 3 shows the result obtained after applying
the procedure DH.
Revista Ibero-Americana de Humanidades, Ciências e Educação. São Paulo, v.7.n.1, Jan. 2021.
ISSN - 2675 – 3375
Revista Ibero- Americana de Humanidades, Ciências e Educação- REASE
Fig. 3. Receiving antenna with DH.
Source: Author
The DH parameters of the antenna receiver are shown in Table 1.
Table 1. DH parameters of the antenna receiver.
5
Elo(i) a i (m) α i (rad) d i (m) θ i (rad) Variável
1 0,19 -π/2 1,60 θ θ (rotação)
1 1
2 0,97 0 0 θ θ (rotação)
2 2
Source: Author
With the coordinate systems properly inserted the physical model of the receiving
antenna, CATIA® V5 R19 calculates the moments of inertia and centers of gravity of each rigid
link already referred to the coordinate system attached to the links. Table 2 presents the physical
parameters of three-dimensional model. The moment of inertia for the nth rigid link was
calculated over the n-th Cartesian system, located in the corresponding center of mass. The
crossed moments of inertia are null.
Table 2. Physical parameters of the antenna receiver.
Massa (kg) Momento de Inércia (kg.m2) Centro de Gravidade (m)
Elo (i)
m I I I x y z
i x y z c c c
1 29,16 58,38 2,09 58,17 -0,14 -1,26 0,07
2 97,39 7,89 62,40 62,33 -0,76 0,00 0,00
Source: Author
Revista Ibero-Americana de Humanidades, Ciências e Educação. São Paulo, v.7.n.1, Jan. 2021.
ISSN - 2675 – 3375
Revista Ibero- Americana de Humanidades, Ciências e Educação- REASE
DYNAMIC MODELING
The purpose of dynamic modeling is to obtain the equations of motion for each degree
of freedom manipulator, allowing relating the movements (displacements) with the generalized
forces (torques) applied at each joint (Latre, L.G., 1988.) According to (Lee, C.S.G.,1983) full
knowledge of the dynamic model of a robot is essential for the computational implementation
of its movement and the control system design.
There are several techniques for dynamic modeling of robotic manipulators (Santos,
R.R., 2005.) Typically the two techniques are widely used in literature to obtain the dynamic
model are the Euler-Lagrange method and the Newton-Euler method. The Newton-Euler
method is to describe the dynamics of a mechanism based on the forces and moments applied to
rigid bodies (links). It is based on two equations: Newton's equation that describes the
translation of the center of mass of rigid body, and the Euler equation that describes the rigid
body rotation around the center of mass. The Euler-Lagrange method is described in a scalar
function of the Lagrangian which is formed by the difference between kinetic energy and
potential energy for each joint system.
The dynamic model of a robotic serial manipulator is expressed by (Santos, R.R., 2005)
n n n
F = D q +H q q +C 6
i ik k ikm k m i
k=1 k=1 m=1
⑴
In matrix shape:
F(t) = D(q(t))q(t)+H(q(t),q(t))+C(q(t)). ⑵
where:
F(t) n x 1 generalized forces vector applied at joints;
D(q(t)) n x n symmetric matrix representing the inertia;
H(q(t),q(t)) n x 1 nonlinear Coriolis and centrifugal force vector;
C(q(t)) n x 1 gravity loading force vector;
q(t) n x 1 vector of position of the joint variables;
q(t) n x 1 vector of velocity of the joint variables;
q(t) n x 1 vector of acceleration of the joint variables.
Euler-Lagrange method
To determine the dynamic model was used Euler-Lagrange formalism that describes the
dynamic behavior of the system in terms of energy stored in the system (Adade Filho, A., 1999).
The Euler-Lagrange equation is expressed as:
Revista Ibero-Americana de Humanidades, Ciências e Educação. São Paulo, v.7.n.1, Jan. 2021.
ISSN - 2675 – 3375
Revista Ibero- Americana de Humanidades, Ciências e Educação- REASE
d  L  L
dt 

q 
−
q
= F
j
⑶
 j  j
Where j is the index related to the rigid element (link), L is the Lagrangian of the system,
given by the difference between kinetic energy and potential energy of the system, and F are the
j
external loads from the potential non-conservative.
Using the Euler-Lagrange equation, the dynamic model of robotic serial manipulator
with rigid links is given by (Fu, K., Gonzalez, R.C., Lee C.S.G., 1987)
n j ( ) n j j ( ) n
 = Tr U J UT q +Tr U J UT q q −m gU jr ⑷
i jk j ji k jkm j ji k m j ji j
j=1 k=1 j=1 k=1 m=1 j=1
Comparing Eq. (1) with Eq. (4), are obtained separately the terms of the dynamic
equation of motion:
n ( )
D = Tr U J UT ⑸
ik jk j ji
j=max(i,k)
n ( )
H = Tr U J UT ⑹
ikm jkm j ji
j=max(i,k,m)
7
n
C =−m gU jr ⑺
i j ji j
j=1
where:
U matrix that represents the effects of the motion of joint i on all the points on link
ji
j;
U matrix that represents the interaction effects of the motion of joint k e m on all
jkm
the
points on link j;
J matrix that contains moments of inertia of link j;
j
m mass of link j;
j
g
acceleration of gravity vector referenced to the base coordinate system.
Dynamic Modelo f the Antenna Receiver
To obtain the dynamic model of the satellite dish receiver with two degrees of freedom
will be considered the DH parameters and physical parameters contained, respectively, in Table
1 and Table 2.
Revista Ibero-Americana de Humanidades, Ciências e Educação. São Paulo, v.7.n.1, Jan. 2021.
ISSN - 2675 – 3375
Revista Ibero- Americana de Humanidades, Ciências e Educação- REASE
An automatic model was developed using the software Maple ® 13 (Mariani, V.C., 2005)
to implement the Euler-Lagrange formulation developed for a robotic serial manipulator,
described in Eq. (4).
Applying the automatic model was obtained Eqs (8) and (9) that describe the dynamic
model for the physical system of the receiving antenna.
  
 = D  +2H  ⑻
1 11 1 112 1 2
 = D 

+H 
2
+C ⑼
2 22 2 211 1 2
Assuming g = 9.81 m/s2, then the equations will be presented for each term of the
matrices and vectors that describe the dynamic equations of motion. The literal terms contained
therein a, a, m and m respectively, DH parameter of the link 1, parameter DH of the link 2, and
1 2 1 2
mass of link 1 and mass of link 2. The joints variables are θ and θ (rotary joints).
1 2
D
Terms of Inertia Matrix :
ik
1
D =2a a m cos()+a 2m +a 2m + a 2m (1+cos(2))−0.29a m
11 1 2 2 2 1 1 1 2 2 2 2 2 1 1 ⑽
−1.514a m cos()−0.757a m (1+cos(2))+27.2565cos(2)+37.2375
1 2 2 2 2 2 2
D
12
=0
8
⑾
D =0
21
⑿
D = a 2m −1.514a m +62.328 ⒀
22 2 2 2 2
H :
Terms of the vector representing the Coriolis effects and centrifugal force,
ikm
H =0 ⒁
111
1
H = −a a m sin()− a 2m sin(2)+0.757m (a sin()+a sin(2))
112 1 2 2 2 2 2 2 2 2 1 2 2 2 ⒂
−27.2565sin(2)
2
1
H =−aa m sin()− a 2m sin(2)+0.757m (a sin()+a sin(2))
121 1 2 2 2 2 2 2 2 2 1 2 2 2 ⒃
−27.2565sin(2)
2
H =0 ⒄
122
Revista Ibero-Americana de Humanidades, Ciências e Educação. São Paulo, v.7.n.1, Jan. 2021.
ISSN - 2675 – 3375
Revista Ibero- Americana de Humanidades, Ciências e Educação- REASE
1
H = a a m sin()+ a 2m sin(2)−0.757m (a sin()+a sin(2))
211 1 2 2 2 2 2 2 2 2 1 2 2 2 ⒅
+27.2565sin(2)
2
H =0 ⒆
212
H =0 ⒇
221
H = 0 ⒇
222
Terms of the vector representing the effects of gravity acceleration C :
i
C =0 21
1
( ) ( )
C =0.757m gcos −m a gcos 22
2 2 2 2 2 2
The Eqs. (8) e (9) can be placed in matrix shape:
  D 0  2H 0       0 
1 = 11 + 112 . 1 2 +
       0 D     0 H      2     C   23
2 22 211 1 2
ADAPTIVE CONTROL SYSTEM
The main objective of a system of closed loop control is to maintain a satisfactory level
9
of performance even when subjected to disturbances and variations in the control system (Dias,
S.M., 2010.)
However, some plants have such wide variations and significant effects on the dynamic
behavior that a classic controller with feedback gain linear and constant coefficients are unable
to provide the necessary flexibility to the system (Tambara, R.V., Gründling, H.A., Della Flora,
L., 2010.)
The basic idea of operation of the adaptive control is to calculate the control signal using
estimates of uncertain parameters of the plant or directly to the controller parameters obtained
through real-time information from the measurable signals of the system (Slotine, J J., Li, W.,
1991.)
Model Reference Adaptive Control (MRAC)
The strategy Model Reference Adaptive Control (MRAC) is considered one of the main
approaches in the literature on adaptive control (Mareels, I.M.Y., Polderman, J.W., 1996.) In
some applications, the plant parameters are not completely known. An alternative to control
solution in these cases is the use of MRAC, where, besides the features of the MRC, the system
inserts a parametric adaptation algorithm which estimates the uncertain parameters of the model
(Tambara, R.V., Gründling, H.A., Della Flora, L., 2010.)
Revista Ibero-Americana de Humanidades, Ciências e Educação. São Paulo, v.7.n.1, Jan. 2021.
ISSN - 2675 – 3375
Revista Ibero- Americana de Humanidades, Ciências e Educação- REASE
In the MRAC system performance is expressed in terms of a reference model, which
generates a desired response to a given reference signal. The error between model output and the
output of the plant (e ) is measured, and through methods of parameter estimation (MIT Rule)
o
controller parameters are modified so that the system behaves like the reference model (Guerra
Vale, M.R.B., Fonseca, D.G.V., Maitelli, A.L., 2008.)
Figure 4 shows the schematic of a MRAC controller.
Fig. 4. Block diagram of a generic MRAC.
Source: Author
10
Thus, the error between the plant output and the output of the reference model is used
to adapt the algorithm to adjust the controller parameters, so that this error tends to zero, thus
allowing the tracking of the asymptotic model (Gonçalves, J.B., 2006.)
MIT Rule
The essential problem of MRAC is to determine the adjustment mechanism so as to
obtain a stable system in which the error signal between the plant output and the output of the
reference model is minimized. The adjustment mechanism called MIT Rule is the original
approach used in MRAC (Bueno, L.P.P., 2006.)
This rule states that for a given error signal e , a cost function J(α) is calculated, being
0
α the parameter of the controller to be adjusted. The cost function is defined by:
1
J() = e2
24
2 0
In order to minimize the cost related to the error, the parameter α can be changed
according to the negative gradient of J, so (Ioannou, P., Sun, J., 1995)
Revista Ibero-Americana de Humanidades, Ciências e Educação. São Paulo, v.7.n.1, Jan. 2021.
ISSN - 2675 – 3375
Revista Ibero- Americana de Humanidades, Ciências e Educação- REASE
d J e
= − = −e 0
. 25
dt  0 
e /
The Eq. (26) expresses the MIT Rule. The partial derivative is called the
0
derived sensitivity of the system and shows how the error (e ) is influenced by the adjustable
0
parameter (α). The parameter γ determines the rate of adaptation of the system (adaptive gain).
The mechanism for setting parameters through the MIT Rule is non-linear due to
multiplication of the error with the partial derivative. Application of this mechanism can result
in unstable systems, particularly if the adaptive gain γ is relatively high (Resende, J. M. O. S.
A., 1995.)
MRAC Controller Antenna Receiver
The dynamic model obtained for the physical system of the receiving antenna has a non-
linear as can be seen in Eqs (8) and (9). A linearization technique called Computed Torque was
applied. The purpose of this linearization is to transform all or part of a nonlinear dynamic
system, resulting in a system to which to apply linear control techniques (Slotine, J J., Li, W.,
1991.) 11
The technique MRAC was used in the control design of receiving antenna, in order to
identify the unknown parameters of the plant, so online. Figure 5 shows the block diagram of
the control technique used in automatic maneuvering of the receiving antenna.
Fig. 5. MRAC block diagram of antenna receiver.
Source: Author
Revista Ibero-Americana de Humanidades, Ciências e Educação. São Paulo, v.7.n.1, Jan. 2021.
ISSN - 2675 – 3375
Revista Ibero- Americana de Humanidades, Ciências e Educação- REASE
Model Uncertainty
For the design of the control system of the antenna receiver is efficient it is necessary
that the dynamic model is as close as possible to the real physical system. It is assumed that the
term refers to energy dissipation is not known and therefore not included in the equations that
define the dynamic model of the receiving antenna (Eqs. 8 and 9). Therefore the dynamic model
of the receiving antenna must be rewritten with the inclusion of the unknown term energy
dissipative.
ˆ
Admitting B as the vector that represents the dissipative forces unknown to the model
given by:
b ˆ 
B ˆ =  1 26
ˆ
 b 
2
The dynamic equations of motion of the antenna receiver can be rewritten as:
   ˆ 
 = D  +2H  +b 27
1 11 1 112 1 2 1 1
 = D 

+H 
2
+C +b
ˆ


28
2 22 2 211 1 2 2 2
In matrix shape:
12
 D 0  2H 0      C  b ˆ 0  
   1 2   =   0 11 D 22   +   0 112 H 211   .   1  1 2 2   +   C 1 2   +    0 1 b ˆ 2    .    1 2   29
Reference Model
For the design of the control system of the receiving antenna was used as a reference
model of a 2nd order system widely discussed in (Nise, N.S., 2009.) Such systems have the well-
defined performance index which allows to easily establish desirable conditions for the plant
output, is in transition and in steady state. Eq. (31) presents the reference model adopted
expressed in the time domain and in the function of natural frequency ω and damping ratio ζ:
n
  =2u −2  −2 30
Ri ni i ni Ri ni Ri
In matrix shape:
 
 =Ωu−Z −Ω
31
R R R
where:
Revista Ibero-Americana de Humanidades, Ciências e Educação. São Paulo, v.7.n.1, Jan. 2021.
ISSN - 2675 – 3375
Revista Ibero- Americana de Humanidades, Ciências e Educação- REASE
2 0 
:=  n1 
0 2
 
n2
32
2 0 
Z :=  1 n1  33
0 2
 
2 n2
u 
u:=  1  34
u
 
2
 
 :=  R1  35
R 
 
R2
The natural frequency ω and damping ratio ζ can be determined from an index of
n
performance imposed on the closed loop system. In order to perform an automatic pointing of
the antenna receiving satisfactory, it is an overshoot %UP = 15%, and a transient peak time T
p
= 1.8 s.
The relationship between the overshoot %UP and the transient peak time T, with the
p
natural frequency ω , and damping ratio ζ, are given by:
n
%UP =e−(/ 1−2 )
36
13

T =
37
p
 1−2
n
Solving Eqs. (37) and (38), result: ω = 2 rad/s e ξ = 0.5.
n
Control law
From the control scheme shown in Figure 5, the adaptive control law used in the antenna
receiver is given by:
 = +
38
i m a
i i

In order is the term of the model expressed in terms of nominal values obtained in the
m
i

dynamic model. Already is the term adaptive expressed as a function of adaptive parameter
a
i
to be estimated.
Applying the technique of Computed Torque wishing to eliminate the nonlinearity of the
ˆ  ˆ 
plant and, considering the uncertain terms of the model b and b , the terms of the model
1 1 2 2
control law, and  , can be written as:
m
1
m
2
Revista Ibero-Americana de Humanidades, Ciências e Educação. São Paulo, v.7.n.1, Jan. 2021.
ISSN - 2675 – 3375
Revista Ibero- Americana de Humanidades, Ciências e Educação- REASE
( )
  
 = D Ωu −Z −Ω +2H 
39
m 11 1 R1 R1 112 1 2
1
( )
 = D Ωu −Z

−Ω + H 
2
+C
40
m 22 2 R2 R2 211 1 2
2
The adaptive terms of the control law,  e  , are:
a a
1 2

 =
41
a 1 1
1

 =
42
a 2 2
2
Where, α e α are the parameters of adaptive control to be set by the estimator.
1 2
Therefore, the control law given in Eq. (39) can be rewritten for each degree of freedom
system:
( )
   
 = D Ωu −Z −Ω +2H  +
43
1 11 1 R1 R1 112 1 2 1 1
( )
 = D Ωu −Z

−Ω +H 
2
+C +

44
2 22 2 R2 R2 211 1 2 2 2
Estimator – MIT Rule
To estimate the parameters of the adaptive controller was used to MIT rule. In this rule it is
desired that the estimated parameter α converges to α* (optimal value), this implies that
14
lim e (t)=0, where e is the error signal (Guerra Vale, M.R.B., 2008.)
t→ 0 0
The adaptive parameter of controller is a function of the partial derivative of the plant output
θ with respect to parameter α multiplied by the error signal. The estimates of the parameters of
i i
the controller can be represented by (Ioannou, P., Sun, J., 1995)
 s 
 = − e
1  s2 +Zs +Ω 1  1 45
 s 
 = −  e
2  s2 +Zs +Ω 2  2 46
Simulation and results
The implementation of the control system was performed in MATLAB® software
through the toolbox SIMULINK®, which features an intuitive graphical language programming
offering an alternative to the classical approach to numerical simulation of engineering problems
(Matsumoto, É.Y., 2004.)
Revista Ibero-Americana de Humanidades, Ciências e Educação. São Paulo, v.7.n.1, Jan. 2021.
ISSN - 2675 – 3375
Revista Ibero- Americana de Humanidades, Ciências e Educação- REASE
In the simulations step functions were used as inputs of the system. Each entry
represents the desired angular displacement for each joint of the system, such that: u = 60°, u =
1 2
30°.
Figure 6 and 7 show the outputs of the plant being controlled by MRAC for the joint 1
and joint 2, respectively, with a fixed gain adaptive (γ) given by: γ = 750, γ = 900.
i 1 2
Fig. 6. Outputs of the control system – joint 1.
(a) Angular Displacement
(b) Angular Velocity
Fig. 7. Outputs of the control system – joint 2.
15
(d) Angular Velocity
(c) Angular Displacement
Source: Author
The main characteristic to be observed in an MRAC is the behavior of the controlled
variable relative to the reference model. The latter is chosen so as to impose on the system the
desired levels of performance. Figure 8 shows the output signal (controlled variable) compared
to the reference model adopted.
Fig. 8. Analysis of output signal (controlled variable).
Revista Ibero-Americana de Humanidades, Ciências e Educação. São Paulo, v.7.n.1, Jan. 2021.
ISSN - 2675 – 3375
Revista Ibero- Americana de Humanidades, Ciências e Educação- REASE
(a) Controlled Variable – joint 1 (b) Controlled Variable – joint 2
Source: Author
The error signal is obtained by the difference between the reference model and the output
signal of the system. Figure 9 shows the error signal for the joint 1 and 2 of the antenna receiver.
Fig. 9. Error signal.
16
(a) Error Signal of the joint 1 (e) (b) Error Signal of the joint 2 (e)
1 2
Source: Author
It is observed that the error signals of Figure 9 have small values which show the
effective control action for both joints of the system of antenna receiver.
The model uncertainties are compensated by the parameter adaptive control. For the
purpose of simulation was added to the model with a dissipative term to evaluate the behavior
of parameter adaptive control.
Figure 10 (a) provides for the joint 1, the behavior between the dissipative term inserted
 b ˆ 
 1 θ   , and the parameter adaptive control , Figure 10 (b) provides for the joint 2, the
 D 1 1
 11 
Revista Ibero-Americana de Humanidades, Ciências e Educação. São Paulo, v.7.n.1, Jan. 2021.
ISSN - 2675 – 3375
Revista Ibero- Americana de Humanidades, Ciências e Educação- REASE
 b ˆ 
behavior between the dissipative term inserted  2 θ   , and the parameter adaptive control
 D 2
 22 

2. The values of the adaptive gains are set: γ = 750, γ = 900.
1 2
Fig. 10. Analysis of the adaptive controller parameter.
(a) Controller parameter – joint 1 (b) Controller parameter – joint 2
Source: Author
Analyzing Figure 10 shows that the contribution of the dissipative term included in the 17
model rapidly tends to zero (about 5s). This is due to the fact that the velocity also tends to zero
in this short period of time. Satisfactorily the contribution of the adaptive control parameter
influences the system output at the same time interval in which the dissipative term acts. Note
also that the adaptive control parameter tends to zero at the same instant in which the dissipative
term is significant in the system.
The variation of the adaptive gain γ has a direct influence on the behavior of the MRAC.
As the system of the antenna receiver has a fixed gain at its plant the adjusting of the γ occurred
empirically, that is, as varying the γ analyzing the behavior of the output.
Figure 11 and 12 show the influence of γ at the output of a system to seal and joint 2
respectively.
With very high values of γ the systems begin to show features of instability. To perform
the pointing of the antenna receiver with satisfactory levels of play can be considered an adaptive
gain to the joint 1 such that: γ ∈ [710, 830] and for joint 2: γ ∈ [880, 990].
1 2
Revista Ibero-Americana de Humanidades, Ciências e Educação. São Paulo, v.7.n.1, Jan. 2021.
ISSN - 2675 – 3375
Revista Ibero- Americana de Humanidades, Ciências e Educação- REASE
Fig. 11. Influence of γ on output system – joint 1.
Source: Author
Fig. 12. Influence of γ on output system – joint 2.
18
Source: Author
CONCLUSION
This work was presented an adaptive control system for the automatic maneuvering of
a receiver dish. For the basis of studies have been adopted the physical and construction
characteristic of a antenna receiver widely used in satellite communications professionals
systems.
The three-dimensional modeling of the physical system made it possible for the antenna
receiver to obtain the physical parameters of the system. This modeling was done in CATIA®
V5 R19. The features offered by this software enabled the physical model obtained contemplate
Revista Ibero-Americana de Humanidades, Ciências e Educação. São Paulo, v.7.n.1, Jan. 2021.
ISSN - 2675 – 3375
Revista Ibero- Americana de Humanidades, Ciências e Educação- REASE
all the constituent parts of the schematic constructive prepared for the mechanism of the antenna
receiver, making it possible to identify the links in the system. The systematic location of
Cartesian coordinate systems in the joints of the system, according to the Denavit-Hartenberg
rules, was also easily performed by this software.
The similarity of the physical system obtained with a robotic serial manipulator allowed
to use the concepts of kinematics and dynamic modeling of manipulators. The Euller-Lagrange
formalism was used in modeling the dynamics of the antenna receiver. To that end, we
developed an algorithm that describes the steps necessary to obtain the dynamic equations of
motion for a robotic serial manipulator. The implementation of this algorithm occurred in the
software MAPLE® 13. As a result of this implementation was developed automatic modeler that
is capable of resulting equations of motion for these manipulators.
The implementation of the MRAC control system adopted to control the movements of
the antenna receiver, took from its implementation in MATLAB® using the toolbox
SIMULINK®.
As the dynamic model obtained was a non-linear model by a feedback linearization
technique (Computed torque) was applied in order to eliminate the nonlinear terms of the
model. The reference model adaptive control action was adopted to control the plant. The
reference model chosen was a standard system of 2nd order, which in addition to the simplicity 19
of implementation allows the desired levels of performance for the system to be easily
established. The parameters of adaptive controller were estimated by MIT rule that evaluates
the sensitivity of the error derivative with respect to the parameter of the controller. The
adaptive gain γ, which determines the rate of adaptation of the system, was adjusted empirically,
drawing upon the expertise of the designer.
Simulations of the control system designed were satisfactory and levels of performance
of the system have been achieved. Were also performed simulations varying the adaptive gain.
It was noticed that for high values of γ the system becomes unstable.
REFERENCES
ADADE FILHO, A. Fundamentos de Robótica: Cinemática, Dinâmica e Controle de
Manipuladores Robóticos. CTA-ITA-IEMP, 1999.
ARMELLINI, F. “Controle Robusto da Antena de um Radar Meteorológico”. XVIII CBA –
Congresso Brasileiro de Automática, 2220-2227 p., 2010.
ARMELLINI, F. Projeto e Implementação do Controle de Posição de Uma Antena de Radar
Meteorológico Através de Servomecanismos. Dissertação de Mestrado – Escola Politécnica da
Universidade de São Paulo /USP, 2006.
Revista Ibero-Americana de Humanidades, Ciências e Educação. São Paulo, v.7.n.1, Jan. 2021.
ISSN - 2675 – 3375
Revista Ibero- Americana de Humanidades, Ciências e Educação- REASE
BUENO, L.P.P. Dinâmicas Emergentes na Família de Memórias Associativas Bidirecionais
Caóticas e sua Habilidade para Saltar Passos. Tese de Doutorado – USP São Carlos /
Universidade de São Paulo, 2006.
CÚNICO, M. Posicionamento Automático de Antenas Parabólicas. UnicemP / Centro
Universitário Positivo, 2006.
DIAS, S.M., Controle Adaptativo Robusto Para um Modelo Desacoplado de um Robô Móvel.
Tese de Doutorado – UFRN / Universidade Federal do Rio Grande do Norte, 2010.
DORF, R.C., Bishop, R.H. Sistemas de Controle Modernos. Ed. LTC, 2004.
Fu, K., Gonzalez, R.C., Lee C.S.G. Robotics: Control, Sensing, Vision and Intelligence. Ed.
McGraw-Hill Book Company, 1987.
GONÇALVES, J.B. “A integrated control for a biped walking robot”. RBCM. Journal of the
Brazilian Society of Mechanical Sciences and Engineering, Vol. 28, No. 4, pp. 453–460, 2006.
GUERRA VALE, M.R.B., FONSECA, D.G.V., MAITELLI, A.L., ARAÚJO, F.M.U.,
“Controle Adaptativo por Modelo de Referência Aplicado em Uma Planta de Neutralização de
PH”. INDUSCON – VIII Conferência Internacional de Aplicações Industriais, 2008.
HA, T.T. Digital Satellite Comunications, Macmillan Publishing Comp, USA, 1986.
IOANNOU, P., Sun, J. Robust Adaptive Control. Ed. Prentice Hall, 1995.
LATRE, L.G., “Modelagem e Controle de Posição de Robôs”. SAB. Revista da Sociedade 20
Brasileira de Automática, Vol. 2, No. 1, pp 3-15, 1988.
LEE, C.S.G. “Robot Arm Dynamics”, IEEE Tutorial on Robotics. Computer Society Press, 1983.
LOPES, A.M. Modelação Cinemática e Dinâmica de Manipuladores de Estrutura em Séria.
Dissertação de Mestrado – FEUP / Faculdade de Engenharia da Universidade do Porto, 2002.
MALAQUIAS, I.M. Projeto e Caracterização de um Sistema de Telemetria para Ensaios em
Vôo de Aeronaves Leves. Dissertação de Mestrado – UFMG / Universidade Federal de Minas
Gerais, 2009.
MAREELS, I.M.Y., POLDERMAN, J.W. Adaptive Systems: An Introduction. Ed. Birkhauser,
1996.
MARIANI, V.C. Maple-Fundamentos e Aplicações, Ed. LTC, 2005.
MARINS, C.N.M., Estudo Analítico e Numérico de um Enlace Digital de Comunicação via
Satélite em condição orbital Geoestacionária. Dissertação de Mestrado – Inatel / Instituto
Nacional de Telecomunicações, 2004.
MATSUMOTO, É.Y. Simulink 5: Fundamentos. Ed. Érica, 2004.
NISE, N.S. Engenharia de Sistemas de Controle. Ed. LTC, 2009.
QUEIROZ, K.I.P.M. Sistema de Controle de Apontamento para Antena da Estação TT&C de
Natal. INPE – Instituto Nacional de Pesquisas Espaciais, 2006.
Revista Ibero-Americana de Humanidades, Ciências e Educação. São Paulo, v.7.n.1, Jan. 2021.
ISSN - 2675 – 3375
Revista Ibero- Americana de Humanidades, Ciências e Educação- REASE
RESENDE, J.M.O.S.A. Estudo do Controlo Adaptativo Como Metodologia Emergente das
Técnicas de Controlo de Sistemas Dinâmicos. Dissertação de Mestrado – FEUP / Faculdade de
Engenharia da Universidade do Porto, 1995.
ROSÁRIO, J.M. Princípios de Mecatrônica. Ed. Prentice Hall do Brasil, 2005.
Santos, R.R. “Otimização do torque aplicado pelos atuadores de robôs usando técnicas de
controle ótimo”. 15º POSMEC. Simpósio do Programa de Pós-Graduação em Engenharia
Mecânica – FEME/UFU, 2005.
SCHILLING, R.J. Fundamentals of Robotics: Analysis & Control. Ed. Prentice Hall, 1990.
SLOTINE, J J., LI, W. Applied Nonlinear Control. Ed. Prentice Hall, 1991.
SOUTO, M.C.B. Desenvolvimento de uma Interface Gráfica para o Sistema de Controle da
Antena da Estação Multimissão de Natal – EMMN. INPE – Instituto Nacional de Pesquisas
Espaciais, 2009.
TAMBARA, R.V., Gründling, H.A., Della Flora, L. “Projeto de um Controlador Adaptativo
Robusto por Modelo de Referência Aplicado a uma Fonte de Potência CA”. XVIII CBA –
Congresso Brasileiro de Automática, pp. 3537-3544, 2010.
TICKOO, S., Catia. V5 R19 for Designers. Ed. CADCIM Technologies, 2009.
21
Revista Ibero-Americana de Humanidades, Ciências e Educação. São Paulo, v.7.n.1, Jan. 2021.
ISSN - 2675 – 3375