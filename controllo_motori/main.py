#!/usr/bin/env python3 
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, SpeedPercent
from ev3dev2.motor import MediumMotor, OUTPUT_D, SpeedPercent

import numpy as np  
import time as time # gestione del tempo
from time import sleep
import os 

OUTPUT_DIR = os.path.dirname(os.path.abspath(__file__))

#definizione matrice di trasformazione di Denavit-Hartenberg
def trasf_DH(d, a, alpha, theta):
  T = np.array([
          [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
          [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
          [0.0,           np.sin(alpha),               np.cos(alpha),               d],
          [0.0,           0.0,                         0.0,                         1.0]
      ])
  return T

def Jacobian(q): 
    T0 = np.eye(4)
    T01 = trasf_DH(d[0], a[0], alpha[0], q[0])
    T12 = trasf_DH(d[1], a[1], alpha[1], q[1])
    T23 = trasf_DH(d[2], a[2], alpha[2], q[2])
    T34 = trasf_DH(d[3], a[3], alpha[3], q[3])                
    T02 = np.matmul(T01,T12)
    T03 = np.matmul(T02,T23)
    T04 = np.matmul(T03,T34)

    # Estrarre gli assi z
    z0 = T0[:3, 2]
    z1 = T01[:3, 2]
    z2 = T02[:3, 2]
    z3 = T03[:3, 2]
    z4 = T04[:3, 2]

    # Estrarre le posizioni
    o0 = np.array([0, 0, 0])
    o1 = T01[:3,3]
    o2 = T02[:3,3]
    o3 = T03[:3,3]
    o4 = T04[:3,3]

    # Calcolo delle J
    J1 = np.cross(z0, o4 - o0) #prodotto vettoriale tra z0 e il vettore che va da o0 a o4
    J2 = np.cross(z1, o4 - o1)
    J3 = np.cross(z2, o4 - o2)
    J4 = np.cross(z3, o4 - o3)
    J = np.vstack((J1, J2)) #impila J1 e J2 in una riga
    J = np.vstack((J, J3)) #impila J e J3 in una riga
    J = np.vstack((J, J4)) #impila J e J4 in una riga
    J = J.T

    return J

def inverseKinematics(q, xd, xdot, k, method, ka):

    # Calcolo dello Jacobiano geometrico
    J = Jacobian(q)
    
    T0 = np.eye(4)
    T01 = trasf_DH(d[0], a[0], alpha[0], q[0])
    T12 = trasf_DH(d[1], a[1], alpha[1], q[1])
    T23 = trasf_DH(d[2], a[2], alpha[2], q[2])
    T34 = trasf_DH(d[3], a[3], alpha[3], q[3])                
    T02 = np.matmul(T01,T12)
    T03 = np.matmul(T02,T23)
    T04 = np.matmul(T03,T34)

    x = T04[:3,3] #posizione dell'organo terminale
    e = xd - x #errore tra posizione desiderata e posizione attuale
    err = np.linalg.norm(e) #norma dell'errore

    K = k * np.eye(3)  #matrice dei guadagni diagonale
    Err = np.matmul(K,e) #termine di correzione proporzionale
    J_ = np.linalg.pinv(J) #inversa dello Jacobiano

    if method == "inverse": #pseudo-inversa destra
        Qdot = np.matmul(J_, xdot + Err)

    elif method == "inverse_redundant": 
        P = np.eye(J.shape[1]) - np.matmul(J_, J) #proiettore nel nullo dello Jacobiano
        grad = np.array(dw_dq(*q), dtype=float).flatten()  # shape (4,)
        Qdot = np.matmul(J_, xdot + Err) + ka * np.matmul(P, grad)

    return Qdot, err #vettore delle velocità angolari e errore

def cartesianPlanner(pi, pf, t0, tf, t): 
    if t <= t0: #prima dell'istante iniziale, la posizione è quella iniziale e la velcotà è nulla
        pd = pi 
        pdot = np.array([0,0,0]) 
    
    elif t <= tf: 
        A = np.array([[t0**3, t0**2, t0, 1], [tf**3, tf**2, tf, 1], [3*t0**2, 2*t0, 1, 0], [3*tf**2, 2*tf, 1, 0]]) #matrice dei coefficienti del polinomio cubico

        b = np.array([[0],[np.linalg.norm(pf-pi)],[0],[0]]) #sono i vincoli 

        x = np.linalg.solve(A, b) #sono le incognite 

        s = x[0]*t**3+x[1]*t**2+x[2]*t+x[3] #calcolo ascissa curvilinea
        sdot = 3*x[0]*t**2+2*x[1]*t+x[2] #derivata dell'ascissa curvilinea
    
        pd = pi + s*(pf-pi)/np.linalg.norm(pf-pi) #posizione desiderata dell'EE
        pdot = sdot*(pf-pi)/np.linalg.norm(pf-pi) #velocità desiderata dell'EE

    elif t > tf: #dopo l'istante finale, la posizione è quella finale e la velocità è nulla
        pd = pf
        pdot = np.array([0,0,0])

    return pd, pdot

def calcolo_posa(T):
    p = T[:3,3]
    return p

def get_q():
    return np.array([
        np.radians(m1.position),
        np.radians(m2.position),
        np.radians(m3.position),
        np.radians(m4.position)
    ])

if __name__ == '__main__':

    # Motori
    m1 = LargeMotor(OUTPUT_A)
    m2 = LargeMotor(OUTPUT_B)
    m3 = LargeMotor(OUTPUT_C)
    m4 = MediumMotor(OUTPUT_D)

    m1.reset()
    m2.reset()
    m3.reset()
    m4.reset()

    m1.polarity = 'normal'
    m2.polarity = 'inversed'
    m3.polarity = 'normal'
    m4.polarity = 'inversed'

    a = np.array([0.09, 0.005, 0.035, 0.02])
    d = np.array([0.0, 0.0, 0.05, 0.0])
    alpha = np.array([np.pi/2, -np.pi/2 , np.pi/2, 0.0])
    q = get_q()

    t0 = 0  #tempo iniziale
    tm = 30  #tempo di transizione tra i due segmenti di traiettoria  
    tf = 60  #tempo finale

    # Inizializzazione SPAZIO DEI GIUNTI

    T0 = np.eye(4)
    T01 = trasf_DH(d[0], a[0], alpha[0], q[0])
    T12 = trasf_DH(d[1], a[1], alpha[1], q[1])
    T23 = trasf_DH(d[2], a[2], alpha[2], q[2])
    T34 = trasf_DH(d[3], a[3], alpha[3], q[3])                
    T02 = np.matmul(T01,T12)
    T03 = np.matmul(T02,T23)
    T04 = np.matmul(T03,T34) 

    P0 = T04[:3,3] #posizione iniziale dell'EE
    # Le lunghezze DH sono espresse in metri, quindi anche i punti della traiettoria
    # devono essere in metri. I valori originali erano in centimetri.
    P1 = np.array([0.100316, -0.019790, 0.055185])
    P2 = np.array([0.056810, -0.047541, 0.048630])

    # inizializzo i vettori
    max_iter = int((tf - t0) / 0.01) * 5  #  massimo numero di iterazioni, con un margine per sicurezza (5 volte il numero teorico)

    Qd      = np.zeros((max_iter, 4))
    Qdot    = np.zeros((max_iter, 4))
    Q_reale = np.zeros((max_iter, 4))
    Pd      = np.zeros((max_iter, 3))
    Pddot   = np.zeros((max_iter, 3))
    errore  = np.zeros(max_iter)
    T_reale = np.zeros(max_iter)

    Qd[0, :]      = q
    Qdot[0, :]    = np.zeros(4)
    Q_reale[0, :] = q
    Pd[0, :]      = P0
    errore[0]     = 0.0

    # MAIN LOOP
    ka = 0 
    k = 5 

    t_start = time.time() # tempo del sistema, inizio da qua
    t_prev = t_start # aggiorno 
    counter = 0

    while True:
        t_now = time.time() # tempo del sistema di quando ho iniziato l'interazione
        tempo = t_now - t_start # secondi trascorsi dall'inizio della simulazione
        dt = t_now - t_prev # tempo trascorso dal ciclo precedente
        t_prev = t_now

        if tempo >= tf:          # ← condizione di uscita sul tempo reale
            break

        if counter >= max_iter - 1:
            print("Raggiunto limite massimo iterazioni")
            break

        T_reale[counter] = tempo

        if tempo < tm:
            Pd[counter,:], Pddot[counter,:] = cartesianPlanner(P0, P1, t0, tm, tempo)
        else:
            Pd[counter,:], Pddot[counter,:] = cartesianPlanner(P1, P2, tm, tf, tempo)

        # Leggo gli angoli dagli encoder
        Q_reale[counter, :] = np.radians([m1.position, m2.position, m3.position, m4.position]) 

        # Inversione cinematica
        Qdot[counter, :], errore[counter] = inverseKinematics(Q_reale[counter, :], Pd[counter, :], Pddot[counter, :], k, "inverse", ka) # utilizzo le posizioni desierate dell'EE per calcolare le velocità angolari dei giunti e l'errore

        # Integrazione numerica, trovo la configurazione successiva
        Qd[counter + 1, :] = Q_reale[counter, :] + dt * Qdot[counter, :]
        q1, q2, q3, q4 = np.degrees(Qd[counter + 1, :])

        # invia i comandi ai motori per raggiungere la configurazione successiva
        m1.on_to_position(SpeedPercent(10), float(q1), brake=True, block=False)
        m2.on_to_position(SpeedPercent(10), float(q2), brake=True, block=False)
        m3.on_to_position(SpeedPercent(10), float(q3), brake=True, block=False)
        m4.on_to_position(SpeedPercent(10), float(q4), brake=True, block=False)
            
        counter += 1
        sleep(0.02)

    m1.stop()
    m2.stop()
    m3.stop()
    m4.stop()

    # taglia alla dimensione reale
    N        = counter
    t_out    = T_reale[:N]
    Qd_out   = np.degrees(Qd[1:N+1])      # target inviato al motore (gradi)
    Qr_out   = np.degrees(Q_reale[:N])    # posizione letta prima del movimento (gradi)
    err_out  = errore[:N]

    # file confronto giunti: una riga per iterazione
    header = "tempo_assoluto;tempo;q1_des;q1_reale;q2_des;q2_reale;q3_des;q3_reale;q4_des;q4_reale;errore"
    rows = np.column_stack([
        np.full(N, time.time()),
        t_out,
        Qd_out[:, 0], Qr_out[:, 0],   # giunto 1
        Qd_out[:, 1], Qr_out[:, 1],   # giunto 2
        Qd_out[:, 2], Qr_out[:, 2],   # giunto 3
        Qd_out[:, 3], Qr_out[:, 3],   # giunto 4
        err_out
    ])


    np.savetxt(os.path.join(OUTPUT_DIR, "confronto_giunti3.txt"), rows, delimiter=";", header=header, fmt="%.4f", comments="")
        
    sleep(2)
    
    #riporto i motori alla configurazione iniziale
    m1.on_to_position(SpeedPercent(10), 0, brake=True, block=False)
    m2.on_to_position(SpeedPercent(10), 0, brake=True, block=False)
    m3.on_to_position(SpeedPercent(10), 0, brake=True, block=False)
    m4.on_to_position(SpeedPercent(10), 0, brake=True, block=True)

    m1.off()
    m2.off()
    m3.off()
    m4.off()
