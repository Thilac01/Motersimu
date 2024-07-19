import streamlit as st
import numpy as np
import matplotlib.pyplot as plt

st.title("DC Motor Simulation")

st.sidebar.header("Input Parameters")

La = st.sidebar.slider("Armature inductance (H)", min_value=0.01, max_value=0.1, value=0.04, step=0.01, key='La')
Ra = st.sidebar.slider("Armature resistance (Ohms)", min_value=1.0, max_value=100.0, value=20.0, step=1.0, key='Ra')
KE = st.sidebar.slider("Back emf constant (V/(rad/sec))", min_value=0.1, max_value=1.0, value=0.5, step=0.1, key='KE')
KT = st.sidebar.slider("Torque constant (Nm/A)", min_value=0.1, max_value=1.0, value=0.5, step=0.1, key='KT')
J = st.sidebar.slider("Moment of Inertia (kg.m^2)", min_value=0.000001, max_value=0.01, value=0.001, step=0.000001, key='J')
D = st.sidebar.slider("Frictional coefficient (Nm/(rad/sec))", min_value=0.1, max_value=1.0, value=0.2, step=0.1, key='D')
dt = st.sidebar.slider("Time step (s)", min_value=0.0001, max_value=0.01, value=0.0001, step=0.0001, key='dt')
Wref = st.sidebar.slider("Reference speed (rad/sec)", min_value=0.1, max_value=10.0, value=2.0, step=0.1, key='Wref')
TL = st.sidebar.slider("Load torque (Nm)", min_value=0.01, max_value=1.0, value=0.05, step=0.01, key='TL')
N = st.sidebar.slider("Number of iterations", min_value=100, max_value=1000, value=500, step=1, key='N')
Vmax = st.sidebar.slider("Max terminal voltage (V)", min_value=1.0, max_value=100.0, value=24.0, step=1.0, key='Vmax')
Vmin = st.sidebar.slider("Min terminal voltage (V)", min_value=0.0, max_value=10.0, value=0.0, step=0.1, key='Vmin')

def run_simulation(La, Ra, KE, KT, J, D, dt, Wref, TL, N, Vmax, Vmin):
    A = np.array([[1, 0, dt, 0],
                  [0, 1, 0, dt],
                  [-Ra/La, -KE/La, 0, 0],
                  [KT/J, -D/J, 0, 0]])
    B = np.array([0, 0, 1/La, 0])
    C = np.array([0, 0, 0, -1/J])
    X = np.array([0, 0, 0, 0])
    
    Iref = (TL + D * Wref) / KT
    V = Iref * Ra + KE * Wref

    if V > Vmax:
        V = Vmax
    elif V < Vmin:
        V = Vmin

    Ia = np.zeros(N)
    w = np.zeros(N)

    for n in range(N):
        Ia[n] = X[0]
        w[n] = X[1]
        X = np.dot(A, X) + B * V + C * TL

    return w, Ia

w, Ia = run_simulation(La, Ra, KE, KT, J, D, dt, Wref, TL, N, Vmax, Vmin)

# Plotting speed
st.subheader("Motor Speed (rad/sec)")
fig_w, ax_w = plt.subplots()
ax_w.plot(w, linewidth=3)
ax_w.set_xlabel("Iteration")
ax_w.set_ylabel("Speed (rad/sec)")
ax_w.grid(True)
st.pyplot(fig_w)

# Plotting current
st.subheader("Armature Current (A)")
fig_I, ax_I = plt.subplots()
ax_I.plot(Ia, linewidth=3)
ax_I.set_xlabel("Iteration")
ax_I.set_ylabel("Current (A)")
ax_I.grid(True)
st.pyplot(fig_I)
