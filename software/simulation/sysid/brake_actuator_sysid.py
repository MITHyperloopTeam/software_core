import numpy as np
from scipy.optimize import minimize

input_file = "/home/gizatt/mit-hyperloop-es/logs/20161014/velocity_steps_extract.csv"

dataset_raw = np.loadtxt(input_file, skiprows=1, delimiter=",")
# indexes into that
j_recv_time = 0
j_utime = 1
j_dist = 2 
j_vel = 3
j_pressure = 4
j_m = 5
j_po = 6
j_pv = 7

i_p_res = 0 # reservoir pressure
i_A_cyl = 1 # cylinder area
i_K_spring = 2 # spring constant
i_B_cyl = 3 # cylinder movement constant
i_C_mtr = 4 # motor pressure increase rate
i_K_pv = 5 
i_K_po = 6 

x0 = np.array([
  80, # reservoir pressure
  100, # cylinder area
  1000, # spring constant
  1, # cylinder movement constant
  100, # motor pressure increase rate
  100, 
  10000
  ])

def brake_actuator_fitting_function(x):
  """Using x as brake system parameters, calculates error over brake testing dataset"""
  err = 0
  for i in range(0, dataset_raw.shape[0] - 1):
    # predict next point from current one
    dt = (dataset_raw[i+1, j_utime] - dataset_raw[i, j_utime]) / 1000.0 / 1000.0
    d_vel = x[i_A_cyl]*dataset_raw[i, j_pressure] - x[i_K_spring]
    pred_vel = dataset_raw[i, j_vel] + dt * d_vel

    d_pos = dt * d_vel + dataset_raw[i, j_vel]
    pred_pos = dataset_raw[i, j_dist] + dt * d_pos

    d_pcyl = -x[i_B_cyl]*dataset_raw[i, j_vel] + \
             x[i_C_mtr]*dataset_raw[i, j_m] + \
             -x[i_K_pv]*np.sqrt(dataset_raw[i, j_pv] * np.abs(dataset_raw[i, j_pressure] - x[i_p_res])) * np.sign(dataset_raw[i, j_pressure] - x[i_p_res]) + \
             -x[i_K_po]*np.sqrt(dataset_raw[i, j_po] * np.abs(dataset_raw[i, j_pressure] - x[i_p_res])) * np.sign(dataset_raw[i, j_pressure] - x[i_p_res])
    pred_pcyl = dataset_raw[i, j_pressure] + dt*d_pcyl

    err += (pred_vel - dataset_raw[i+1, j_vel])**2 + (pred_pos - dataset_raw[i+1, j_dist])**2 + (pred_pcyl - dataset_raw[i+1, j_pressure])**2
  print x, ", err ", err
  return err

res = minimize(brake_actuator_fitting_function, x0, method='SLSQP',
               options={'disp': True})

print "final: ", res  