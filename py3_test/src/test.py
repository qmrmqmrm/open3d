import numpy as np

a = np.array(
    [[-0.5010704, -0.44409148, -0.74277265, 4.54253237],
     [-0.71916404, 0.6911058, 0.07194343, 1.53114217],
     [0.48138503, 0.5702241, - 0.66566728, 1.97647568],
     [0.0, 0.0,
      0.0, 1.0]])

b = np.array(
    [[-0.01362947, -0.32115327, 0.94692915, 2.0],
     [-0.8272164, -0.52838459, -0.1911093, 3.0],
     [0.56171814, -0.78592004, -0.25846165, 4.0],
     [0.0, 0.0, 0.0, 1.0]])

ss = np.array(
    [[-0.44523699, - 0.55536911, 0.70237396, 2.84662465],
     [-0.36490212, 0.82886115, 0.42407032, 0.45962802],
     [-0.81768604, - 0.06748596, - 0.57169501, 5.29786157],
     [0., 0., 0., 1.]])
c =  np.linalg.inv(ss)
print(c)