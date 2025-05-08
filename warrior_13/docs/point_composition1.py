import matplotlib.pyplot as plt
import numpy as np

# Definimos los vectores B y C
B = np.array([2, 1])
C = np.array([1, 3])

# Calculamos el vector A como la suma de B y C
A = B + C

# Configuramos la figura
fig, ax = plt.subplots()

# Dibujamos los vectores usando quiver
ax.quiver(0, 0, A[0], A[1], angles='xy', scale_units='xy', scale=1, color='red', label='$P$')
ax.quiver(0, 0, B[0], B[1], angles='xy', scale_units='xy', scale=1, color='blue', label='$P_{robot}$')
ax.quiver(B[0], B[1], C[0], C[1], angles='xy', scale_units='xy', scale=1, color='green', label='$P_{relativa}$')

# ANOTACIONES
ax.annotate('(0,0)', xy=(-0.30,-0.30))


# Añadimos configuraciones adicionales
ax.set_xlim(-1, 5)
ax.set_ylim(-1, 5)
ax.set_aspect('equal', adjustable='box')
plt.legend()
plt.title("$P = P_{robot}*P_{relativa}$")


# Quitar ejes y bordes
ax.set_xticks([])
ax.set_yticks([])
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)
ax.spines['left'].set_visible(False)
ax.spines['bottom'].set_visible(False)


plt.show()
