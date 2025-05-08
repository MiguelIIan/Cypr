import matplotlib.pyplot as plt


labels = 'B', 'A', 'B', 'C', 'C'
sizes = [100/6, 100/6, 100/6, 25, 25]
colors=['blue', 'green', 'blue', 'red', 'red']
explode = (0.05, 0.1, 0.05, 0.05, 0.05) 

fig, ax = plt.subplots()
ax.pie(sizes, explode=explode, labels=labels, colors=colors, startangle=0)

ax.annotate('90', xy=(1.2, 0))

plt.show()