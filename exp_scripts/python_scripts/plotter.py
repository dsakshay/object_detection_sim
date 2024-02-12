import matplotlib.pyplot as plt

x=[1,2,3]
y=[9,8,7]

plt.plot(x,y)
for a,b in zip(x, y): 
    plt.text(a, b, str(b))
plt.show()
