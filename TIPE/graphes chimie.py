#Importation des modules utiles

import matplotlib.pyplot as plt
import numpy as np
from numpy import random

#Concentration en hydroxyde de sodium, en mol/L

C=0.1 

#Graduation en absisses

J=np.array([3,7,17,126])

#Volumes équivalents en mL

Veq1=np.array([26.7,16.9,9.4,45.3])
Veq2=np.array([50.4,47.4,41.7,90.2])

#Calcul des quantités de matière d'éthanol

n=np.array(C*(Veq2-Veq1)*10**-3)

#Simulation Monte Carlo

u_Veq=0.05 #en mL
N=1000
Veq1_mc=random.uniform(Veq1-u_Veq,Veq1+u_Veq,(N,len(n))) #Simulation de N valeurs pour chacune des n valeurs de Veq1
Veq2_mc=random.uniform(Veq2-u_Veq,Veq2+u_Veq,(N,len(n))) #Simulation de N valeurs pour chacune des n valeurs de Veq2
n_mc=C*((Veq2_mc-Veq1_mc)*10**-3)

u_n=np.empty(len(n))
for k in np.arange(len(n)):
    u_n[k]=np.std(n_mc[:,k],ddof=1)

u_n_moy=np.mean(u_n)

#Tracé des graphes

plt.scatter(J,n,marker='+')
plt.title("Quantité d'éthanol dans le mélange")
plt.xlabel("Nombre de jours écoulés")
plt.ylabel("Quantité d'éthanol formée en mol")
plt.errorbar(J,n,xerr=0,yerr=u_n_moy,fmt='.')
plt.show()

#Régression linéaire : tracé de n=f(J)

a=np.zeros(N)
b=np.zeros(N)
for j in range(N):
    a[j],b[j]=np.polyfit(J,n_mc[j,:],1)
A=np.mean(a)
B=np.mean(b)
plt.scatter(J,n,marker='+',color='orange')
plt.plot(J,A*J+B,color='orange')
plt.title("Régression linéaire : tracé de n=f(J)")
plt.xlabel("Nombre de jours écoulés")
plt.ylabel("Quantité d'éthanol dans le mélange")
print("Le coefficient de corrélation pour la régression linéaire n=f(J) est {:.2f} ".format(np.corrcoef(J,n)[0][1]))
plt.show()

#Régression linéaire : tracé de ln(n)=f(J)

a=np.zeros(N)
b=np.zeros(N)
c=np.log(n_mc)
for j in range(N):
    a[j],b[j]=np.polyfit(J,c[j,:],1)
A=np.mean(a)
B=np.mean(b)
plt.scatter(J,np.log(n),marker='+',color='green')
plt.plot(J,A*J+B,color='green')
plt.title("Régression linéaire : tracé de ln(n)=f(J)")
plt.xlabel("Nombre de jours écoulés")
plt.ylabel("ln(Quantité d'éthanol dans le mélange)")
print("Le coefficient de corrélation pour la régression linéaire ln(n)=f(J) est {:.2f} ".format(np.corrcoef(J,np.log(n))[0][1]))
plt.show()

#Régression linéaire : tracé de 1/n=f(J)

a=np.zeros(N)
b=np.zeros(N)
c=1/(n_mc)
for j in range(N):
    a[j],b[j]=np.polyfit(J,c[j,:],1)
A=np.mean(a)
B=np.mean(b)
plt.scatter(J,1/(n),marker='+',color='purple')
plt.plot(J,A*J+B,color='purple')
plt.title("Régression linéaire : tracé de 1/n=f(J)")
plt.xlabel("Nombre de jours écoulés")
plt.ylabel("1/(Quantité d'éthanol dans le mélange)")
print("Le coefficient de corrélation pour la régression linéaire 1/n=f(J) est {:.2f} ".format(np.corrcoef(J,1/n)[0][1]))
plt.show()