#Importation des modules utiles

import numpy as np
import matplotlib.pyplot as plt

#Hauteurs

H=np.arange(10,45,5)

#Valeurs 

d=289e-12 #en C/N ou m/V
epsilonT=1.15e-8 #en F/m
epsilon0=8.85e-12 #en F/m
sE=15.5e-12 #en m²/N
r=2.5e-2 #en m
A=np.pi*r**2
e=3.5e-3 #en m
C=epsilonT*A/e
k=(epsilonT/(sE*epsilonT-d**2))*(A/e) #en N/m
alpha=d/(sE*epsilonT-d**2)#en V/m
C2=0.1e-6 #en F
beta=C2/C

#Coefficient formule piezoélectricité

B=(alpha*C*(1+alpha*d)-d*k)/(1+(sE*e/A)*(C*alpha**2-k))

#Mesures à vide --------------------------------------------------------------------
U11=np.array([2.3250,2.3438,2.4438,2.3750,2.4188,2.4625]) #6 mesures pour H=10cm
U12=np.array([3.0375,2.9813,3.1250,3.1625,3.0688,3.0438]) #6 mesures pour H=15cm
U13=np.array([3.4750,3.6000,3.5125,3.5875,3.6375,3.6625]) #6 mesures pour H=20cm
U14=np.array([4.2125,4.4000,4.2750,4.2000,4.2250,4.3625]) #6 mesures pour H=25cm
U15=np.array([4.8750,4.7375,4.9000,4.9125,4.7500,4.7625]) #6 mesures pour H=30cm
U16=np.array([5.2375,5.3375,5.3250,5.3000,5.1875,5.4375]) #6 mesures pour H=35cm
U17=np.array([5.8250,5.7000,5.8625,5.8750,6.0000,5.8000]) #6 mesures pour H=40cm

#Tension en entrée à vide
U1=beta*np.array([U11,U12,U13,U14,U15,U16,U17]) #tableau 7 lignes, 6 colonnes

#Mesures pour l'eau ----------------------------------------------------------------
U21=np.array([2.0687,2.1000,2.0438,2.0437,2.0563,2.0313]) #6 mesures pour H=10cm
U22=np.array([2.7875,2.6563,2.7625,2.9063,2.7563,2.8938]) #6 mesures pour H=15cm
U23=np.array([3.3313,3.4000,3.5313,3.5625,3.4188,3.2438]) #6 mesures pour H=20cm
U24=np.array([4.1000,4.0875,4.0000,3.9000,3.9375,3.9250]) #6 mesures pour H=25cm
U25=np.array([4.6375,4.8250,4.7125,4.8375,4.6750,4.6000]) #6 mesures pour H=30cm
U26=np.array([5.1625,5.1500,5.1625,5.1250,5.1125,5.2750]) #6 mesures pour H=35cm
U27=np.array([5.7125,6.0125,6.0250,6.1000,5.8875,6.0500]) #6 mesures pour H=40cm

#Tension en entrée pour l'eau
U2=beta*np.array([U21,U22,U23,U24,U25,U26,U27]) #tableau 7 lignes, 6 colonnes

#Mesures pour le matériau non newtonien ---------------------------------------------
U31=np.array([292.5e-3,432.5e-3,325.0e-3,350.0e-3,425.0e-3,347.5e-3]) #6 mesures pour H=10cm
U32=np.array([615.0e-3,512.5e-3,634.0e-3,676.5e-3,502.0e-3,542.0e-3]) #6 mesures pour H=15cm
U33=np.array([814.0e-3,932.5e-3,891.5e-3,923.0e-3,826.5e-3,876.5e-3])   #6 mesures pour H=20cm
U34=np.array([1.0375,1.1500,1.0563,1.2355,1.1813,996.5e-3])   #6 mesures pour H=25cm
U35=np.array([1.2750,1.4875,1.4375,1.3500,1.4255,1.3875])   #6 mesures pour H=30cm
U36=np.array([1.5325,1.4775,1.5245,1.6665,1.5985,1.5355])   #6 mesures pour H=35cm
U37=np.array([1.6525,1.8995,1.9875,1.7125,1.8050,1.8750])   #6 mesures pour H=40cm

#Tension en entrée pour le matériau non newtonien
U3=beta*np.array([U31,U32,U33,U34,U35,U36,U37]) #tableau 7 lignes, 6 colonnes

#Forces exercées sur le capteur -----------------------------------------------------

#À vide
F1=np.zeros(7)
for i in range(7):
    F1[i]=B*np.mean(U1[i,:])
    
#Eau
F2=np.zeros(7) 
for i in range(7):
    F2[i]=B*np.mean(U2[i,:])

#Matériau non newtonien
F3=np.zeros(7)
for i in range(7):
    F3[i]=B*np.mean(U3[i,:])
    
#Incertides sur les valeurs de forces ----------------------------------------------- 

#À vide
I1=np.zeros(7)
for i in range(7):
    I1[i]=B*(np.std(U1[i,:],ddof=1)/np.sqrt(6))
#Eau
I2=np.zeros(7)
for i in range(7):
    I2[i]=B*(np.std(U2[i,:],ddof=1)/np.sqrt(6))

#Matériau non newtonien
I3=np.zeros(7)
for i in range(7):
    I3[i]=B*(np.std(U3[i,:],ddof=1)/np.sqrt(6))
    
#Tracé des graphes F=f(H) + barres d'erreurs -----------------------------------------   

#À vide
plt.scatter(H,F1,marker="+",label="à vide",color="green")
for i in range(7):
    plt.errorbar(H[i],F1[i],xerr=0.5/np.sqrt(3),yerr=I1[i],fmt='.',color="green") #demi étendue pour les mesures de hauteurs=0.5mm
    
#Eau
plt.scatter(H,F2,marker="+",label="eau",color="blue")
for i in range(7):
    plt.errorbar(H[i],F2[i],xerr=0.5/np.sqrt(3),yerr=I2[i],fmt='.',color="blue") #demi étendue pour les mesures de hauteurs=0.5mm

#Matériau non newtonien
plt.scatter(H,F3,marker="+",label="matériau non newtonien",color="orange")
for i in range(7):
    plt.errorbar(H[i],F3[i],xerr=0.5/np.sqrt(3),yerr=I3[i],fmt='.',color="orange") #demi étendue pour les mesures de hauteurs=0.5mm

plt.title("Force exercée sur le capteur en fonction de la hauteur de la chute")
plt.xlabel("Hauteur de la chute (en cm)")
plt.ylabel("Force exercée sur le capteur (en N)")
plt.legend()
plt.show()