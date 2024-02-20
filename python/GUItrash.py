from algorithms.dqn_agent import runDQN
from algorithms.geneticAlgorithmTest import runGA
from algorithms.munkres_algorithm import runMunkres
from algorithms.simulated_annealing import runSimulatedAnnealing
from tkinter import *
import customtkinter as ctk
from PIL import Image

ctk.set_appearance_mode("dark")  # Modes: system (default), light, dark
ctk.set_default_color_theme("blue")  # Themes: blue (default), dark-blue, green

root = ctk.CTk()
root.title('B.O.W.S.E.R.')
root.geometry('1000x600')

def gaWin():
   ga = ctk.CTkToplevel(root)
   ga.title("Genetic Algorithm")
   ga.geometry("1000x600")

def saWin():
   sa = ctk.CTkToplevel(root)
   sa.title("Simulated Annealing")
   sa.geometry("1000x600")

def maWin():
   ma = ctk.CTkToplevel(root)
   ma.title("Munkres Algorithm")
   ma.geometry("1000x600")

def dtrainWin():
   dtrain = ctk.CTkToplevel(root)
   dtrain.title("Deep Q-Learning (Train)")
   dtrain.geometry("1000x600")

def dtestWin():
   dtest = ctk.CTkToplevel(root)
   dtest.title("Deep Q-Learning (Test)")
   dtest.geometry("1000x600")


logo = ctk.CTkImage(light_image=Image.open('BOWSER LOGO FINAL.png'),size=(150,150))
logoLabel = ctk.CTkLabel(root, text="", image = logo).place(x=10,y=10)

title = ctk.CTkLabel(root,text="B.O.W.S.E.R.",font=("Helvetica",50))
title.pack(pady=40)

gaBut = ctk.CTkButton(root,text="Genetic Algorithm",command=gaWin)
gaBut.pack(pady=20)

saBut = ctk.CTkButton(root,text="Simulated Annealing",command=saWin)
saBut.pack(pady=20)

maBut = ctk.CTkButton(root,text="Munkres Algorithm",command=maWin)
maBut.pack(pady=20)

dtrainBut = ctk.CTkButton(root,text="Deep Q-Learning (Train)",command=dtrainWin)
dtrainBut.pack(pady=20)

dtestBut = ctk.CTkButton(root,text="Deep Q-Learning (Test)",command=dtestWin)
dtestBut.pack(pady=20)

root.mainloop()