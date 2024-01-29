from algorithms.dqn_agent import runDQN
from algorithms.geneticAlgorithmTest import runGA
from algorithms.munkres_algorithm import runMunkres
from algorithms.simulated_annealing import runSimulatedAnnealing
from tkinter import *
import customtkinter as ctk


ctk.set_appearance_mode("dark")  # Modes: system (default), light, dark
ctk.set_default_color_theme("blue")  # Themes: blue (default), dark-blue, green

root = ctk.CTk()
root.title('B.O.W.S.E.R.')
root.geometry('1000x600')

ga = ctk.CTkToplevel(root)
ga.title("Genetic Algorithm")
ga.geometry("1000x600")

sa = ctk.CTkToplevel(root)
sa.title("Simulated Annealing")
sa.geometry("1000x600")

ma = ctk.CTkToplevel(root)
ma.title("Munkres Algorithm")
ma.geometry("1000x600")

dtrain = ctk.CTkToplevel(root)
dtrain.title("Deep Q-Learning (Train)")
dtrain.geometry("1000x600")

dtest = ctk.CTkToplevel(root)
dtest.title("Deep Q-Learning (Test)")
dtest.geometry("1000x600")




root.mainloop()