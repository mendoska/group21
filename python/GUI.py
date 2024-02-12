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


def submit():
    rangemin = rminEntry.get()
    print(rangemin)
    rangemax = rmaxEntry.get()
    print(rangemax)


rLabel = ctk.CTkLabel(root, text="Range")
rLabel.pack(pady=5)
rminEntry = ctk.CTkEntry(root, placeholder_text="Min")
rminEntry.pack(pady=10)
rmaxEntry = ctk.CTkEntry(root, placeholder_text="Max")
rmaxEntry.pack(pady=10)
rButton = ctk.CTkButton(root,text="Submit",command=submit)
rButton.pack(pady=5)


root.mainloop()