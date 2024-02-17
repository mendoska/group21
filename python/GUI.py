from algorithms.dqn_agent import runDQN
from algorithms.geneticAlgorithmTest import runGA
from algorithms.munkres_algorithm import runMunkres
from algorithms.simulated_annealing import runSimulatedAnnealing
from tkinter import *
import customtkinter as ctk
import tkinter as tk

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
    algorithm = algoDropdown.get()
    print("Selected Algorithm:", algorithm)
    num_threats = threatScale.get()
    print("Number of Threats:", num_threats)

def updateThreatLabel(value):
    currentThreatLabel.configure(text=f"Current Number of Threats: {int(value)}")

rLabel = ctk.CTkLabel(root, text="Range")
rLabel.pack(pady=5)
rminEntry = ctk.CTkEntry(root, placeholder_text="Min")
rminEntry.pack(pady=10)
rmaxEntry = ctk.CTkEntry(root, placeholder_text="Max")
rmaxEntry.pack(pady=10)

algoLabel = ctk.CTkLabel(root, text="Select Algorithm")
algoLabel.pack(pady=5)
algoOptions = ["DQN", "Genetic Algorithm", "Munkres", "Simulated Annealing"]
algoDropdown = ctk.CTkOptionMenu(root, values=algoOptions)
algoDropdown.pack(pady=10)

threatVar = tk.IntVar(value=1)
threatScale = ctk.CTkSlider(root, from_=1, to=50, variable=threatVar, command=updateThreatLabel)
threatScale.pack(pady=10)
currentThreatLabel = ctk.CTkLabel(root, text=f"Current Number of Threats: {threatVar.get()}")
currentThreatLabel.pack(pady=5)

submitButton = ctk.CTkButton(root, text="Start", command=submit)
submitButton.pack(pady=5)

root.mainloop()