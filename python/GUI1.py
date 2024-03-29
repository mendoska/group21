from algorithms.dqn_agent import runDQN
from algorithms.geneticAlgorithmTest import runGA
from algorithms.munkres_algorithm import runMunkres
from algorithms.simulated_annealing import runSimulatedAnnealing
from tkinter import *
from PIL import Image
import customtkinter as ctk
import tkinter as tk
import pandas as pd
import random 
import time
import csv
#from app import run_BOWSER_simulation

masterTemplate =  "dataFiles/threat_location_original.csv"
threatFileLocation = "dataFiles/threat_location.csv"
weaponFileLocation = "dataFiles/weapon_data.csv"
dqnModelPath = "dataFiles/trained_model.zip"

ctk.set_appearance_mode("dark")  # Modes: system (default), light, dark
ctk.set_default_color_theme("blue")  # Themes: blue (default), dark-blue, green

root = ctk.CTk()
root.title('B.O.W.S.E.R.')
root.geometry('1000x600')

# Read the number of inputs 
def count_threats(threat_file):
    with open(threat_file,'r') as file:
        reader = csv.reader(file)
        threats = list(reader)
        return len(threats)

def submit():   
    outputLabel.configure(text="")
    try:
        rangemin = int(rminEntry.get())
        rangemax = int(rmaxEntry.get())
        if rangemin <= 0 or rangemax <= 0:
            print("Invalid Range Provided")
            return
    except ValueError:
        print("Invalid Range Provided")
        return
    algorithm = algoDropdown.get()
    num_threats = int(threatScale.get())
    
    #simulation_leaker_percent, algorithm_leaker_percentage = run_BOWSER_simulation(spawnRange=(rangemin, rangemax), algorithmChoice=algorithm, numberOfDrones=num_threats)
    
    
    #return # temp so that the entire thing doesnt explode
    
        
    columns = ['name', 'x', 'y', 'z', 'min_range', 'speed', 'type']
    df = pd.read_csv(masterTemplate, header=None, names=columns)
    limited_df = df.sample(num_threats)
    limited_df['min_range'] = [random.choice(range(rangemin, rangemax+1, 10)) for _ in range(num_threats)]
    limited_df.to_csv(threatFileLocation, index=False, header=False)

    time.sleep(3)
    if algorithm == "DQN":
         current_num_threats = count_threats(threatFileLocation)
         runDQN(savePath=dqnModelPath, train=True, num_threats=current_num_threats)
         time.sleep(3)
         response, leaker_percentage = runDQN(loadPath=dqnModelPath, train=False,threatFilePath=threatFileLocation)
         outputLabel.configure(text=f"Leaker Percentage: {leaker_percentage:.2f}%")
         dleaker.configure(text=f"Deep Q-Learning: \n{leaker_percentage:.2f}%")
         #printOutputList(outputList=response)

    elif algorithm == "Genetic Algorithm":
         leaker_percentage = runGA(threatFileLocation=threatFileLocation)
         leaker_percentage = (1.00 - leaker_percentage) * 100
         outputLabel.configure(text=f"Leaker Percentage: {leaker_percentage}%")
         gleaker.configure(text=f"Genetic Algorithm: \n{leaker_percentage:.2f}%")

    elif algorithm == "Munkres":
         leaker_percentage = runMunkres(threatFileLocation=threatFileLocation, weaponFileLocation=weaponFileLocation)
         outputLabel.configure(text=f"Leaker Percentage: {leaker_percentage}%")
         mleaker.configure(text=f"Munkres Algorithm: \n{leaker_percentage:.2f}%")
         #printOutputList(outputList=response)

    elif algorithm == "Simulated Annealing":
         leaker_percentage = runSimulatedAnnealing()
         leaker_percentage = leaker_percentage * 100
         outputLabel.configure(text=f"Leaker Percentage: {leaker_percentage}%")
         sleaker.configure(text=f"Simulated Annealing: \n{leaker_percentage:.2f}%")
         #printOutputList(outputList=response)

    rwin = ctk.CTk()
    rwin.title('Your Result')
    rwin.geometry('700x500')

    num_leakers = int(num_threats * leaker_percentage * 0.01)

    rTitle = ctk.CTkLabel(rwin,text=algorithm,font=("Rockwell Extra Bold",50))
    rTitle.pack(pady=40)

    rTheatsLabel = ctk.CTkLabel(rwin,text="Threats",font=("Helvetica",20))
    rTheatsLabel.pack(pady=5)
    rTheats = ctk.CTkLabel(rwin,text=num_threats,font=("Helvetica",15), bg_color= "red")
    rTheats.pack(pady=20)

    rleakersLabel = ctk.CTkLabel(rwin,text="Leakers",font=("Helvetica",20))
    rleakersLabel.pack(pady=5)
    rleakersLabel = ctk.CTkLabel(rwin,text=num_leakers,font=("Helvetica",15), bg_color= "red")
    rleakersLabel.pack(pady=20)

    rleakerPercentageLabel = ctk.CTkLabel(rwin,text="Leaker percentage",font=("Helvetica",20))
    rleakerPercentageLabel.pack(pady=5)
    rleakerPercentageLabel = ctk.CTkLabel(rwin,text=f"{leaker_percentage}%",font=("Helvetica",15), bg_color= "red")
    rleakerPercentageLabel.pack(pady=20)

    rwin.mainloop()

#def printOutputList(outputList: list):
#    text = tk.Text(master=window, height=100, width=100)
#    text.grid(column=10, row=10)
#    for entry in outputList:
#        print(entry)
#        text.insert(tk.END, str(entry) + '\n')

def updateThreatLabel(value):
    currentThreatLabel.configure(text=f"Current Number of Threats: {int(value)}")

logo = ctk.CTkImage(light_image=Image.open('BOWSER LOGO FINAL.png'),size=(150,150))
logoLabel = ctk.CTkLabel(root, text="", image = logo).place(x=10,y=10)

title = ctk.CTkLabel(root,text="B.O.W.S.E.R.",font=("Rockwell Extra Bold",50))
title.pack(pady=40)

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
# My computer can only handle 10 threats before dropping below 50% of real time
threatScale = ctk.CTkSlider(root, from_=1, to=10, variable=threatVar, command=updateThreatLabel)
threatScale.pack(pady=10)
currentThreatLabel = ctk.CTkLabel(root, text=f"Current Number of Threats: {threatVar.get()}")
currentThreatLabel.pack(pady=5)

submitButton = ctk.CTkButton(root, text="Start", command=submit)
submitButton.pack(pady=5)

outputLabel = ctk.CTkLabel(root, text="", wraplength=800)
outputLabel.pack(side="bottom", pady=10)

dleaker = ctk.CTkLabel(root, text="Deep Q-Learning: not run yet", wraplength=200)
dleaker.pack(side="right")
dleaker.place(x=800,y=150)

gleaker = ctk.CTkLabel(root, text="Genetic Algorithm: not run yet", wraplength=200)
gleaker.pack(side="right")
gleaker.place(x=800,y=190)

mleaker = ctk.CTkLabel(root, text="Munkres Algorithm: not run yet", wraplength=200)
mleaker.pack(side="right")
mleaker.place(x=800,y=230)

sleaker = ctk.CTkLabel(root, text="Simulated Annealing: not run yet", wraplength=200)
sleaker.pack(side="right")
sleaker.place(x=800,y=270)

root.mainloop()