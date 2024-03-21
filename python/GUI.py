from algorithms.dqn_agent import runDQN
# from algorithms.dqn_agent1 import runDQN
from algorithms.geneticAlgorithmTest import runGA
from algorithms.munkres_algorithm import runMunkres
from algorithms.simulated_annealing import runSimulatedAnnealing
from tkinter import *
from PIL import Image, ImageTk
import customtkinter as ctk
import tkinter as tk
import pandas as pd
import random 
import time
import csv
try:
    from app import run_BOWSER_simulation
    RUN_SIMULATION = False
except:
    RUN_SIMULATION = False


masterTemplate =  "dataFiles/threat_location_original.csv"
threatFileLocation = "dataFiles/threat_location.csv"
weaponFileLocation = "dataFiles/weapon_data.csv"
dqnModelPath = "dataFiles/trained_model.zip"
historyFile = "dataFiles/history.csv"

ctk.set_appearance_mode("dark")  # Modes: system (default), light, dark
ctk.set_default_color_theme("blue")  # Themes: blue (default), dark-blue, green

root = ctk.CTk()
root.title('B.O.W.S.E.R.')
root.geometry('1000x600')

def write_history(history_file,list):
    fieldnames = ['algorithm_name', 'leaker_percentage']
    new_list = {'algorithm_name': list[0], 'leaker_percentage': list[1]}
    with open(history_file,'a',newline='') as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writerow(new_list)
        file.close()

# sort by leaker percentage
def read_history(history_file):
    sortedList = pd.read_csv(history_file)
    sortedList.sort_values(sortedList.columns[1],axis=0,inplace=True)
    return sortedList.to_string(index=False)

# Read the number of inputs 
def count_threats(threat_file):
    with open(threat_file, 'r') as file:
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
    unit = unitDropdown.get()
    if unit =="km":
        rangemin *= 1000
        rangemax *= 1000
    elif unit == "ft":
        rangemin *= 0.3048
        rangemax *= 0.3048
    elif unit == "mi":
        rangemin *= 1852
        rangemax *= 1852
    else:
        pass
    algorithm = algoDropdown.get()
    num_threats = int(threatScale.get())
    
    if RUN_SIMULATION:
        simulation_leaker_percentage, algorithm_leaker_percentage = run_BOWSER_simulation(spawnRange=(rangemin, rangemax), algorithmChoice=algorithm, numberOfDrones=num_threats)
    else:
        simulation_leaker_percentage = 10.0
        algorithm_leaker_percentage = 20.0
    
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
        response, leaker_percentage = runDQN(loadPath=dqnModelPath, train=False, threatFilePath=threatFileLocation)
        outputLabel.configure(text=f"Leaker Percentage: {leaker_percentage:.2f}%")
        #dleaker.configure(text=f"Deep Q-Learning: \n{leaker_percentage:.2f}%")
        print(response)
        

    elif algorithm == "Genetic Algorithm":
        response, leaker_percentage = runGA(threatFileLocation=threatFileLocation)
        leaker_percentage = (1.00 - leaker_percentage) * 100
        outputLabel.configure(text=f"Leaker Percentage: {leaker_percentage}%")
        #gleaker.configure(text=f"Genetic Algorithm: \n{leaker_percentage:.2f}%")

    elif algorithm == "Munkres":
        response,leaker_percentage = runMunkres(threatFileLocation=threatFileLocation, weaponFileLocation=weaponFileLocation)
        outputLabel.configure(text=f"Leaker Percentage: {leaker_percentage}%")
        #mleaker.configure(text=f"Munkres Algorithm: \n{leaker_percentage:.2f}%")
        print(response)

    elif algorithm == "Simulated Annealing":
        response,leaker_percentage = runSimulatedAnnealing()
        leaker_percentage = leaker_percentage * 100
        outputLabel.configure(text=f"Leaker Percentage: {leaker_percentage}%")
        #sleaker.configure(text=f"Simulated Annealing: \n{leaker_percentage:.2f}%")
        print(response)
    
    save_list = [algorithm,leaker_percentage]
    write_history(historyFile,save_list)
    savedList = read_history(historyFile)
    
    leaderBoard.configure(text="\n".join(savedList.split('\n')[:10]))

    rwin = ctk.CTk()
    rwin.title('Your Result')
    rwin.geometry('700x500')

    algorithm_num_leakers = int(num_threats * algorithm_leaker_percentage * 0.01)
    simulation_num_leakers = int(num_threats * simulation_leaker_percentage * 0.01)
    

    rTitle = ctk.CTkLabel(rwin,text=algorithm,font=("Rockwell Extra Bold",50))
    rTitle.pack(pady=40)

    rTheatsLabel = ctk.CTkLabel(rwin,text="Threats",font=("Helvetica",20))
    rTheatsLabel.pack(pady=5)
    rTheats = ctk.CTkLabel(rwin,text=num_threats,font=("Helvetica",15), bg_color= "red")
    rTheats.pack(pady=20)

    rleakersLabel = ctk.CTkLabel(rwin,text="Leakers",font=("Helvetica",20))
    rleakersLabel.pack(pady=5)
    rleakersLabel = ctk.CTkLabel(rwin,text=algorithm_num_leakers,font=("Helvetica",15), bg_color= "red")
    rleakersLabel.pack(pady=20)

    rActualLeakageLabel = ctk.CTkLabel(rwin,text="Actual Leakage",font=("Helvetica",20))
    rActualLeakageLabel.pack(side="left", padx=5)
    rActualLeakage = ctk.CTkLabel(rwin,text=f"{simulation_leaker_percentage}%",font=("Helvetica",15), bg_color= "red")
    rActualLeakage.pack(side="left", padx=20)

    rPredictedLeakageLabel = ctk.CTkLabel(rwin,text="Predicted Leakage",font=("Helvetica",20))
    rPredictedLeakageLabel.pack(side="right", padx=5)
    rPredictedLeakage = ctk.CTkLabel(rwin,text=f"{algorithm_leaker_percentage}%",font=("Helvetica",15), bg_color= "red")
    rPredictedLeakage.pack(side="right", padx=20)

    # rleakerPercentageLabel = ctk.CTkLabel(rwin,text="Leaker percentage",font=("Helvetica",20))
    # rleakerPercentageLabel.pack(pady=5)
    # rleakerPercentageLabel = ctk.CTkLabel(rwin,text=f"{leaker_percentage}%",font=("Helvetica",15), bg_color= "red")
    # rleakerPercentageLabel.pack(pady=20)

    rwin.mainloop()

def updateThreatLabel(value):
    currentThreatLabel.configure(text=f"Current Number of Threats: {int(value)}")

#def minSlider(value):
#    minLabel.configure(text=f"Min: {int(value)}")

#def maxSlider(value):
#    maxLabel.configure(text=f"Max: {int(value)}")

def angleSlider(value):
    angleLabel.configure(text=f"Angle: {int(value)} degree")

savedList = read_history(historyFile)

logo = ctk.CTkImage(light_image=Image.open('BOWSER LOGO FINAL.png'),size=(150,150))
logoLabel = ctk.CTkLabel(root, text="", image = logo).place(x=10,y=10)

#yrh = ctk.CTkImage(light_image=Image.open('BOWSER LOGO FINAL.png'),size=(40,40))
#yrhImg = ctk.CTkLabel(root, text="", image = yrh).place(x=350,y=180)
#yrhLabel = ctk.CTkLabel(root,text="You are here",font=("Helvetica",10),height=10).place(x=343,y=220)

title = ctk.CTkLabel(root,text="B.O.W.S.E.R.",font=("Rockwell Extra Bold",50))
title.pack(pady=40)

canvas = tk.Canvas(root, width=500, height=500, borderwidth=0, highlightthickness=0,bg="black")
canvas.pack(pady=20)
canvas.place(x=720,y=200)

def _create_circle(self, x, y, r, **kwargs):
    return self.create_oval(x-r, y-r, x+r, y+r, **kwargs)
tk.Canvas.create_circle = _create_circle

canvas.create_circle(250, 250, 10, fill="#BBB", outline="")


def showRange():
    canvas = tk.Canvas(root, width=500, height=500, borderwidth=0, highlightthickness=0,
                    bg="black")
    canvas.pack(pady=20)
    canvas.place(x=720,y=200)

    rangemin = int(rminEntry.get())
    rangemax = int(rmaxEntry.get())
    angle = int(angleEntry.get())
    unit = unitDropdown.get()

    startAngle = 90 - (angle/2)
    endAngle = 90 + (angle/2)
    rSize = 10

    if rangemax > 900:
        rangemax /= 9
        rangemin /= 9
        rSize /= 9
        unitLabel.configure(text=f"|______| 450 {unit}")
    elif rangemax > 300:
        rangemax /= 3
        rangemin /= 3
        rSize /= 3
        unitLabel.configure(text=f"|______| 150 {unit}")
    else:
        unitLabel.configure(text=f"|______| 50 {unit}")

    def _create_circle(self, x, y, r, **kwargs):
        return self.create_oval(x-r, y-r, x+r, y+r, **kwargs)
    tk.Canvas.create_circle = _create_circle

    def _create_circle_arc(self, x, y, r, **kwargs):
        if "start" in kwargs and "end" in kwargs:
            kwargs["extent"] = kwargs.pop("end") - kwargs["start"]
        return self.create_arc(x-r, y-r, x+r, y+r, **kwargs)
    tk.Canvas.create_circle_arc = _create_circle_arc

    canvas.create_circle_arc(250, 250, rangemax, fill="green", outline="", start=startAngle, end=endAngle)
    canvas.create_circle_arc(250, 250, rangemin, fill="red", outline="", start=startAngle, end=endAngle)
    canvas.create_circle(250, 250, rSize, fill="#BBB", outline="")

rLabel = ctk.CTkLabel(root, text="Spawn Range")
rLabel.pack(pady=5)
rLabel.place(x=350,y=150)
rminEntry = ctk.CTkEntry(root, placeholder_text="Min")
rminEntry.pack(pady=10)
rminEntry.place(x=320,y=190)
rmaxEntry = ctk.CTkEntry(root, placeholder_text="Max")
rmaxEntry.pack(pady=10)
rmaxEntry.place(x=320,y=230)

angleEntry = ctk.CTkSlider(root, from_=1, to=359, command=angleSlider)
angleEntry.pack(pady=2)
angleEntry.set(1)
angleEntry.place(x=290,y=270)
angleLabel = ctk.CTkLabel(root, text=f"Angle: {angleEntry.get()} degree")
angleLabel.pack(pady=2)
angleLabel.place(x=345,y=290)

rangeButton = ctk.CTkButton(root, text="View Range", command=showRange)
rangeButton.pack(pady=5)
rangeButton.place(x=320,y=325)

radarLabel = ctk.CTkLabel(root, text="Visualize Spawn Range")
radarLabel.pack(pady=5)
radarLabel.place(x=710,y=130)

unitLabel = ctk.CTkLabel(root, text="")
unitLabel.pack(pady=5)
unitLabel.place(x=900, y=560)

scaleLabel = ctk.CTkLabel(root, text="|______|_______|_______|_______|______")
scaleLabel.pack(pady=5)
scaleLabel.place(x=577, y=560)

#minVar = tk.IntVar(value=1)
#rminEntry = ctk.CTkSlider(root, from_=10, to=50, variable=minVar,command=minSlider,progress_color="red")
#rminEntry.pack(pady=2)
#minLabel = ctk.CTkLabel(root, text=f"Min: {rminEntry.get()}")
#minLabel.place(x=600,y=170)
#maxVar = tk.IntVar(value=1)
#rmaxEntry = ctk.CTkSlider(root, from_=10, to=50, variable=maxVar,command=maxSlider,fg_color="red")
#rmaxEntry.pack(pady=10)
#maxLabel = ctk.CTkLabel(root, text=f"Max: {rmaxEntry.get()}")
#maxLabel.place(x=600,y=200)

unitOptions = ["m", "km", "ft", "mi"]
unitDropdown = ctk.CTkOptionMenu(root, values=unitOptions,width=60)
unitDropdown.pack(pady=10)
unitDropdown.place(x=480,y=210)

algoLabel = ctk.CTkLabel(root, text="Select Algorithm")
algoLabel.pack(pady=5)
algoLabel.place(x=345,y=370)
algoOptions = ["DQN", "Genetic Algorithm", "Munkres", "Simulated Annealing"]
algoDropdown = ctk.CTkOptionMenu(root, values=algoOptions)
algoDropdown.pack(pady=10)
algoDropdown.place(x=320,y=410)

threatVar = tk.IntVar(value=1)
threatScale = ctk.CTkSlider(root, from_=1, to=10, variable=threatVar, command=updateThreatLabel)
threatScale.pack(pady=10)
threatScale.place(x=290,y=450)
currentThreatLabel = ctk.CTkLabel(root, text=f"Current Number of Threats: {threatVar.get()}")
currentThreatLabel.pack(pady=5)
currentThreatLabel.place(x=310,y=470)

submitButton = ctk.CTkButton(root, text="Start", command=submit)
submitButton.pack(pady=5)
submitButton.place(x=320,y=510)

outputLabel = ctk.CTkLabel(root, text="", wraplength=800)
outputLabel.pack(side="bottom", pady=10)

#dleaker = ctk.CTkLabel(root, text="Deep Q-Learning: No Tracked Runs in Current Session", wraplength=200)
#dleaker.pack(side="right")
#dleaker.place(x=800,y=150)

#gleaker = ctk.CTkLabel(root, text="Genetic Algorithm: No Tracked Runs in Current Session", wraplength=200)
#gleaker.pack(side="right")
#gleaker.place(x=800,y=190)

#mleaker = ctk.CTkLabel(root, text="Munkres Algorithm: No Tracked Runs in Current Session", wraplength=200)
#mleaker.pack(side="right")
#mleaker.place(x=800,y=230)

#sleaker = ctk.CTkLabel(root, text="Simulated Annealing: No Tracked Runs in Current Session", wraplength=200)
#sleaker.pack(side="right")
#sleaker.place(x=800,y=270)

leaderBoard = ctk.CTkLabel(root, text="\n".join(savedList.split('\n')[:10]), anchor="e", justify=RIGHT, wraplength=400)
leaderBoard.pack(side="left")
leaderBoard.place(x=50,y=210)

root.mainloop()