from algorithms.dqn_agent import runDQN
# from algorithms.dqn_agent1 import runDQN
from algorithms.geneticAlgorithmTest import runGA
from algorithms.munkres_algorithm import runMunkres
from algorithms.simulated_annealing import runSimulatedAnnealing
from algorithms.ACO import runACO
from tkinter import *
from PIL import Image, ImageTk
import customtkinter as ctk
import tkinter as tk
import pandas as pd
import random 
import time
import csv
from sim_app import simulate_BOWSER_simulation
import shutil

try:
    from app import run_BOWSER_simulation
    RUN_SIMULATION = False # change to Ture for running simulation or False for not
except:
    RUN_SIMULATION = False
    

threat_coordinates = {}

# import files
masterTemplate =  "dataFiles/threat_location_original.csv"
threatFileLocation = "dataFiles/threat_location.csv"
weaponFileOriginal = "dataFiles/weapon_data_original.csv"
weaponFileLocation = "dataFiles/weapon_data.csv"
dqnModelPath = "dataFiles/trained_model.zip"
historyFile = "dataFiles/history.csv"

shutil.copyfile(weaponFileOriginal, weaponFileLocation)

# set GUI theme
ctk.set_appearance_mode("dark")  # Modes: system (default), light, dark
ctk.set_default_color_theme("blue")  # Themes: blue (default), dark-blue, green

# inital main window
root = ctk.CTk()
root.title('B.O.W.S.E.R.')
root.geometry('1250x750')

# write name and leaker percentage to history file
def write_history(history_file,list):
    fieldnames = ['algorithm_name', 'leaker_percentage']
    new_list = {'algorithm_name': list[0], 'leaker_percentage': list[1]}
    with open(history_file,'a',newline='') as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writerow(new_list)
        file.close()

# read history file and sort by leaker percentage
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

# all thing that happen after hit the start button
def submit():
    outputLabel.configure(text="")

# get values from the inputs
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

# run simulation with or without gazebo    
    if RUN_SIMULATION:
        simulation_leaker_percentage, algorithm_leaker_percentage = run_BOWSER_simulation(spawnRange=(rangemin, rangemax), algorithmChoice=algorithm, numberOfDrones=num_threats)
    else:
        simulation_leaker_percentage, algorithm_leaker_percentage = simulate_BOWSER_simulation(spawnRange=(rangemin, rangemax), algorithmChoice=algorithm, numberOfDrones=num_threats, threatCoordinates=threat_coordinates)

# call write and read history function        
    save_list = [algorithm,simulation_leaker_percentage]
    write_history(historyFile,save_list)
    savedList = read_history(historyFile)

# update leaderboard
    leaderBoard.configure(text="\n".join(savedList.split('\n')[:10]))

# result window
    rwin = ctk.CTk()
    rwin.title('Your Result')
    rwin.geometry('700x500')

    algorithm_num_leakers = int(num_threats * algorithm_leaker_percentage * 0.01)
    simulation_num_leakers = int(num_threats * simulation_leaker_percentage * 0.01)
    
    rTitle = ctk.CTkLabel(rwin,text=algorithm,font=("Rockwell Extra Bold",50))
    rTitle.pack(pady=40)

# number of threat
    rTheatsLabel = ctk.CTkLabel(rwin,text="Threats",font=("Helvetica",20))
    rTheatsLabel.pack(pady=5)
    rTheats = ctk.CTkLabel(rwin,text=num_threats,font=("Helvetica",15), bg_color= "red")
    rTheats.pack(pady=20)

# number of leaker
    rleakersLabel = ctk.CTkLabel(rwin,text="Leakers",font=("Helvetica",20))
    rleakersLabel.pack(pady=5)
    rleakersLabel = ctk.CTkLabel(rwin,text=algorithm_num_leakers,font=("Helvetica",15), bg_color= "red")
    rleakersLabel.pack(pady=20)

# leaker percentage from simulation
    rActualLeakageLabel = ctk.CTkLabel(rwin,text="Actual Leakage",font=("Helvetica",20))
    rActualLeakageLabel.pack(side="left", padx=5)
    rActualLeakage = ctk.CTkLabel(rwin,text=f"{simulation_leaker_percentage}%",font=("Helvetica",15), bg_color= "red")
    rActualLeakage.pack(side="left", padx=20)

# leaker percentage from algorithm
    rPredictedLeakageLabel = ctk.CTkLabel(rwin,text="Predicted Leakage",font=("Helvetica",20))
    rPredictedLeakageLabel.pack(side="right", padx=5)
    rPredictedLeakage = ctk.CTkLabel(rwin,text=f"{algorithm_leaker_percentage}%",font=("Helvetica",15), bg_color= "red")
    rPredictedLeakage.pack(side="right", padx=20)

    rwin.mainloop()

# show update of input
def updateThreatLabel(value):
    currentThreatLabel.configure(text=f"Current Number of Threats: {int(value)}")

def angleSlider(value):
    angleLabel.configure(text=f"Angle: {int(value)} degree")

def directionSlider(value):
    dirLabel.configure(text=f"Direction: {int(value)} degree")

# for inital leaderboard
savedList = read_history(historyFile)

# BOWSER logo
logo = ctk.CTkImage(light_image=Image.open('BOWSER_LOGO_FINAL.png'),size=(150,150))
logoLabel = ctk.CTkLabel(root, text="", image = logo).place(x=10,y=10)

# BOWSER title
title = ctk.CTkLabel(root,text="B.O.W.S.E.R.",font=("Rockwell Extra Bold",50))
title.pack(pady=40)
title.place(x=425,y=25)

# inital radar screen
canvas = tk.Canvas(root, width=500, height=500, borderwidth=0, highlightthickness=0,bg="black")
canvas.pack(pady=20)
canvas.place(x=720,y=200)

def _create_circle(self, x, y, r, **kwargs):
    return self.create_oval(x-r, y-r, x+r, y+r, **kwargs)
tk.Canvas.create_circle = _create_circle

canvas.create_circle(250, 250, 10, fill="#BBB", outline="")

# update radar screen
def showRange(event):
    canvas = tk.Canvas(root, width=500, height=500, borderwidth=0, highlightthickness=0,
                    bg="black")
    canvas.pack(pady=20)
    canvas.place(x=720,y=200)

# get values
    rangemin = int(rminEntry.get())
    rangemax = int(rmaxEntry.get())
    angle = int(angleEntry.get())
    direction = int(dirSlider.get())
    unit = unitDropdown.get()

# define radar angle and direction
    startAngle = direction - (angle/2)
    endAngle = direction + (angle/2)
    rSize = 10

# change scale of radar screen by rangemax
    if rangemax > 900:
        rangemax /= 9
        rangemin /= 9
        rSize /= 9
        unitLabel.configure(text=f"|________| 450 {unit}")
    elif rangemax > 300:
        rangemax /= 3
        rangemin /= 3
        rSize /= 3
        unitLabel.configure(text=f"|________| 150 {unit}")
    else:
        unitLabel.configure(text=f"|________| 50 {unit}")

# create radar circles
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

# new window for more feature button
def openNewWindow():
    mwin = ctk.CTk()
    mwin.title('More Features')
    mwin.geometry('1200x700')
    ctk.CTkLabel(mwin, text="Spawn Coordinates", font=("Helvetica", 20, "bold")).pack(pady=10)

    # rangemin = float(rminEntry.get())
    # rangemax = float(rmaxEntry.get())
    global threat_coordinates

# read and save threat coordinates
    def saveThreatCoordinates():
        global threat_coordinates
        updated_threat_coordinates = {}
        for i in range(1, threatVar.get() + 1):
            x = float(threat_coordinates[f"threat_{i}_x"].get())
            y = float(threat_coordinates[f"threat_{i}_y"].get())
            z = float(threat_coordinates[f"threat_{i}_z"].get())
            min_range = float(threat_coordinates[f"threat_{i}_min_range"].get())  
            max_range = float(threat_coordinates[f"threat_{i}_max_range"].get())
            if min_range <= abs(x) <= max_range and min_range <= abs(y) <= max_range and 0 <= abs(z):
                print(f"Threat {i}: x={x}, y={y}, z={z}")
                updated_threat_coordinates[i] = {"x": x, "y": y, "z": z, "min_range": min_range, "max_range": max_range}
            else:
                print(f"Coordinates for Threat {i} are not within the range.")
        threat_coordinates = updated_threat_coordinates
        print(threat_coordinates)

    # Current bug where the sliders position affects the threat count by one. 
    # Just move it slightly to the left wihtout changing the number and itll work.
    for i in range(1, threatVar.get() + 1):
        frame = tk.Frame(mwin)
        frame.pack(pady=2)
        
        tk.Label(frame, text=f"Threat {i}").pack(side=tk.LEFT)
        tk.Label(frame, text="x:").pack(side=tk.LEFT)
        threat_coordinates[f"threat_{i}_x"] = tk.Entry(frame)
        threat_coordinates[f"threat_{i}_x"].pack(side=tk.LEFT, pady=2)
        
        tk.Label(frame, text="y:").pack(side=tk.LEFT)
        threat_coordinates[f"threat_{i}_y"] = tk.Entry(frame)
        threat_coordinates[f"threat_{i}_y"].pack(side=tk.LEFT, pady=2)
        
        tk.Label(frame, text="z:").pack(side=tk.LEFT)
        threat_coordinates[f"threat_{i}_z"] = tk.Entry(frame)
        threat_coordinates[f"threat_{i}_z"].pack(side=tk.LEFT, pady=2)

        tk.Label(frame, text="Min Range:").pack(side=tk.LEFT)  
        threat_coordinates[f"threat_{i}_min_range"] = tk.Entry(frame) 
        threat_coordinates[f"threat_{i}_min_range"].pack(side=tk.LEFT, pady=2)

        tk.Label(frame, text="Max Range:").pack(side=tk.LEFT)  
        threat_coordinates[f"threat_{i}_max_range"] = tk.Entry(frame)
        threat_coordinates[f"threat_{i}_max_range"].pack(side=tk.LEFT, pady=2)

    tk.Button(mwin, text="Save Threat Coordinates", command=saveThreatCoordinates).pack(pady=10)

    ctk.CTkLabel(mwin, text="Set Weapon Coordinates", font=("Helvetica", 20, "bold")).pack(pady=10)
    weapon_coordinates = {}

# read and save weapon coordinates
    def saveWeaponCoordinates():
        with open(weaponFileLocation, 'r') as file:
            lines = file.readlines()
        with open(weaponFileLocation, 'w') as file:
            for line in lines:
                split_line = line.split(',')
                weapon_name = split_line[0]
                if weapon_name in weapon_coordinates:
                    coords = weapon_coordinates[weapon_name]          
                    split_line[1] = str(coords['x'].get())
                    split_line[2] = str(coords['y'].get())
                    updated_line = ','.join(split_line)
                    file.write(updated_line)
                else:
                    file.write(line)
        print("Weapon coordinates updated.")

    for weapon in ["long range missile", "medium range missile", "short range missile", "directed energy"]:
        frame = tk.Frame(mwin)
        frame.pack(pady=2)
        
        tk.Label(frame, text=f"{weapon}").pack(side=tk.LEFT)
        tk.Label(frame, text="x:").pack(side=tk.LEFT)
        weapon_coordinates[weapon] = {}
        weapon_coordinates[weapon]['x'] = tk.Entry(frame)
        weapon_coordinates[weapon]['x'].pack(side=tk.LEFT, pady=2)
        
        tk.Label(frame, text="y:").pack(side=tk.LEFT)
        weapon_coordinates[weapon]['y'] = tk.Entry(frame)
        weapon_coordinates[weapon]['y'].pack(side=tk.LEFT, pady=2)

    tk.Button(mwin, text="Save Weapon Coordinates", command=saveWeaponCoordinates).pack(pady=10)

    mwin.mainloop()

# spawn range input section
rLabel = ctk.CTkLabel(root, text="Spawn Range", font=("Helvetica", 20, "bold"))
rLabel.pack(pady=5)
rLabel.place(x=325,y=150)

rminEntry = ctk.CTkEntry(root, placeholder_text="Min")
rminEntry.pack(pady=10)
rminEntry.place(x=320,y=190)
rminEntry.bind("<KeyRelease>",showRange)

rmaxEntry = ctk.CTkEntry(root, placeholder_text="Max")
rmaxEntry.pack(pady=10)
rmaxEntry.place(x=320,y=230)
rmaxEntry.bind("<KeyRelease>",showRange)

# the circle will overlap itself and disappear for angle equal 360
angleEntry = ctk.CTkSlider(root, from_=1, to=359, command=angleSlider)
angleEntry.pack(pady=2)
angleEntry.set(1)
angleEntry.place(x=290,y=270)
angleEntry.bind("<ButtonRelease>",showRange)

angleLabel = ctk.CTkLabel(root, text=f"Angle: {angleEntry.get()} degree")
angleLabel.pack(pady=2)
angleLabel.place(x=345,y=290)

dirSlider = ctk.CTkSlider(root, from_=0, to=360, command=directionSlider)
dirSlider.pack(pady=2)
dirSlider.set(90)
dirSlider.place(x=290,y=320)
dirSlider.bind("<ButtonRelease>",showRange)

dirLabel = ctk.CTkLabel(root, text=f"Direction: {dirSlider.get()} degree")
dirLabel.pack(pady=2)
dirLabel.place(x=345,y=340)

unitOptions = ["m", "km", "ft", "mi"]
unitDropdown = ctk.CTkOptionMenu(root, values=unitOptions,width=60)
unitDropdown.pack(pady=10)
unitDropdown.place(x=480,y=210)

# radar labels section
radarLabel = ctk.CTkLabel(root, text="Visualize Spawn Range", font=("Helvetica", 20, "bold"))
radarLabel.pack(pady=5)
radarLabel.place(x=720,y=150)

unitLabel = ctk.CTkLabel(root, text="")
unitLabel.pack(pady=5)
unitLabel.place(x=921, y=700)

scaleLabel = ctk.CTkLabel(root, text="|________|________|________|________|")
scaleLabel.pack(pady=5)
scaleLabel.place(x=721, y=700)

# algorithm and threat input section
algoLabel = ctk.CTkLabel(root, text="Select Algorithm", font=("Helvetica", 20, "bold"))
algoLabel.pack(pady=5)
algoLabel.place(x=313,y=380)
algoOptions = ["DQN", "Genetic Algorithm", "Munkres", "Simulated Annealing", "ACO"]
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

# threat and weapon coordinates input section
moreFeaturesButton = ctk.CTkButton(root, text="More Features", command=openNewWindow)
moreFeaturesButton.pack(pady=5)
moreFeaturesButton.place(x=320,y=510)

# start button
submitButton = ctk.CTkButton(root, text="Start", font=("Helvetica", 20, "bold"), text_color="red", command=submit)
submitButton.pack(pady=5)
submitButton.place(x=320,y=550)

outputLabel = ctk.CTkLabel(root, text="", wraplength=800)
outputLabel.pack(pady=10)
outputLabel.place(x=320,y=700)

# display leaderboard
leaderBoard = ctk.CTkLabel(root, text="\n".join(savedList.split('\n')[:10]), anchor="e", justify=RIGHT, wraplength=400)
leaderBoard.pack(side="left")
leaderBoard.place(x=50,y=210)

root.mainloop()