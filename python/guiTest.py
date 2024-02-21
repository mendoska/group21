from algorithms.dqn_agent import runDQN
from algorithms.geneticAlgorithmTest import runGA
from algorithms.munkres_algorithm import runMunkres
from algorithms.simulated_annealing import runSimulatedAnnealing
from time import time
import tkinter as tk
from tkinter import *
from tkinter import ttk

threatFileLocation="dataFiles/threat_location.csv"
weaponFileLocation="dataFiles/weapon_data.csv"

window = tk.Tk()
window.title(" Algorithms ")

window.geometry("900x1000")

def selectAlgorithm(algorithmChoice):
    if algorithmChoice == None:
        algorithmChoice = input("""
        Please Input 1 - 4 to Select Algorithm:
        ---------------------------------------
        1: Deep Q Network
        2: Genetic Algorithm
        3: Munkres (Hungarian)
        4: Simulated Annealing \n""")


    if algorithmChoice == "1":
        runDQN()
    elif algorithmChoice == "2":
        res = runGA()
        printOutputList(outputList=res)
    elif algorithmChoice == "3":
        res = runMunkres(threatFileLocation=threatFileLocation,weaponFileLocation=weaponFileLocation)
        printOutputList(outputList=res)    
    elif algorithmChoice == "4":
        res = runSimulatedAnnealing()
        printOutputList(outputList=res)    
    else:
        print("Invalid Input")
    
    

def printOutputList(outputList: list):
    text = tk.Text(master=window, height=100, width=100)
    text.grid(column=10, row=10)
    for entry in outputList:
        print(entry)
        text.insert(tk.END, str(entry) + '\n')

def testTimeForSingleAlgorithm(selection:str) -> float:
    start_time = time()
    selectAlgorithm(selection)
    finish_time = time()
    return finish_time - start_time

def testTimeAllArrays():
    r = []
    for i in range(4):
        r.append(f'{i+1}: {testTimeForSingleAlgorithm(f"{i+1}")} Seconds')
    printOutputList(outputList=r)

algorithmChoice=[]

def on_click(text,algorithmChoice):
    algorithmChoice.clear()
    algorithmChoice.append(text)
    print("algorithm choice is is:", algorithmChoice)
    if text != '5':
        selectAlgorithm(str(algorithmChoice[0]))
    else:
        testTimeAllArrays()



lb=Label(window, text="Choose the algorithm you would like to run or test time all arrays.")
lb.grid(column=0, row=0)


b1=ttk.Button(window, text= "1. Deep Q Network", command=lambda:on_click("1",algorithmChoice))
b1.grid(column=0, row=1)

b2=ttk.Button(window, text= "2. Genetic Algorithm", command=lambda: on_click("2",algorithmChoice))
b2.grid(column=0, row=2)

b3=ttk.Button(window, text= "3. Munkres (Hungarian)", command=lambda: on_click("3",algorithmChoice))
b3.grid(column=0, row=3)

b4=ttk.Button(window, text= "4. Simulated Annealing", command=lambda: on_click("4",algorithmChoice))
b4.grid(column=0, row=4)

b5=ttk.Button(window, text= "Test Time All Arrays", command=lambda: on_click("5",algorithmChoice))
b5.grid(column=0, row=5)



window.mainloop()


#selectAlgorithm()
#testTimeAllArrays()
