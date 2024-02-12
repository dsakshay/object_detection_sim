from datetime import datetime 

now = datetime.now()

filename = now.strftime("%d/%m/%Y-%H:%M:%S")

print(filename)
