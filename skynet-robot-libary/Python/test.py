write_train = ""
write_test = ""

with open("optimization_data.csv", "r") as rf:
    lines = rf.read().split("\n")[1::]
    for index, line in enumerate(lines):
        if index % 4 == 3:
            write_test += line + "\n" 
        else:
            write_train += line + "\n"

with open("optimization_test.csv", "w") as wf:
    wf.write(write_test)
with open("optimization_train.csv", "w") as wf:
    wf.write(write_train)