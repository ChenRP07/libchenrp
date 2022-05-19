import json
import sys

if __name__ == '__main__':
    print("Load json file...")
    global_ = open("global_compile.json")
    global_file = json.load(global_)
    filename = sys.argv[1] + "/build/compile_commands.json"
    local_ = open(filename)
    local_file = json.load(local_)
    print(filename)
    print("Change commands...")
    merge=global_file[0]['command']
    merge_value=merge.split()
    for file in local_file:
        old = file["command"]
        old_value = old.split()
        add_value = []
        for v in merge_value:
            if v != '-isystem' and old_value.count(v) == 0:
                add_value.append('-isystem')
                add_value.append(v)
        #print(add_value)
        new_value = []
        for v in old_value:
            new_value.append(v)
            if v.find('-I') != -1:
                for i in add_value:
                    new_value.append(i)
        
        new = ""
        for v in new_value:
            new = new + ' ' + v
        new = new[1:]
        file["command"] = new
    print("Write json file...")
    with open(filename, 'w') as f:
        f.write(json.dumps(local_file, indent=4, ensure_ascii=False))
